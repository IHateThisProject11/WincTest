/* Core/Src/Uploader.c
 * Stream a FatFS file to a remote host using the Microchip WINC1500 socket API.
 * - Raw TCP (Uploader_SendFile)
 * - HTTP POST (Uploader_SendFileHTTP)
 * - Raw TCP by hostname with DNS (Uploader_SendFileHost)
 *
 * Single-call / non-reentrant by design.
 */

#include "Uploader.h"
#include "ff.h"
#include "stm32h5xx_hal.h"
#include "m2m_wifi.h"

#ifdef __has_include
#  if __has_include("socket/include/socket.h")
#    include "socket/include/socket.h"
#  elif __has_include("socket.h")
#    include "socket.h"
#  else
#    error "socket.h not found in include paths"
#  endif
#else
#  include "socket.h"
#endif

#include <stdio.h>
#include <string.h>

/* ---------------- Tunables ---------------- */
#ifndef UPLOADER_CHUNK
#define UPLOADER_CHUNK            1024u     /* bytes per send() */
#endif

#ifndef UPLOADER_EVENT_POLL_MS
#define UPLOADER_EVENT_POLL_MS       1u
#endif

/* ---------------- State ---------------- */
typedef enum {
    ST_IDLE = 0,
    ST_CONNECTING,
    ST_SENDING_HDR,
    ST_SENDING_FILE,
    ST_DONE,
    ST_ERROR
} uploader_state_t;

static volatile uploader_state_t s_state = ST_IDLE;
static volatile int              s_sock  = -1;
static FIL                       s_fil;
static UINT                      s_br    = 0;        /* unread bytes remaining in s_buf */
static uint8_t                   s_buf[UPLOADER_CHUNK];
static uint32_t                  s_total_sent = 0;
static int                       s_errno = 0;
static uint32_t                  s_deadline = 0;     /* 0 = no timeout */

/* HTTP header buffer */
static char                      s_hdr[192];
static uint16_t                  s_hdr_len = 0;
static uint16_t                  s_hdr_off = 0;

/* DNS (for hostname variant) */
static volatile uint32_t         s_resolved_ip = 0;  /* network byte order */
static volatile uint8_t          s_dns_done    = 0;

/* ---------------- Helpers ---------------- */
static inline uint32_t _millis(void) { return HAL_GetTick(); }

static int _expired(uint32_t dl_ms) {
    return (dl_ms != 0u) && (_millis() >= dl_ms);
}

static void _set_error(int code) {
    s_errno = code;
    s_state = ST_ERROR;
    if (s_sock >= 0) {
        close(s_sock);
        s_sock = -1;
    }
    /* Ensure file is closed */
    f_close(&s_fil);
}

static int _read_next_chunk(void) {
    UINT n = 0;
    FRESULT fr = f_read(&s_fil, s_buf, sizeof(s_buf), &n);
    if (fr != FR_OK) {
        printf("Uploader: f_read error %u\r\n", (unsigned)fr);
        _set_error(-30);
        return -1;
    }
    s_br = n;
    return (n > 0) ? 1 : 0; /* 1 = got data, 0 = EOF */
}

static void _try_send_next(void) {
    if (s_state == ST_SENDING_HDR) {
        if (s_hdr_off < s_hdr_len) {
            uint16 to_send = (uint16)(s_hdr_len - s_hdr_off);
            if (to_send > sizeof(s_buf)) to_send = sizeof(s_buf);
            memcpy(s_buf, &s_hdr[s_hdr_off], to_send);
            sint16 rc = send(s_sock, s_buf, to_send, 0);
            if (rc < 0) {
                printf("Uploader: send(header) rc=%d\r\n", rc);
                _set_error(-40);
                return;
            }
            return; /* progress handled in SOCKET_MSG_SEND */
        } else {
            s_state = ST_SENDING_FILE;
        }
    }

    if (s_state == ST_SENDING_FILE) {
        if (s_br == 0) {
            int have = _read_next_chunk();
            if (have < 0) return; /* error handled */
            if (have == 0) {
                /* EOF */
                close(s_sock);
                s_sock = -1;
                f_close(&s_fil);
                s_state = ST_DONE;
                return;
            }
        }
        sint16 rc = send(s_sock, s_buf, (uint16)s_br, 0);
        if (rc < 0) {
            printf("Uploader: send(data) rc=%d\r\n", rc);
            _set_error(-41);
            return;
        }
        return; /* progress handled in SOCKET_MSG_SEND */
    }
}

// Add near top:
static const char* _sock_err(int e){
    switch(e){
    case -1:  return "general failure";
    case -2:  return "invalid address";
    case -3:  return "address in use";
    case -4:  return "max sockets";
    case -5:  return "invalid arg";
    case -6:  return "addr already";
    case -7:  return "timeout";
    case -8:  return "busy";
    case -9:  return "invalid";
    case -10: return "abort";
    case -11: return "reset";
    case -12: return "timeout/refused";
    default:  return "?";
    }
}
static void _print_ip_be(uint32_t be){
    uint8_t a = (be >> 24) & 0xFF, b = (be >> 16) & 0xFF, c = (be >> 8) & 0xFF, d = be & 0xFF;
    printf("DNS: %u.%u.%u.%u\r\n", a,b,c,d);
}


/* ---------------- Socket callbacks ---------------- */
static void uploader_socket_cb(SOCKET sock, uint8 u8Msg, void *pvMsg) {
    (void)sock;
    switch (u8Msg) {
    case SOCKET_MSG_CONNECT: {
        tstrSocketConnectMsg *p = (tstrSocketConnectMsg *)pvMsg;
        if (p && p->s8Error == 0) {
            if (s_state == ST_CONNECTING) {
                s_state = (s_hdr_len > 0) ? ST_SENDING_HDR : ST_SENDING_FILE;
                s_br = 0;
                _try_send_next();
            }
        } else {
            printf("Uploader: connect failed err=%d\r\n", p ? p->s8Error : -1);
            _set_error(-20);
        }
        break;
    }
    case SOCKET_MSG_SEND: {
        sint16 *psent = (sint16 *)pvMsg;
        if (!psent || *psent < 0) {
            printf("Uploader: send cb err=%d\r\n", psent ? *psent : -1);
            _set_error(-42);
            break;
        }
        sint16 sent = *psent;

        if (s_state == ST_SENDING_HDR) {
            s_hdr_off += (uint16)sent;
            _try_send_next();
        } else if (s_state == ST_SENDING_FILE) {
            s_total_sent += (uint32_t)sent;
            if ((uint32_t)sent >= s_br) {
                s_br = 0; /* chunk fully consumed */
            } else {
                memmove(s_buf, s_buf + sent, s_br - (uint16)sent);
                s_br -= (uint16)sent;
            }
            _try_send_next();
        }
        break;
    }
    default:
        break;
    }
}

/* DNS resolve callback (two-callback API; avoids tstrDnsReply type entirely) */
static void uploader_dns_cb(uint8 *pu8HostName, uint32 u32HostIp) {
    (void)pu8HostName;
    s_resolved_ip = u32HostIp;   // network byte order
    s_dns_done = 1;
    _print_ip_be(s_resolved_ip);
}


/* Common connect/stream loop (socket already set up for IP case) */
static int uploader_run_loop(uint32_t timeout_ms) {
    s_deadline = (timeout_ms ? (_millis() + timeout_ms) : 0u);

    while (1) {
        m2m_wifi_handle_events(NULL);

        if (s_state == ST_DONE)  return 0;
        if (s_state == ST_ERROR) return s_errno ? s_errno : -99;
        if (_expired(s_deadline)) {
            printf("Uploader: timeout\r\n");
            _set_error(-12);
            return s_errno;
        }
        HAL_Delay(UPLOADER_EVENT_POLL_MS);
    }
}

/* ---------------- Public API ---------------- */

int Uploader_SendFile(const char *fatfs_path, const char *pc_ip, uint16_t port, uint32_t timeout_ms)
{
    if (!fatfs_path || !pc_ip) return -1;

    FRESULT fr = f_open(&s_fil, fatfs_path, FA_READ);
    if (fr != FR_OK) {
        printf("Uploader: f_open('%s') error %u\r\n", fatfs_path, (unsigned)fr);
        return -2;
    }

    /* Build sockaddr from dotted IP string */
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_port        = _htons(port);
    addr.sin_addr.s_addr = nmi_inet_addr((char *)pc_ip);

    /* Init socket layer + callbacks (idempotent) */
    socketInit();
    registerSocketCallback(uploader_socket_cb, uploader_dns_cb);

    /* No header (raw) */
    s_hdr_len = 0;
    s_hdr_off = 0;

    /* Create + connect */
    s_state = ST_CONNECTING;
    s_errno = 0;
    s_total_sent = 0;
    s_br = 0;

    s_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (s_sock < 0) {
        printf("Uploader: socket() failed (%d)\r\n", s_sock);
        _set_error(-10);
        return s_errno;
    }
    sint8 rc = connect(s_sock, (struct sockaddr *)&addr, sizeof(addr));
    if (rc < 0) {
        printf("Uploader: connect() rc=%d\r\n", rc);
        _set_error(-11);
        return s_errno;
    }

    /* Run */
    int r = uploader_run_loop(timeout_ms);
    f_close(&s_fil);
    return r;
}

int Uploader_SendFileHTTP(const char *fatfs_path, const char *pc_ip, uint16_t port,
                          const char *uri_path, const char *content_type, uint32_t timeout_ms)
{
    if (!fatfs_path || !pc_ip) return -1;

    /* Get file size for Content-Length */
    FILINFO finfo;
    memset(&finfo, 0, sizeof(finfo));
    FRESULT frs = f_stat(fatfs_path, &finfo);
    if (frs != FR_OK) {
        printf("Uploader: f_stat('%s') error %u\r\n", fatfs_path, (unsigned)frs);
        return -2;
    }
    unsigned long fsize = (unsigned long)finfo.fsize;

    FRESULT fr = f_open(&s_fil, fatfs_path, FA_READ);
    if (fr != FR_OK) {
        printf("Uploader: f_open('%s') error %u\r\n", fatfs_path, (unsigned)fr);
        return -3;
    }

    /* Build HTTP header */
    const char *ct  = content_type ? content_type : "text/plain";
    const char *uri = (uri_path && *uri_path) ? uri_path : "upload";
    int n = snprintf(s_hdr, sizeof(s_hdr),
                     "POST /%s HTTP/1.1\r\n"
                     "Host: %s\r\n"
                     "Content-Type: %s\r\n"
                     "Content-Length: %lu\r\n"
                     "Connection: close\r\n"
                     "\r\n",
                     uri, pc_ip, ct, fsize);
    if (n <= 0 || (size_t)n >= sizeof(s_hdr)) {
        printf("Uploader: header too long (%d)\r\n", n);
        f_close(&s_fil);
        return -4;
    }
    s_hdr_len = (uint16_t)n;
    s_hdr_off = 0;

    /* sockaddr from dotted IP */
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_port        = _htons(port);
    addr.sin_addr.s_addr = nmi_inet_addr((char *)pc_ip);

    /* Init socket + callbacks */
    socketInit();
    registerSocketCallback(uploader_socket_cb, uploader_dns_cb);

    /* Create + connect */
    s_state = ST_CONNECTING;
    s_errno = 0;
    s_total_sent = 0;
    s_br = 0;

    s_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (s_sock < 0) {
        printf("Uploader: socket() failed (%d)\r\n", s_sock);
        f_close(&s_fil);
        _set_error(-10);
        return s_errno;
    }
    sint8 rc = connect(s_sock, (struct sockaddr *)&addr, sizeof(addr));
    if (rc < 0) {
        printf("Uploader: connect() rc=%d\r\n", rc);
        f_close(&s_fil);
        _set_error(-11);
        return s_errno;
    }

    /* Run */
    int r = uploader_run_loop(timeout_ms);
    f_close(&s_fil);
    return r;
}

/* Hostname (DNS) variant for ngrok/tunnels/etc.  Add the prototype to your header if you plan to call it. */
int Uploader_SendFileHost(const char *fatfs_path,
                          const char *host, uint16_t port, uint32_t timeout_ms)
{
    if (!fatfs_path || !host) return -1;

    FRESULT fr = f_open(&s_fil, fatfs_path, FA_READ);
    if (fr != FR_OK) {
        printf("Uploader: f_open('%s') error %u\r\n", fatfs_path, (unsigned)fr);
        return -2;
    }

    socketInit();
    registerSocketCallback(uploader_socket_cb, uploader_dns_cb);

    s_dns_done    = 0;
    s_resolved_ip = 0;

    if (gethostbyname((uint8*)host) != SOCK_ERR_NO_ERROR) {
        f_close(&s_fil);
        return -3; /* DNS start failure */
    }

    /* Wait for DNS result while pumping events */
    uint32_t deadline = timeout_ms ? (_millis() + timeout_ms) : 0;
    while (!s_dns_done) {
        m2m_wifi_handle_events(NULL);
        if (_expired(deadline)) {
            printf("Uploader: DNS timeout\r\n");
            f_close(&s_fil);
            return -12;
        }
        HAL_Delay(UPLOADER_EVENT_POLL_MS);
    }
    if (s_resolved_ip == 0) {
        f_close(&s_fil);
        return -13; /* DNS failed */
    }

    /* No header (raw) */
    s_hdr_len = 0;
    s_hdr_off = 0;

    /* Build sockaddr from resolved IP */
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_port        = _htons(port);
    addr.sin_addr.s_addr = s_resolved_ip; /* already network byte order */

    /* Create + connect */
    s_state = ST_CONNECTING;
    s_errno = 0;
    s_total_sent = 0;
    s_br = 0;

    s_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (s_sock < 0) {
        printf("Uploader: socket() failed (%d)\r\n", s_sock);
        _set_error(-10);
        return s_errno;
    }
    sint8 rc = connect(s_sock, (struct sockaddr *)&addr, sizeof(addr));
    if (rc < 0) {
        printf("Uploader: connect() rc=%d\r\n", rc);
        _set_error(-11);
        return s_errno;
    }

    /* Run */
    int r = uploader_run_loop(timeout_ms);
    f_close(&s_fil);
    return r;
}
