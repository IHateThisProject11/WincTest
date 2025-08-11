/* Core/Src/Uploader.c
 * Stream a FatFS file to a remote host using the Microchip WINC socket API.
 *
 * This module is self-contained. It registers its own socket callback and drives
 * the async connect/send sequence while you block inside the API call.
 *
 * It uses only the headers that already exist in your tree:
 *   - ff.h        (FatFS)
 *   - socket.h    (WINC1500 sockets)
 *   - m2m_wifi.h  (for m2m_wifi_handle_events)
 *   - stm32h5xx_hal.h (HAL_GetTick for timeouts)
 */
#include "Uploader.h"
#include "ff.h"
#include "socket.h"
#include "m2m_wifi.h"
#include "stm32h5xx_hal.h"
#include <string.h>
#include <stdio.h>
static volatile uint32_t s_resolved_ip = 0;
static volatile int s_dns_done = 0;

/* ---------- Tunables ---------- */
#ifndef UPLOADER_CHUNK
#define UPLOADER_CHUNK   1024u   /* bytes per send() */
#endif

#ifndef UPLOADER_EVENT_POLL_MS
#define UPLOADER_EVENT_POLL_MS  1u
#endif

/* ---------- Internal state (single-use, not re-entrant) ---------- */
typedef enum {
    ST_IDLE = 0,
    ST_CONNECTING,
    ST_SENDING_HDR,
    ST_SENDING_FILE,
    ST_DONE,
    ST_ERROR
} uploader_state_t;

static volatile uploader_state_t s_state = ST_IDLE;
static volatile int s_sock = -1;
static FIL s_fil;                 /* open file */
static UINT s_br = 0;             /* bytes read from file */
static uint8_t s_buf[UPLOADER_CHUNK]; /* TX buffer (must persist across callbacks) */
static uint32_t s_total_sent = 0;
static int s_errno = 0;           /* negative error code to return */
static uint32_t s_deadline = 0;   /* tick when we time out (0 = no timeout) */

/* HTTP header buffer */
static char s_hdr[192];
static uint16_t s_hdr_len = 0;
static uint16_t s_hdr_off = 0;

/* ---------- Forward decl ---------- */
static void uploader_socket_cb(SOCKET sock, uint8 u8Msg, void *pvMsg);
static int uploader_connect_and_stream(const char *pc_ip, uint16_t port, uint32_t timeout_ms, int use_http);

/* ---------- Helpers ---------- */
static int32_t _millis(void) { return (int32_t)HAL_GetTick(); }

static int _expired(uint32_t dl) {
    return (dl != 0u) && ((uint32_t)_millis() >= dl);
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
    /* Decide whether we're sending header or file */
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
            return; /* wait for SOCKET_MSG_SEND */
        } else {
            s_state = ST_SENDING_FILE;
        }
    }

    if (s_state == ST_SENDING_FILE) {
        if (s_br == 0) {
            int have = _read_next_chunk();
            if (have < 0) return;      /* error already handled */
            if (have == 0) {
                /* EOF: done */
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
        return; /* wait for SOCKET_MSG_SEND to account progress */
    }
}

/* Socket callback advances the state machine */
static void uploader_socket_cb(SOCKET sock, uint8 u8Msg, void *pvMsg) {
    switch (u8Msg) {
    case SOCKET_MSG_CONNECT: {
        tstrSocketConnectMsg *p = (tstrSocketConnectMsg *)pvMsg;
        if ((p) && (p->s8Error == 0)) {
            if (s_state == ST_CONNECTING) {
                if (s_hdr_len > 0) s_state = ST_SENDING_HDR;
                else               s_state = ST_SENDING_FILE;
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
            printf("Uploader: send callback err=%d\r\n", psent ? *psent : -1);
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
                s_br = 0; /* chunk consumed */
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

/* Common connect/stream driver for both raw and HTTP modes */
static int uploader_connect_and_stream(const char *pc_ip, uint16_t port, uint32_t timeout_ms, int use_http) {
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_port        = _htons(port);
    addr.sin_addr.s_addr = nmi_inet_addr((char *)pc_ip);

    /* Init socket layer (idempotent) and register our callback */
    socketInit();
    registerSocketCallback(uploader_socket_cb, 0);

    s_state = ST_CONNECTING;
    s_errno = 0;
    s_total_sent = 0;
    s_hdr_off = 0;

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

    s_deadline = (timeout_ms ? (uint32_t)_millis() + timeout_ms : 0u);

    while (1) {
        m2m_wifi_handle_events(NULL);

        if (s_state == ST_DONE) {
            return 0;
        }
        if (s_state == ST_ERROR) {
            return s_errno ? s_errno : -99;
        }
        if (_expired(s_deadline)) {
            printf("Uploader: timeout\r\n");
            _set_error(-12);
            return s_errno;
        }
        HAL_Delay(UPLOADER_EVENT_POLL_MS);
    }
}

/* ---------------- Public API ---------------- */

int Uploader_SendFile(const char *fatfs_path, const char *pc_ip, uint16_t port, uint32_t timeout_ms) {
    if (!fatfs_path || !pc_ip) return -1;

    FRESULT fr = f_open(&s_fil, fatfs_path, FA_READ);
    if (fr != FR_OK) {
        printf("Uploader: f_open('%s') error %u\r\n", fatfs_path, (unsigned)fr);
        return -2;
    }

    /* Raw mode: no header */
    s_hdr_len = 0;
    s_hdr_off = 0;

    int rc = uploader_connect_and_stream(pc_ip, port, timeout_ms, 0);

    f_close(&s_fil);
    return rc;
}

int Uploader_SendFileHTTP(const char *fatfs_path, const char *pc_ip, uint16_t port,
                          const char *uri_path, const char *content_type, uint32_t timeout_ms) {
    if (!fatfs_path || !pc_ip) return -1;

    FILINFO finfo;
    memset(&finfo, 0, sizeof(finfo));
    FRESULT fr = f_stat(fatfs_path, &finfo);
    if (fr != FR_OK) {
        printf("Uploader: f_stat('%s') error %u\r\n", fatfs_path, (unsigned)fr);
        return -2;
    }
    unsigned long fsize = (unsigned long)finfo.fsize;

    fr = f_open(&s_fil, fatfs_path, FA_READ);
    if (fr != FR_OK) {
        printf("Uploader: f_open('%s') error %u\r\n", fatfs_path, (unsigned)fr);
        return -3;
    }

    const char *ct = content_type ? content_type : "text/plain";
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

    int rc = uploader_connect_and_stream(pc_ip, port, timeout_ms, 1);

    f_close(&s_fil);
    return rc;
}


static void uploader_dns_cb(SOCKET sock, uint8 u8Msg, void *pvMsg) {
    if (u8Msg == SOCKET_MSG_DNS_RESOLVE) {
        tstrDnsReply *r = (tstrDnsReply*)pvMsg;
        s_resolved_ip = (r && r->u32HostIP) ? r->u32HostIP : 0;
        s_dns_done = 1;
    }
    /* chain into your existing uploader_socket_cb for other events */
    uploader_socket_cb(sock, u8Msg, pvMsg);
}

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
    registerSocketCallback(uploader_dns_cb, 0);
    s_dns_done = 0; s_resolved_ip = 0;

    if (gethostbyname((uint8*)host) != SOCK_ERR_NO_ERROR) {
        f_close(&s_fil);
        return -3;
    }

    uint32_t deadline = timeout_ms ? HAL_GetTick() + timeout_ms : 0;
    while (!s_dns_done) {
        m2m_wifi_handle_events(NULL);
        if (deadline && HAL_GetTick() >= deadline) {
            printf("Uploader: DNS timeout\r\n");
            f_close(&s_fil);
            return -12;
        }
        HAL_Delay(1);
    }
    if (s_resolved_ip == 0) {
        f_close(&s_fil);
        return -13; // DNS failed
    }

    /* No headers (raw TCP) */
    s_hdr_len = 0; s_hdr_off = 0;

    /* Reuse existing connector with resolved IP */
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_port        = _htons(port);
    addr.sin_addr.s_addr = s_resolved_ip;

    /* from here identical to uploader_connect_and_stream(...), inlined for clarity */
    s_state = ST_CONNECTING; s_errno = 0; s_total_sent = 0; s_br = 0;
    s_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (s_sock < 0) { _set_error(-10); f_close(&s_fil); return s_errno; }
    if (connect(s_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        _set_error(-11); f_close(&s_fil); return s_errno;
    }

    s_deadline = deadline;
    while (1) {
        m2m_wifi_handle_events(NULL);
        if (s_state == ST_DONE) { f_close(&s_fil); return 0; }
        if (s_state == ST_ERROR){ f_close(&s_fil); return s_errno ? s_errno : -99; }
        if (s_deadline && HAL_GetTick() >= s_deadline) {
            printf("Uploader: timeout\r\n"); _set_error(-12); f_close(&s_fil); return s_errno;
        }
        HAL_Delay(1);
    }
}
