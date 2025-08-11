/* Core/Inc/Uploader.h
 *
 * Minimal file uploader for WINC1500 sockets + FatFS.
 * Adds two blocking convenience functions you can call once Wi-Fi is up:
 *   - Uploader_SendFile(): raw TCP stream (Plan A)
 *   - Uploader_SendFileHTTP(): HTTP POST (Plan B)
 *
 * This file does not modify any existing code; just include & call.
 */
#pragma once

#include <stdint.h>

/**
 * @brief Send a file as a raw TCP byte stream to a host:port.
 *
 * @param fatfs_path  Path in FatFS namespace, e.g. "0:/test.txt"
 * @param pc_ip       Dotted IPv4 string of the receiver (e.g. "192.168.1.23")
 * @param port        TCP port on the receiver (e.g. 9000)
 * @param timeout_ms  Overall timeout for connect + transfer (ms). 0 = no timeout.
 * @return 0 on success; negative error code on failure.
 */
int Uploader_SendFile(const char *fatfs_path, const char *pc_ip, uint16_t port, uint32_t timeout_ms);

/**
 * @brief Send a file via HTTP POST to http://pc_ip:port/<uri_path>.
 *
 * @param fatfs_path   Path in FatFS namespace, e.g. "0:/test.txt"
 * @param pc_ip        Dotted IPv4 string of the receiver
 * @param port         TCP port (e.g. 5000)
 * @param uri_path     URI path without leading slash (e.g. "upload")
 * @param content_type MIME type string (e.g. "text/plain"); pass NULL for default
 * @param timeout_ms   Overall timeout for connect + transfer (ms). 0 = no timeout.
 * @return 0 on success; negative error code on failure.
 */
int Uploader_SendFileHTTP(const char *fatfs_path, const char *pc_ip, uint16_t port,
                          const char *uri_path, const char *content_type, uint32_t timeout_ms);


int Uploader_SendFileHost(const char *fatfs_path,
                          const char *host, uint16_t port, uint32_t timeout_ms);
