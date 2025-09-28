
#ifndef JURA_PROTO_H
#define JURA_PROTO_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// --- Public API ---

// Sendet einen Klartext-"code" (z. B. "@BREW 03 04 10 01").
// Plain, wenn (flags & 0x0A) == 0x08; sonst DB-kodiert + Terminator.
int jura_send_from_code(const char* ascii_code, uint32_t flags);

// Sendet "code" und liest eine Antwort ein.
// - Plain: wartet auf CRLF "\r\n", gibt Antwort ohne CRLF zurück.
// - DB: wartet auf 8-Byte-Terminator und decodiert automatisch (Klartext zurück).
// Rückgabe: >=0 = Länge der Antwort, 0=Timeout, <0 Fehler.
int jura_query(const char* code, uint32_t flags,
               uint8_t* reply_plain, size_t reply_cap, int timeout_ms);

// --- DB-Codec helpers (öffentlich) ---
size_t jura_encode_db_with_term(const uint8_t* plain, size_t len, uint8_t* out, size_t out_cap);
size_t jura_decode_db(const uint8_t* in, size_t in_len, uint8_t* out, size_t out_cap);

// --- UART/VFS - von dir zu implementieren auf echter Hardware ---

// Write: Rückgabe <0 Fehler, sonst gesendete Bytes
int jura_uart_write_bytes(const uint8_t* data, size_t len);

// Read mit Timeout (ms): Rückgabe <0 Fehler, 0 Timeout, >0 gelesene Bytes
int jura_uart_read_bytes(uint8_t* dst, size_t maxlen, int timeout_ms);

#ifdef __cplusplus
}
#endif

#endif // JURA_PROTO_H
