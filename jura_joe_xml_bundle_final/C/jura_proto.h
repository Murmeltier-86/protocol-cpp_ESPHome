#ifndef JURA_PROTO_H
#define JURA_PROTO_H
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
int jura_send_from_code(const char* ascii_code, uint32_t flags);
int jura_uart_write_bytes(const uint8_t* data, size_t len);
size_t jura_encode_db_with_term(const uint8_t* plain, size_t len, uint8_t* out, size_t out_cap);
size_t jura_decode_db(const uint8_t* in, size_t in_len, uint8_t* out, size_t out_cap);
#ifdef __cplusplus
}
#endif
#endif
