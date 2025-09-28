#include "jura_proto.h"
#include <string.h>
static size_t jura_encode_db_core(const uint8_t *plain, size_t len, uint8_t *out, size_t out_cap) {
    size_t o = 0;
    for (size_t i = 0; i < len; i++) {
        uint8_t v = plain[i];
        for (int pair = 0; pair < 4; pair++) {
            if (o >= out_cap) return o;
            uint8_t sym = 0xDB;
            int shift = 6 - 2*pair;
            uint8_t two = (uint8_t)((v >> shift) & 0x03);
            if (two & 0x01) sym |= 0x04;
            if (two & 0x02) sym |= 0x20;
            out[o++] = sym;
        }
    }
    return o;
}
size_t jura_encode_db_with_term(const uint8_t* plain, size_t len, uint8_t* out, size_t out_cap) {
    static const uint8_t TERM[8] = {0xDF,0xFF,0xDB,0xDB,0xFB,0xFB,0xDB,0xDB};
    size_t o = jura_encode_db_core(plain, len, out, out_cap);
    for (int i=0;i<8 && o<out_cap;i++) out[o++] = TERM[i];
    return o;
}
size_t jura_decode_db(const uint8_t *in, size_t in_len, uint8_t *out, size_t out_cap) {
    uint8_t acc = 0; int pair_cnt = 0; size_t o = 0;
    for (size_t i = 0; i < in_len; i++) {
        uint8_t b = in[i];
        if ( (b & 0xDB) != 0xDB ) continue;
        acc >>= 2;
        if (b & 0x04) acc |= 0x40;
        if (b & 0x20) acc |= 0x80;
        if (++pair_cnt == 4) { if (o < out_cap) out[o] = acc; o++; acc=0; pair_cnt=0; }
    }
    if (o < out_cap) out[o] = 0;
    return o;
}
int jura_send_from_code(const char* ascii_code, uint32_t flags) {
    if (!ascii_code) return -1;
    size_t n = strlen(ascii_code);
    if ( (flags & 0x0A) == 0x08 ) {
        return jura_uart_write_bytes((const uint8_t*)ascii_code, n);
    } else {
        uint8_t buf[512];
        size_t out_len = jura_encode_db_with_term((const uint8_t*)ascii_code, n, buf, sizeof(buf));
        return jura_uart_write_bytes(buf, out_len);
    }
}
