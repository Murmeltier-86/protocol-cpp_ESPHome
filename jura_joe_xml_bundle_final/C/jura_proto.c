
#include "jura_proto.h"
#include <string.h>
#include <stdio.h>

// ---- DB-Codec (wie in FW beobachtet) ----

static size_t jura_encode_db_core(const uint8_t *plain, size_t len, uint8_t *out, size_t out_cap) {
    size_t o = 0;
    for (size_t i = 0; i < len; i++) {
        uint8_t v = plain[i];
        for (int pair = 0; pair < 4; pair++) {
            if (o >= out_cap) return o;
            uint8_t sym = 0xDB;
            int shift = 6 - 2*pair;              // 7..6, 5..4, 3..2, 1..0
            uint8_t two = (uint8_t)((v >> shift) & 0x03);
            if (two & 0x01) sym |= 0x04;         // low-Bit -> Bit2
            if (two & 0x02) sym |= 0x20;         // high-Bit -> Bit5
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
    uint8_t acc = 0;
    int pair_cnt = 0;
    size_t o = 0;
    for (size_t i = 0; i < in_len; i++) {
        uint8_t b = in[i];
        if ( (b & 0xDB) != 0xDB ) continue;
        acc >>= 2;
        if (b & 0x04) acc |= 0x40;
        if (b & 0x20) acc |= 0x80;
        if (++pair_cnt == 4) {
            if (o < out_cap) out[o] = acc;
            o++;
            acc = 0;
            pair_cnt = 0;
        }
    }
    if (o < out_cap) out[o] = 0;
    return o;
}

// ---- Senden ----

int jura_send_from_code(const char* ascii_code, uint32_t flags) {
    if (!ascii_code) return -1;
    size_t n = strlen(ascii_code);

    // Plain, wenn (flags & 0x0A) == 0x08
    if ( (flags & 0x0A) == 0x08 ) {
        return jura_uart_write_bytes((const uint8_t*)ascii_code, n);
    } else {
        uint8_t buf[1024];
        size_t out_len = jura_encode_db_with_term((const uint8_t*)ascii_code, n, buf, sizeof(buf));
        return jura_uart_write_bytes(buf, out_len);
    }
}

// ---- Terminator-Erkennung ----
static int is_db_terminator_window(const uint8_t *w) {
    static const uint8_t T[8]={0xDF,0xFF,0xDB,0xDB,0xFB,0xFB,0xDB,0xDB};
    for(int i=0;i<8;i++) if (w[i]!=T[i]) return 0;
    return 1;
}

// ---- Query (Send + Receive + optional Decode) ----
int jura_query(const char* code, uint32_t flags,
               uint8_t* reply_plain, size_t reply_cap, int timeout_ms)
{
    if (!code || !reply_plain || reply_cap==0) return -1;

    // 1) senden
    if (jura_send_from_code(code, flags) < 0) return -2;

    // 2) Antwort lesen:
    uint8_t rx[4096]; size_t rlen=0;
    const int step_ms = 25;
    int waited=0;
    while (waited < timeout_ms && rlen < sizeof(rx)) {
        int n = jura_uart_read_bytes(rx + rlen, sizeof(rx) - rlen, step_ms);
        if (n < 0) return -3;
        if (n > 0) {
            rlen += (size_t)n;

            if ((flags & 0x0A) == 0x08) {
                // PLAIN: auf CRLF prüfen
                if (rlen >= 2 && rx[rlen-2]=='\r' && rx[rlen-1]=='\n') {
                    size_t out = (rlen-2 < reply_cap ? rlen-2 : reply_cap-1);
                    memcpy(reply_plain, rx, out); reply_plain[out]=0;
                    return (int)out;
                }
            } else {
                // DB: auf 8-Byte-Terminator prüfen
                if (rlen >= 8 && is_db_terminator_window(rx + rlen - 8)) {
                    size_t out = jura_decode_db(rx, rlen-8, reply_plain, reply_cap);
                    return (int)out;
                }
            }
        }
        waited += step_ms;
    }
    return 0; // Timeout
}

// ---- UART STUBS (Desktop): bitte auf Embedded anpassen ----
int jura_uart_write_bytes(const uint8_t* data, size_t len) {
    // Desktop-Stub: nur hexdump auf stdout
    fprintf(stdout, "UART TX (%zu bytes):", len);
    for (size_t i=0;i<len;i++) fprintf(stdout, " %02X", data[i]);
    fprintf(stdout, "\n");
    fflush(stdout);
    return (int)len;
}

int jura_uart_read_bytes(uint8_t* dst, size_t maxlen, int timeout_ms) {
    // Desktop-Stub: simuliert Timeout (keine Antwort)
    (void)dst; (void)maxlen; (void)timeout_ms;
    return 0;
}
