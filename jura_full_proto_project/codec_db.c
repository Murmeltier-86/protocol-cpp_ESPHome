#include <string.h>
#include <stdint.h>
#include <stddef.h>

typedef enum { DB_AUTO=0, DB_2B4B=1, DB_NONE=2 } db_mode_t;

static inline int sym2bits(uint8_t s){
    switch(s){
        case 0xFF: return 0; // 00
        case 0xDF: return 1; // 01
        case 0xFB: return 2; // 10
        case 0xDB: return 3; // 11
        default:   return -1;
    }
}
static inline uint8_t bits2sym(int two_bits){
    static const uint8_t lut[4] = { 0xFF, 0xDF, 0xFB, 0xDB };
    return lut[two_bits & 3];
}

static size_t decode_2b4b(const uint8_t *in, size_t n, uint8_t *out, size_t max){
    size_t o=0;
    for (size_t i=0; i+3<n && o<max; i+=4){
        int b0 = sym2bits(in[i+0]); if (b0<0) break;
        int b1 = sym2bits(in[i+1]); if (b1<0) break;
        int b2 = sym2bits(in[i+2]); if (b2<0) break;
        int b3 = sym2bits(in[i+3]); if (b3<0) break;
        out[o++] = (uint8_t)((b0<<6) | (b1<<4) | (b2<<2) | b3);
    }
    return o;
}
static size_t encode_2b4b(const uint8_t *in, size_t n, uint8_t *out, size_t max){
    size_t o=0;
    for (size_t i=0; i<n && o+4<=max; i++){
        uint8_t v = in[i];
        out[o++] = bits2sym((v>>6)&3);
        out[o++] = bits2sym((v>>4)&3);
        out[o++] = bits2sym((v>>2)&3);
        out[o++] = bits2sym((v>>0)&3);
    }
    return o;
}

// einfache Heuristik: wenn >80% Bytes ∈ {FF,DF,FB,DB} → 2b4b
static int looks_like_2b4b(const uint8_t *in, size_t n){
    if (n<16) return 0;
    size_t ok=0;
    for (size_t i=0;i<n;i++){
        uint8_t b=in[i];
        if (b==0xFF || b==0xDF || b==0xFB || b==0xDB) ok++;
    }
    return ok*5 >= n*4; // ≥80%
}

size_t jdb_decode(db_mode_t *mode, const uint8_t *in, size_t n, uint8_t *out, size_t max){
    db_mode_t m = *mode;
    if (m==DB_AUTO) m = looks_like_2b4b(in,n) ? DB_2B4B : DB_NONE;
    size_t k = (m==DB_2B4B) ? decode_2b4b(in,n,out,max)
                            : (n>max?max:n), memcpy(out,in,(n>max?max:n)), (n>max?max:n);
    *mode = m;
    return k;
}
size_t jdb_encode(db_mode_t mode, const uint8_t *in, size_t n, uint8_t *out, size_t max){
    if (mode==DB_2B4B) return encode_2b4b(in,n,out,max);
    size_t k = (n>max?max:n); memcpy(out,in,k); return k;
}
