#include "codec_db.h"
#include <string.h>
static size_t esc_decode(const uint8_t *in, size_t n, uint8_t *out, size_t max){
    size_t o=0;
    for (size_t i=0;i<n && o<max;i++){
        uint8_t b=in[i];
        if (b==0xDB && i+1<n){
            uint8_t e=in[++i];
            if (e==0xDC) out[o++]=0xDB;
            else if (e==0xDD) out[o++]=0x00;
            else out[o++]=e;
        } else out[o++]=b;
    }
    return o;
}
static size_t esc_encode(const uint8_t *in, size_t n, uint8_t *out, size_t max){
    size_t o=0;
    for (size_t i=0;i<n && o<max;i++){
        uint8_t b=in[i];
        if (b==0xDB){ if (o+2>max) break; out[o++]=0xDB; out[o++]=0xDC; }
        else if (b==0x00){ if (o+2>max) break; out[o++]=0xDB; out[o++]=0xDD; }
        else out[o++]=b;
    }
    return o;
}
static size_t pack4_decode(const uint8_t *in, size_t n, uint8_t *out, size_t max){
    if (n<2 || in[0]!=0xDB) return 0;
    size_t o=0;
    for (size_t i=1; i+3<n && o<max; i+=4){
        uint8_t a=in[i], b=in[i+1], c=in[i+2], d=in[i+3];
        uint8_t v = ((a&0x03)<<6)|((b&0x03)<<4)|((c&0x03)<<2)|(d&0x03);
        out[o++]=v;
    }
    return o;
}
static size_t pack4_encode(const uint8_t *in, size_t n, uint8_t *out, size_t max){
    if (max<1) return 0; size_t o=0; out[o++]=0xDB;
    for (size_t i=0;i<n && o+4<=max;i++){
        uint8_t v=in[i];
        out[o++]=(v>>6)&0x03;
        out[o++]=(v>>4)&0x03;
        out[o++]=(v>>2)&0x03;
        out[o++]= v    &0x03;
    }
    return o;
}
static db_mode_t guess(const uint8_t *in, size_t n){
    if (n>=2 && in[0]==0xDB){
        size_t small=0; for(size_t i=1;i<n;i++) if ((in[i]&0xFC)==0) small++;
        if (small > n/2) return DB_4TO1;
        return DB_ESC;
    }
    return DB_NONE;
}
size_t jdb_decode(db_mode_t *mode, const uint8_t *in, size_t n, uint8_t *out, size_t max){
    db_mode_t m = (*mode==DB_AUTO)? guess(in,n): *mode;
    size_t k=0;
    switch(m){
        case DB_ESC:  k=esc_decode(in,n,out,max); break;
        case DB_4TO1: k=pack4_decode(in,n,out,max); break;
        case DB_NONE: default: k = (n>max)?max:n; memcpy(out,in,k); break;
    }
    *mode = m;
    return k;
}
size_t jdb_encode(db_mode_t mode, const uint8_t *in, size_t n, uint8_t *out, size_t max){
    switch(mode){
        case DB_ESC:  return esc_encode(in,n,out,max);
        case DB_4TO1: return pack4_encode(in,n,out,max);
        case DB_NONE: default: { size_t k=(n>max)?max:n; memcpy(out,in,k); return k; }
    }
}
