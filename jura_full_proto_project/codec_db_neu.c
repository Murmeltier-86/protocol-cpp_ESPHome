// codec_db.c  — 2b/4sym Auto-Decoder + Encoder
#include <string.h>
#include <stdint.h>
#include <stddef.h>

typedef struct {
  uint8_t sym[4];   // Reihenfolge der 4 Leitungs-Symbole -> Bits 00,01,10,11
  int     msb_first;// 1: erstes Symbol -> Bits 7..6; 0: erstes -> Bits 1..0
  int     align;    // Startversatz 0..3 vor der 4er-Gruppierung
  int     valid;    // 1 wenn gefunden
} db2b_cfg_t;

static db2b_cfg_t g_cfg = { .sym = {0xFF,0xDF,0xFB,0xDB}, .msb_first = 1, .align = 0, .valid = 0 };

// 24 Permutationen der 4 Symbole:
static const uint8_t PERM[24][4] = {
  {0,1,2,3},{0,1,3,2},{0,2,1,3},{0,2,3,1},{0,3,1,2},{0,3,2,1},
  {1,0,2,3},{1,0,3,2},{1,2,0,3},{1,2,3,0},{1,3,0,2},{1,3,2,0},
  {2,0,1,3},{2,0,3,1},{2,1,0,3},{2,1,3,0},{2,3,0,1},{2,3,1,0},
  {3,0,1,2},{3,0,2,1},{3,1,0,2},{3,1,2,0},{3,2,0,1},{3,2,1,0}
};
static const uint8_t SYMS[4] = {0xFF,0xDF,0xFB,0xDB}; // Leitungssymbole

static inline int sym2bits_with_map(uint8_t b, const uint8_t map[4]){
  for (int i=0;i<4;i++) if (b==map[i]) return i; // 0..3 == 00..11
  return -1;
}

static size_t decode_try(const uint8_t *in, size_t n, uint8_t *out, size_t max,
                         const uint8_t map[4], int msb_first, int align) {
  size_t o=0;
  if (n<= (size_t)align) return 0;
  size_t i = align;
  while (i+3 < n && o < max){
    int q0 = sym2bits_with_map(in[i+0], map);
    int q1 = sym2bits_with_map(in[i+1], map);
    int q2 = sym2bits_with_map(in[i+2], map);
    int q3 = sym2bits_with_map(in[i+3], map);
    if ((q0|q1|q2|q3) < 0) break;
    uint8_t v;
    if (msb_first) v = (q0<<6) | (q1<<4) | (q2<<2) | q3;
    else           v = (q3<<6) | (q2<<4) | (q1<<2) | q0;
    out[o++] = v;
    i += 4;
  }
  return o;
}

// ASCII-Score: druckbar, \r\n, beginnt mit '@' oder '&'
static int score_ascii(const uint8_t *p, size_t n){
  if (!n) return -1;
  int printable=0, crlf=0, atamp=0;
  for (size_t i=0;i<n;i++){
    uint8_t c=p[i];
    if (c==0x0D && i+1<n && p[i+1]==0x0A) crlf+=5;
    if (c>=0x20 && c<=0x7E) printable++;
  }
  if (p[0]=='@' || p[0]=='&') atamp=5;
  return printable + crlf + atamp;
}

static void autodetect(const uint8_t *in, size_t n){
  int bestS = -1; db2b_cfg_t best = {0};
  uint8_t map[4];
  for (int pi=0; pi<24; ++pi){
    for (int k=0;k<4;k++) map[k] = SYMS[ PERM[pi][k] ];
    for (int msb=0; msb<=1; ++msb){
      for (int al=0; al<4; ++al){
        uint8_t tmp[512];
        size_t m = decode_try(in, n, tmp, sizeof tmp, map, msb, al);
        int s = score_ascii(tmp, m);
        if (s > bestS){ bestS=s; best.sym[0]=map[0]; best.sym[1]=map[1]; best.sym[2]=map[2]; best.sym[3]=map[3];
                        best.msb_first=msb; best.align=al; best.valid=1; }
      }
    }
  }
  if (best.valid) g_cfg = best;
}

size_t jdb_decode_auto(const uint8_t *in, size_t n, uint8_t *out, size_t max){
  if (!g_cfg.valid) autodetect(in, n);
  // Sicherheitsnetz: wenn Eingabe nicht wie Leitungscode aussieht, roh durchreichen
  size_t sym_ok=0; for (size_t i=0;i<n;i++){ uint8_t b=in[i];
    if (b==0xFF||b==0xDF||b==0xFB||b==0xDB) sym_ok++; }
  if (sym_ok < n*3/4) { size_t k = (n>max?max:n); memcpy(out,in,k); return k; }
  return decode_try(in, n, out, max, g_cfg.sym, g_cfg.msb_first, g_cfg.align);
}

size_t jdb_encode_auto(const uint8_t *in, size_t n, uint8_t *out, size_t max){
  if (!g_cfg.valid) { // Default-Mapping (häufig)
    g_cfg.sym[0]=0xFF; g_cfg.sym[1]=0xDF; g_cfg.sym[2]=0xFB; g_cfg.sym[3]=0xDB;
    g_cfg.msb_first=1; g_cfg.align=0; g_cfg.valid=1;
  }
  size_t o=0;
  for (size_t i=0;i<n && o+4<=max; ++i){
    uint8_t v = in[i];
    uint8_t q0 = g_cfg.msb_first ? ((v>>6)&3) : ((v>>0)&3);
    uint8_t q1 = g_cfg.msb_first ? ((v>>4)&3) : ((v>>2)&3);
    uint8_t q2 = g_cfg.msb_first ? ((v>>2)&3) : ((v>>4)&3);
    uint8_t q3 = g_cfg.msb_first ? ((v>>0)&3) : ((v>>6)&3);
    out[o++] = g_cfg.sym[q0];
    out[o++] = g_cfg.sym[q1];
    out[o++] = g_cfg.sym[q2];
    out[o++] = g_cfg.sym[q3];
  }
  return o;
}

// öffentliche API wie zuvor
typedef enum { DB_AUTO=0, DB_2B4B=1, DB_NONE=2 } db_mode_t;
size_t jdb_decode(db_mode_t *mode, const uint8_t *in, size_t n, uint8_t *out, size_t max){
  (void)mode; return jdb_decode_auto(in,n,out,max);
}
size_t jdb_encode(db_mode_t mode, const uint8_t *in, size_t n, uint8_t *out, size_t max){
  (void)mode; return jdb_encode_auto(in,n,out,max);
}
