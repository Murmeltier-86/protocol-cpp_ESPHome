#pragma once
#include <stddef.h>
#include <stdint.h>
typedef enum { DB_AUTO=0, DB_ESC=1, DB_4TO1=2, DB_NONE=3 } db_mode_t;
size_t jdb_decode(db_mode_t *mode, const uint8_t *in, size_t in_len,
                  uint8_t *out, size_t out_max);
size_t jdb_encode(db_mode_t mode, const uint8_t *in, size_t in_len,
                  uint8_t *out, size_t out_max);