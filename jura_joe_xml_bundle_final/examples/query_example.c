
#include "C/jura_proto.h"
#include <stdio.h>
#include <string.h>

int main() {
    uint8_t buf[1024];

    // Beispiel: Plain-Status (endet mit CRLF); Plain wenn (flags & 0x0A) == 0x08
    int n = jura_query("&STAT?", 0x08, buf, sizeof(buf), 2000);
    if (n > 0) printf("Antwort (plain): %.*s\n", n, buf);
    else printf("Keine Antwort / Timeout (plain), rc=%d\n", n);

    // Beispiel: DB-Command
    n = jura_query("@BREW 03 04 10 01", 0x00, buf, sizeof(buf), 3000);
    if (n > 0) printf("Antwort (db->decoded): %.*s\n", n, buf);
    else printf("Keine Antwort / Timeout (db), rc=%d\n", n);

    return 0;
}
