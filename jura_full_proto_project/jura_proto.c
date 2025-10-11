#include "jura_proto.h"
#include "uart_machine.h"
#include "codec_db.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG="JURA";
static db_mode_t s_mode = DB_AUTO;
static uint8_t   s_rxbuf[2048];
static size_t    s_rxlen;

static void on_rx(const uint8_t *d, size_t n){
    if (s_rxlen + n > sizeof s_rxbuf) s_rxlen = 0;
    memcpy(s_rxbuf + s_rxlen, d, n); s_rxlen += n;
}

int jura_proto_init(void){
    uartm_set_rx_callback(on_rx);
    s_mode = DB_AUTO;
    s_rxlen = 0;
    return 0;
}

static int send_ascii(const char *s){ return uartm_write((const uint8_t*)s, strlen(s)); }
static int await_any(char *out, int max, int ms){
    int left = ms/10;
    while (left--){
        if (s_rxlen){
            vTaskDelay(pdMS_TO_TICKS(20));
            size_t n=s_rxlen; s_rxlen=0;
            uint8_t dec[2048]; size_t m=jdb_decode(&s_mode, s_rxbuf, n, dec, sizeof dec);
            int k = (m>max-1)?(max-1):m; memcpy(out, dec, k); out[k]=0; return k;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return 0;
}

void jura_proto_handshake(void){
    char resp[1024];
    send_ascii("&WHO\r\n");
    if (await_any(resp, sizeof resp, 500)) { s_mode=DB_NONE; return; }
    const char *probes[] = { "@TR:37", "@TR:32", "@t2:8188", "@TS:00" };
    for (size_t i=0;i<sizeof(probes)/sizeof(probes[0]);i++){
        send_ascii(probes[i]); send_ascii("\r\n");
        if (await_any(resp, sizeof resp, 800)) { return; }
    }
    s_mode = DB_ESC;
}

int jura_proto_req_machine_xml(char *xml_out, int xml_max){
    send_ascii("@hr:00\r\n");
    int n = await_any(xml_out, xml_max, 1500);
    if (n<32) { send_ascii("@hr:05\r\n"); n = await_any(xml_out, xml_max, 1500); }
    return n;
}

int jura_proto_write_machine_xml(const char *xml, int xml_len){
    send_ascii("@ha:00\r\n");
    send_ascii("@HD:000000000040\r\n");
    uartm_write((const uint8_t*)xml, xml_len);
    send_ascii("@hu:ok\r\n");
    return 0;
}