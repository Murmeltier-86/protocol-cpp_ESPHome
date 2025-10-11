#include "jura_proto.h"
#include "uart_machine.h"
#include "esp_log.h"

static const char *TAG="APP";

void app_main(void) {
    uartm_init(/*tx=*/17, /*rx=*/16, /*baud=*/9600);
    jura_proto_init();
    jura_proto_handshake();
    ESP_LOGI(TAG, "Handshake fertig. Ab jetzt: Abfragen starten.");
    char xml[2048];
    int xml_len = jura_proto_req_machine_xml(xml, sizeof xml);
    if (xml_len > 0) {
        char val[64];
        if (xml_get_value(xml, "Machine/Status/WaterLevel", val, sizeof val))
            ESP_LOGI(TAG, "WaterLevel=%s", val);
        xml_set_value(xml, "Machine/Settings/Temp", "93");
        jura_proto_write_machine_xml(xml, xml_len);
    }
}