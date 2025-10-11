#pragma once
#include <stddef.h>
int  jura_proto_init(void);
void jura_proto_handshake(void);
int  jura_proto_req_machine_xml(char *xml_out, int xml_max);
int  jura_proto_write_machine_xml(const char *xml, int xml_len);