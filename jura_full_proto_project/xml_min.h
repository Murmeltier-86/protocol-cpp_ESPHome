#pragma once
#include <stdbool.h>
bool xml_get_value(const char *xml, const char *path, char *out, int out_max);
bool xml_set_value(char *xml, const char *path, const char *val);