#pragma once

#define ESP_LOGW(tag, fmt, ...) (void) sizeof(tag), (void) sizeof(fmt)
#define ESP_LOGD(tag, fmt, ...) (void) sizeof(tag), (void) sizeof(fmt)
#define ESP_LOGI(tag, fmt, ...) (void) sizeof(tag), (void) sizeof(fmt)
#define ESP_LOGCONFIG(tag, fmt, ...) (void) sizeof(tag), (void) sizeof(fmt)

