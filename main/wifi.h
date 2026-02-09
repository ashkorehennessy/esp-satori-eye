#ifndef ESP_SATORI_EYE_WIFI_H
#define ESP_SATORI_EYE_WIFI_H
#include "esp_err.h"

// 定义 AP 的配置信息
#define ESP_WIFI_SSID      "Satori-Eye"
#define ESP_WIFI_PASS      "ystkxhdaze"          // 密码
#define ESP_WIFI_CHANNEL   1
#define MAX_STA_CONN       4                     // 最多同时连接数

void wifi_init_softap(void);
#endif //ESP_SATORI_EYE_WIFI_H