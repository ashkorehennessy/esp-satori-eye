#include <stdio.h>
#include "esp_chip_info.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_system.h"

void app_main(void)
{
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    size_t free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    printf("Total Free PSRAM: %d Bytes (%.2f MB)\n", free_psram, free_psram / 1024.0 / 1024.0);
}
