#include "ai.h"
#include "cat_detect.hpp"
#include "dl_image_jpeg.hpp"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include <cstring>

static const char *TAG = "AI";

uint8_t *g_ai_buf_a = nullptr;
uint8_t *g_ai_buf_b = nullptr;
QueueHandle_t xAIQueue = nullptr;

extern "C" void ai_init_service(void) {
    // 1. 分配两个巨大的 RGB 缓冲区
    g_ai_buf_a = static_cast<uint8_t*>(heap_caps_malloc(AI_RGB_SIZE, MALLOC_CAP_SPIRAM));
    g_ai_buf_b = static_cast<uint8_t*>(heap_caps_malloc(AI_RGB_SIZE, MALLOC_CAP_SPIRAM));

    // 创建队列 (只存指针，深度为 2)
    xAIQueue = xQueueCreate(2, sizeof(uint8_t *));

    if (!g_ai_buf_a || !g_ai_buf_b) {
        ESP_LOGE(TAG, "Failed to allocate AI buffers!");
    }
}


extern "C" void ai_decode_jpeg_wrapper(uint8_t *jpg_data, size_t jpg_len, uint8_t *target_rgb_buf) {
    // 构造 ESP-DL 需要的结构体
    dl::image::jpeg_img_t jpeg_img = {
        .data = jpg_data,
        .data_len = jpg_len
    };

    // 执行软解码
    auto img_decoded = dl::image::sw_decode_jpeg(jpeg_img, dl::image::DL_IMAGE_PIX_TYPE_RGB888);

    // 将解码后的数据搬运到目标全局 Buffer
    if (img_decoded.data) {
        // 如果尺寸匹配，直接拷贝
        size_t copy_len = img_decoded.width * img_decoded.height * 3;
        if (copy_len > AI_RGB_SIZE) copy_len = AI_RGB_SIZE; // 防止溢出

        memcpy(target_rgb_buf, img_decoded.data, copy_len);

        // 4. 重要：释放 sw_decode_jpeg 内部 malloc 的内存
        heap_caps_free(img_decoded.data);
    } else {
        ESP_LOGE(TAG, "JPEG Decode Failed");
    }
}

extern "C" void ai_inference_task(void *arg) {
    // 初始化模型
    auto *detect = new CatDetect();
    uint8_t *current_rgb_buf = nullptr;

    ESP_LOGI(TAG, "Core 1 AI Task Started");

    while (true) {
        // 阻塞等待 Core 0 传来的指针
        if (xQueueReceive(xAIQueue, &current_rgb_buf, portMAX_DELAY) == pdTRUE) {

            // 构造 img_t 给模型
            dl::image::img_t img;
            img.data = current_rgb_buf; // 直接用全局变量的地址
            img.width = AI_W;
            img.height = AI_H;
            img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB888;

            // 推理
            int64_t start_time = esp_timer_get_time();
            auto &detect_results = detect->run(img);
            int64_t end_time = esp_timer_get_time();

            // 打印结果
            for (const auto &res : detect_results) {
                ESP_LOGI(TAG, "Det: Cat:%d Score:%.2f, time:%dms", res.category, res.score, end_time-start_time);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}