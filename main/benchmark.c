#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

// 全局开关
static volatile bool g_bench_running = true;

// 通用的测试任务函数
void IRAM_ATTR benchmark_task(void *arg) {
    // 获取传入的核心 ID (0 或 1)
    int core_id = (int)arg;

    // 创建不同的 TAG，方便在日志里一眼区分
    char TASK_TAG[16];
    snprintf(TASK_TAG, sizeof(TASK_TAG), "BENCH_C%d", core_id);

    uint64_t counter = 0;
    int64_t start_time = esp_timer_get_time();

    ESP_LOGW(TASK_TAG, "Started on Core %d", xPortGetCoreID());

    while (g_bench_running) {
        // === 模拟负载 (和之前完全一样的算法) ===
        volatile uint32_t dummy = 0;
        for(int i=0; i<1000; i++) {
            dummy++;
        }
        counter++;

        // === 每秒统计 ===
        int64_t now = esp_timer_get_time();
        if ((now - start_time) >= 1000000) {
            // 打印得分
            ESP_LOGW(TASK_TAG, "Idle Score: %llu ops/sec", counter);

            counter = 0;
            start_time = now;

            // 喂狗，防 Core Dump
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    vTaskDelete(NULL);
}

void start_benchmark(void) {
    // 启动 Core 0 的测试任务
    // 堆栈给 4096 防止溢出，优先级 1 (最低)，参数传入 (void*)0
    xTaskCreatePinnedToCore(benchmark_task, "bench_c0", 4096, (void*)0, 1, NULL, 0);

    // 启动 Core 1 的测试任务
    // 参数传入 (void*)1，绑定到核心 1
    xTaskCreatePinnedToCore(benchmark_task, "bench_c1", 4096, (void*)1, 1, NULL, 1);
}