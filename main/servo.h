#ifndef ESP_SATORI_EYE_SERVO_H
#define ESP_SATORI_EYE_SERVO_H
#include "esp_err.h"
#include <stdint.h>

// MG90S 舵机参数
#define SERVO_MIN_ANGLE     0
#define SERVO_MAX_ANGLE     180
#define SERVO_MIN_PULSE_US  500    // 0° 对应脉宽
#define SERVO_MAX_PULSE_US  2400   // 180° 对应脉宽
#define SERVO_PERIOD_MS     20     // PWM 周期 (50Hz)

// 初始化舵机（预留 LEDC PWM）
void servo_init(void);

// 设置三轴舵机角度，更新 CTX 并输出 log
void servo_set(int16_t x, int16_t y, int16_t eyelid);

#endif //ESP_SATORI_EYE_SERVO_H
