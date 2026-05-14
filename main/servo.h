#ifndef ESP_SATORI_EYE_SERVO_H
#define ESP_SATORI_EYE_SERVO_H
#include "esp_err.h"
#include <stdint.h>

// MG90S 舵机参数
#define SERVO_MIN_ANGLE     0
#define SERVO_MAX_ANGLE     180
#define SERVO_MIN_PULSE_US  500    // 0° 对应脉宽
#define SERVO_MAX_PULSE_US  2400   // 180° 对应脉宽
#define SERVO_FREQ_HZ       50     // PWM 频率 (50Hz = 20ms 周期)

// 舵机引脚定义
#define SERVO_PIN_X         1      // X 轴 (水平)
#define SERVO_PIN_Y         2      // Y 轴 (垂直)
#define SERVO_PIN_EYELID    4      // 眼皮

// 初始化舵机 LEDC PWM
void servo_init(void);

// 设置三轴舵机角度 (0~180°)
void servo_set(int16_t x, int16_t y, int16_t eyelid);

#endif //ESP_SATORI_EYE_SERVO_H
