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

// 舵机引脚定义（物理通道）
#define SERVO_PIN_1         1      // 通道1: X 轴 (水平)
#define SERVO_PIN_2         2      // 通道2: Y 轴 (垂直) + 下眼皮联动
#define SERVO_PIN_3         4      // 通道3: 上眼皮

// Y-眼皮联动参数（线性拟合: servo3 = RATIO * servo2 + OFFSET - SCALE * eyelid）
#define EYELID_Y_RATIO    (-1.4f)   // servo3 对 servo2 的跟随比例
#define EYELID_Y_OFFSET   312.0f    // 截距
#define EYELID_SCALE      1.79f     // eyelid(0~100) 到 servo3 的缩放系数

// 初始化舵机 LEDC PWM
void servo_init(void);

// 高级接口：逻辑控制（自动处理 Y-眼皮联动）
// x: 水平角度 0~180
// y: 垂直角度 0~180（同时驱动 servo2 + 联动计算 servo3）
// eyelid: 睁眼程度 0(闭合) ~ 100(全开)
void servo_set(int16_t x, int16_t y, int16_t eyelid);

// 底层接口：直接控制三路舵机角度（无联动，调试用）
void servo_set_raw(int16_t s1, int16_t s2, int16_t s3);

#endif //ESP_SATORI_EYE_SERVO_H
