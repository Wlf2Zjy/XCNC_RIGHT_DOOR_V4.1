#ifndef ELEMACH_H
#define ELEMACH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "tim.h"

// 电机参数结构体
typedef struct {
    uint8_t current_subdivision;  // 当前细分
    uint16_t target_rpm;          // 目标转速
    uint8_t direction;            // 方向
} Motor_TypeDef;

// 函数声明
void Motor_Init(void);
void SubdivisionSet(uint8_t subdivision);
void Motor_SetDirection(uint8_t dir);
void Motor_SetSpeed(uint16_t rpm);
void Motor_SetSpeedFromInput(uint16_t input_value);
void Motor_Enable(void);
void Motor_Disable(void);
void Motor_UpdateFrequency(uint8_t subdivision);
void Motor_ChangeSubdivision(uint8_t new_subdivision);
void Motor_DistanceMove(uint8_t direction, uint32_t pulse_count, uint16_t speed_value);

// 全局变量声明（外部引用）
extern Motor_TypeDef motor;
extern volatile uint8_t motor_enabled;
extern volatile uint8_t motor_distance_mode;
extern volatile uint32_t target_pulse_count;
extern volatile uint32_t current_pulse_count;
extern volatile uint16_t motor_current_speed;
extern volatile uint8_t motor_current_direction;

#ifdef __cplusplus
}
#endif

#endif /* ELEMACH_H */
