#include "Elemach.h"
#include "main.h"
#include "tim.h"
/*测试调用
//    Motor_SetSpeedFromInput(2500);  // 对应250 RPM  
//		Motor_SetDirection(0);          // 往上转
//		HAL_Delay(50000);
*/

Motor_TypeDef motor = {8, 0, 0};  // 默认8细分，60RPM，正转

// 电机状态变量
volatile uint8_t motor_enabled = 0;
volatile uint16_t motor_current_speed = 0;
volatile uint8_t motor_current_direction = 0;

// 定距离运动变量
volatile uint32_t target_pulse_count = 0;
volatile uint32_t current_pulse_count = 0;
volatile uint8_t motor_distance_mode = 0;

// 输入值范围定义
#define INPUT_MIN 0
#define INPUT_MAX 3000
#define RPM_MIN 0
#define RPM_MAX 300

/**
 * @brief 根据输入值(0-3000)线性映射到RPM(0-300)
 * @param input_value 输入值 (0-3000)
 * @return 对应的RPM值 (0-300)
 */
uint16_t MapInputToRPM(uint16_t input_value)
{
    // 限制输入范围
    //if (input_value < INPUT_MIN) input_value = INPUT_MIN;
    if (input_value > INPUT_MAX) input_value = INPUT_MAX;
    
    // 线性映射: RPM = (input * RPM_MAX) / INPUT_MAX
    uint32_t rpm = ((uint32_t)input_value * RPM_MAX) / INPUT_MAX;
    
    return (uint16_t)rpm;
}

/**
 * @brief 根据输入值设置电机速度
 * @param input_value 输入值 (0-3000)
 */
void Motor_SetSpeedFromInput(uint16_t input_value)
{
    // 映射到RPM
    uint16_t rpm = MapInputToRPM(input_value);
    
    // 设置速度
    Motor_SetSpeed(rpm);
}

/**
 * @brief 设置电机进行定距离运动
 */
void Motor_DistanceMove(uint8_t direction, uint32_t pulse_count, uint16_t speed_value)
{
    target_pulse_count = pulse_count;
    current_pulse_count = 0;
    motor_distance_mode = 1;
    
    Motor_SetDirection(direction);
    Motor_SetSpeedFromInput(speed_value);
    Motor_Enable();
    
    motor_enabled = 1;
    motor_current_speed = speed_value;
    motor_current_direction = direction;
}


/**
 * @brief 电机初始化
 */
void Motor_Init(void)
{
    // 设置初始细分
    SubdivisionSet(motor.current_subdivision);
    
    // 设置初始方向
    Motor_SetDirection(motor.direction);
    
    // 使能电机
    Motor_Enable();
    
    // 明确设置初始速度为0，并停止PWM
    motor.target_rpm = 0;
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);  // 确保PWM停止
}

/**
 * @brief 设置细分模式
 * @param subdivision 细分值 (8, 16, 32, 64)
 */
void SubdivisionSet(uint8_t subdivision)
{
    switch(subdivision)
    {
        case 8:   // 8细分
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
            break;
        case 16:  // 16细分
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
            break;
        case 32:  // 32细分
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
            break;
        case 64:  // 64细分
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
            break;
        default:  // 默认8细分
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
            subdivision = 8;
            break;
    }
    motor.current_subdivision = subdivision;
}

/**
 * @brief 设置电机旋转方向
 * @param dir 1 = 探针往下转, 0 =探针往上转
 */
void Motor_SetDirection(uint8_t dir)
{
    if(dir == 1)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET); 
    }
    else
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);   
    }
    motor.direction = dir;
}

/**
 * @brief 设置电机转速
 * @param rpm 转速值 (RPM)
 */
void Motor_SetSpeed(uint16_t rpm)
{
    motor.target_rpm = rpm;
    
    // 如果速度为0，立即停止PWM
    if (rpm == 0) {
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
    } else {
        // 只有速度不为0时才更新频率
        Motor_UpdateFrequency(motor.current_subdivision);
    }
}

/**
 * @brief 使能电机
 */
void Motor_Enable(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);  // EN低电平使能
}

/**
 * @brief 禁用电机
 */
void Motor_Disable(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);    // EN高电平禁用
}

/**
 * @brief 更新PWM频率以保持设定转速
 * @param subdivision 细分值
 */
void Motor_UpdateFrequency(uint8_t subdivision)
{
    uint32_t steps_per_rev;
    uint32_t required_freq;
    uint32_t arr_value;
    
    // 根据细分计算每转所需脉冲数
    switch(subdivision)
    {
        case 8:
            steps_per_rev = 1600;  // 200 * 8
            break;
        case 16:
            steps_per_rev = 3200;  // 200 * 16
            break;
        case 32:
            steps_per_rev = 6400;  // 200 * 32
            break;
        case 64:
            steps_per_rev = 12800; // 200 * 64
            break;
        default:
            steps_per_rev = 1600;  // 默认8细分
            break;
    }
    
    // 计算所需频率：频率 = (RPM × 每转脉冲数) ÷ 60
    required_freq = (motor.target_rpm * steps_per_rev) / 60;
    
    // 如果速度为0，停止PWM输出
    if (motor.target_rpm == 0) {
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
        return;
    }
    
    // 计算ARR值：ARR = (定时器时钟 ÷ 频率) - 1
    // 定时器时钟 = 72MHz / (PSC + 1) = 72MHz / 72 = 1MHz
    arr_value = (1000000 / required_freq) - 1;
    
    // 限制ARR值范围
    if(arr_value < 10) arr_value = 10;
    if(arr_value > 0xFFFF) arr_value = 0xFFFF;
    
    // 停止PWM输出
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
    
    // 更新ARR值
    __HAL_TIM_SET_AUTORELOAD(&htim1, arr_value);
    
    // 重新启动PWM输出
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

/**
 * @brief 切换细分并保持转速
 * @param new_subdivision 新的细分值
 */
void Motor_ChangeSubdivision(uint8_t new_subdivision)
{
    if(new_subdivision == motor.current_subdivision) {
        return;
    }
    
    // 停止PWM输出
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
    
    // 设置新的细分
    SubdivisionSet(new_subdivision);
    
    // 更新频率以保持转速
    Motor_UpdateFrequency(new_subdivision);
}
