/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Elemach.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "stm32f1xx_hal.h"
#include "Brush_Led.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FRAME_HEADER_1 0xFE     // 帧头第一个字节
#define FRAME_HEADER_2 0xFE     // 帧头第二个字节
#define FRAME_END 0xFA          // 帧结束字节

// 接收状态枚举
typedef enum {
    RX_STATE_WAIT_HEADER1,      // 等待帧头第一个字节
    RX_STATE_WAIT_HEADER2,      // 等待帧头第二个字节
    RX_STATE_WAIT_LENGTH,       // 等待长度字节
	  RX_STATE_WAIT_ID,           // 等待ID字节
    RX_STATE_WAIT_CMD,          // 等待指令字节
    RX_STATE_WAIT_CONTENT,      // 等待内容字节
    RX_STATE_WAIT_END           // 等待结束字节
} RxState;


volatile RxState rxState = RX_STATE_WAIT_HEADER1;  // 接收状态
uint8_t uart_rxByte;  // 串口接收的单个字节
// 协议帧变量
uint8_t rx_id;
uint8_t rx_cmd;                 // 接收到的指令
uint8_t rx_len;                 // 接收到的长度
uint8_t rx_content[64];         // 接收到的内容
uint8_t rx_content_index;       // 内容索引
volatile uint8_t frameReceived = 0; // 帧接收完成标志
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern UART_HandleTypeDef huart2;  //串口2的485通信

// 发送数据函数（无需控制方向引脚）
void RS485_SendData(uint8_t *pData, uint16_t Size)
{
    HAL_UART_Transmit(&huart2, pData, Size, 1000);
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// 限位中断标志
volatile uint8_t upper_limit_triggered = 0;
volatile uint8_t lower_limit_triggered = 0;

// 限位消抖变量 
#define DEBOUNCE_COUNT 2  // 消抖计数阈值 (2 × 10ms = 20ms)
volatile uint8_t upper_limit_raw = 0;      // 上限位原始状态
volatile uint8_t lower_limit_raw = 0;      // 下限位原始状态
volatile uint8_t upper_limit_debounce_counter = 0;  // 上限位消抖计数器
volatile uint8_t lower_limit_debounce_counter = 0;  // 下限位消抖计数器
volatile uint8_t upper_limit_stable = 0;   // 上限位稳定状态
volatile uint8_t lower_limit_stable = 0;   // 下限位稳定状态

// 发送响应帧
void send_response_frame(uint8_t cmd, uint8_t return_len, uint8_t *return_content) {
    uint8_t frame[64];
    uint8_t index = 0;
    
    // 帧头
    frame[index++] = FRAME_HEADER_1;
    frame[index++] = FRAME_HEADER_2;
    
    // 返回长度 (ID1字节 + 指令1字节 + 内容n字节 + 结束符1字节)
    frame[index++] = return_len + 3;
    
    // 舵机ID
    frame[index++] = rx_id;
    
    // 返回指令
    frame[index++] = cmd;
    
    // 返回内容
    if (return_content != NULL && return_len > 0) {
        memcpy(&frame[index], return_content, return_len);
        index += return_len;
    }
    
    // 结束位
    frame[index++] = FRAME_END;
    
    // 发送响应
    RS485_SendData(frame, index);
}

// 处理接收到的协议帧 
void process_protocol_frame(void) {
    uint8_t response_content[1] = {0};
    uint8_t response_len = 1;
		
// RFID控制指令 (0x01)
if (rx_cmd == 0x01 && rx_content_index >= 3) {
    // 解析指令内容: [方向(1字节)] [速度高字节] [速度低字节]
    uint8_t direction = rx_content[0];
    uint16_t speed_value = (rx_content[1] << 8) | rx_content[2];
    
    // 边界检查
    if (speed_value > 3000) {
        speed_value = 3000;
    }
    
    // 检查限位状态（使用中断标志）
    uint8_t allow_motion = 1; // 默认允许运动
    
    if (direction == 0x01) { // 下降
        if (lower_limit_triggered) {
            allow_motion = 0; // 下限位已触发，不允许下降
        }
    } else if (direction == 0x00) { // 上升  
        if (upper_limit_triggered) {
            allow_motion = 0; // 上限位已触发，不允许上升
        }
    }
    
    if (!allow_motion) {
        // 限位触发，停止电机并返回错误
        Motor_Disable();
        motor_enabled = 0;
        response_content[0] = 0x00; // 限位触发错误
    } else {
        // 设置电机方向和速度
        Motor_SetDirection(direction);
        Motor_SetSpeedFromInput(speed_value);
        Motor_Enable();
        
        // 更新状态变量
        motor_enabled = 1;
        motor_current_speed = speed_value;
        motor_current_direction = direction;
        
        response_content[0] = 0x01; // 成功
    }
    
    // 发送响应帧
    send_response_frame(0x01, response_len, response_content);
    }
     
		//RFID位移指定的脉冲距离（0x02）
		else if (rx_cmd == 0x02 && rx_content_index >= 5) {
        // 先停止任何正在进行的连续运动
//        if (motor_enabled && !motor_distance_mode) {
//            Motor_Disable();
//            motor_enabled = 0;
//        }
        
        // 解析指令内容: [运动方向] [运动距离高字节] [运动距离低字节] [速度高字节] [速度低字节]
        uint8_t direction = rx_content[0];
        uint16_t distance = (rx_content[1] << 8) | rx_content[2];
        uint16_t speed_value = (rx_content[3] << 8) | rx_content[4];
        
        // 边界检查
        if (speed_value > 3000) {
            speed_value = 3000;
        }
        
        // 定距离运动不检查限位，直接启动
        Motor_DistanceMove(direction, distance, speed_value);
        
        response_content[0] = 0x01; // 成功
        
        // 发送响应帧
        send_response_frame(0x02, response_len, response_content);
    }
		// 电磁铁开关指令（0x03）
		else if (rx_cmd == 0x03 && rx_content_index >= 1) {
        // 根据内容控制
        if (rx_content[0] == 0x01) {
					// 打开电磁铁
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
            response_content[0] = 0x01; // 成功响应
        } 
        else if (rx_content[0] == 0x00) {
            // 关闭电磁铁
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
            response_content[0] = 0x01; // 成功响应
        }
        else {
            response_content[0] = 0x00; // 失败响应
        }
        
        // 发送响应帧 (指令码0x03)
        send_response_frame(0x03, response_len, response_content);
    }
		// RFID运动状态查询指令 (0x08)
    else if (rx_cmd == 0x08) {
        // 检查电机是否在运动（连续运动或定距离运动）
        if (motor_enabled) {
            response_content[0] = 0x01; // 电机在运动
        } else {
            response_content[0] = 0x00; // 电机停止
        }
        
        // 发送响应帧
        send_response_frame(0x08, response_len, response_content);
    }
    
    // 重置接收状态
    rx_content_index = 0;
}

// 串口接收完成回调
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
			uint8_t data = uart_rxByte;

        //防数据丢失：无论状态如何，只要遇到连续两个FE，就重新开始
        static uint8_t prevByte = 0;
        if (prevByte == FRAME_HEADER_1 && data == FRAME_HEADER_2) {
            rxState = RX_STATE_WAIT_LENGTH;
            rx_content_index = 0;
            prevByte = data;
            HAL_UART_Receive_IT(&huart2, &uart_rxByte, 1);
            return;
        }
        prevByte = data;
				
        switch (rxState) {
            case RX_STATE_WAIT_HEADER1:
                if (data == FRAME_HEADER_1) {
                    rxState = RX_STATE_WAIT_HEADER2;
                }
                break;
                
            case RX_STATE_WAIT_HEADER2:
                if (data == FRAME_HEADER_2) {
                    rxState = RX_STATE_WAIT_LENGTH;
                } else {
                    rxState = RX_STATE_WAIT_HEADER1;
                }
                break;
                
            case RX_STATE_WAIT_LENGTH:
                rx_len = data;
                rx_content_index = 0;
                
                if (rx_len >= 3) {  // 至少包含ID、指令和结束符
                    rxState = RX_STATE_WAIT_ID;  // 新增状态
                } else {
                    rxState = RX_STATE_WAIT_HEADER1;  // 无效长度
                }
                break;
                
            case RX_STATE_WAIT_ID: //等待ID状态
                rx_id = data;  // 存储接收到的ID
                rxState = RX_STATE_WAIT_CMD;
                break;
                
            case RX_STATE_WAIT_CMD:
                rx_cmd = data;
                if (rx_len > 3) {  // 接收 (长度-ID-指令-结束符)
                    rxState = RX_STATE_WAIT_CONTENT;
                } else {
                    rxState = RX_STATE_WAIT_END;
                    frameReceived = 1;
                }
                break;
                
            case RX_STATE_WAIT_CONTENT:
                if (rx_content_index < sizeof(rx_content)) {
                    rx_content[rx_content_index++] = data;
                } else {
                    rxState = RX_STATE_WAIT_HEADER1;  // 缓冲区溢出，重置状态
                }
								
                // 检查是否接收完所有内容 (长度 = ID1字节 + 指令1字节 + 内容n字节 + 结束符1字节)
                if (rx_content_index >= (rx_len - 3)) {  
                    rxState = RX_STATE_WAIT_END;
                }
                break;
                
            case RX_STATE_WAIT_END:
                if (data == FRAME_END) {  // 完整帧接收完成
                    frameReceived = 1;
                }
                rxState = RX_STATE_WAIT_HEADER1;
                break;
								
						 default:
                rxState = RX_STATE_WAIT_HEADER1;
                break;
        }
        
        // 重新启动接收
        HAL_UART_Receive_IT(&huart2, &uart_rxByte, 1);
    }
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	Motor_Init();
  // 初始化限位中断标志（读取当前状态）
  upper_limit_triggered = (HAL_GPIO_ReadPin(SW_RFID_U_GPIO_Port, SW_RFID_U_Pin) == GPIO_PIN_RESET);
  lower_limit_triggered = (HAL_GPIO_ReadPin(SW_RFID_D_GPIO_Port, SW_RFID_D_Pin) == GPIO_PIN_RESET);
	upper_limit_stable = upper_limit_raw;
  lower_limit_stable = lower_limit_raw;
  upper_limit_triggered = upper_limit_raw;
  lower_limit_triggered = lower_limit_raw;
  upper_limit_debounce_counter = DEBOUNCE_COUNT;
  lower_limit_debounce_counter = DEBOUNCE_COUNT;

  HAL_TIM_Base_Start_IT(&htim1);  // 启用TIM1更新中断（用于脉冲计数）
	HAL_TIM_Base_Start_IT(&htim4);  // 启用TIM4更新中断（用于限位消抖）
	
	HAL_UART_Receive_IT(&huart2, &uart_rxByte, 1);  // 启动串口2接收中断
	
	Motor_ChangeSubdivision(8);  //细分设置
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(frameReceived) {
        process_protocol_frame();
        frameReceived = 0;
    }
				HAL_Delay(1);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// 定时器更新中断回调函数 - （较于上一版合并处理了TIM1和TIM4）
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) {
        // TIM1：用于脉冲计数（定距离运动模式）
        if (motor_distance_mode && motor_enabled) {
            current_pulse_count++;
            
            // 检查是否达到目标脉冲数
            if (current_pulse_count >= target_pulse_count) {
                // 达到目标距离，停止电机
                Motor_Disable();
                motor_enabled = 0;
                motor_distance_mode = 0;
            }
        }
    }
    else if (htim->Instance == TIM4) {
        // TIM4：用于限位消抖处理（10ms周期）
        // 上限位消抖处理
        if (upper_limit_debounce_counter < DEBOUNCE_COUNT) {
            upper_limit_debounce_counter++;
            
            if (upper_limit_debounce_counter == DEBOUNCE_COUNT) {
                // 消抖完成，更新稳定状态
                uint8_t new_upper_state = upper_limit_raw;
                
                if (new_upper_state != upper_limit_stable) {
                    upper_limit_stable = new_upper_state;
                    upper_limit_triggered = new_upper_state;
                    
                    // 如果限位触发且电机正在上升，立即停止（仅限连续运动模式）
                    if (upper_limit_triggered && !motor_distance_mode && motor_enabled && motor_current_direction == 0x00) {
                        Motor_Disable();
                        motor_enabled = 0;
                        motor_current_speed = 0;
                    }
                }
            }
        }
        
        // 下限位消抖处理
        if (lower_limit_debounce_counter < DEBOUNCE_COUNT) {
            lower_limit_debounce_counter++;
            
            if (lower_limit_debounce_counter == DEBOUNCE_COUNT) {
                // 消抖完成，更新稳定状态
                uint8_t new_lower_state = lower_limit_raw;
                
                if (new_lower_state != lower_limit_stable) {
                    lower_limit_stable = new_lower_state;
                    lower_limit_triggered = new_lower_state;
                    
                    // 如果限位触发且电机正在下降，立即停止（仅限连续运动模式）
                    if (lower_limit_triggered && !motor_distance_mode && motor_enabled && motor_current_direction == 0x01) {
                        Motor_Disable();
                        motor_enabled = 0;
                        motor_current_speed = 0;
                    }
                }
            }
        }
    }
}

// 外部中断回调函数 - 仅记录原始状态
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch(GPIO_Pin) {
        case SW_RFID_U_Pin:  // 上限位 PB3
            upper_limit_raw = (HAL_GPIO_ReadPin(SW_RFID_U_GPIO_Port, SW_RFID_U_Pin) == GPIO_PIN_RESET);
            upper_limit_debounce_counter = 0; // 重置消抖计数器
            break;
            
        case SW_RFID_D_Pin:  // 下限位 PB4
            lower_limit_raw = (HAL_GPIO_ReadPin(SW_RFID_D_GPIO_Port, SW_RFID_D_Pin) == GPIO_PIN_RESET);
            lower_limit_debounce_counter = 0; // 重置消抖计数器
            break;
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
