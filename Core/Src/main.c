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
#include "Rfid_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FRAME_HEADER_1 0xFE     // 帧头第一个字节
#define FRAME_HEADER_2 0xFE     // 帧头第二个字节
#define FRAME_END 0xFA          // 帧结束字节
#define EPC_DATA_LEN 16        // EPC 数据长度 (16字节)

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
    uint8_t response_content[EPC_DATA_LEN + 1] = {0}; 
    uint8_t response_len = 1;
		switch (rx_cmd)
    {
    // 0x01 — RFID控制指令
    case 0x01:
        if (rx_content_index >= 3)
        {
            uint8_t direction = rx_content[0];  // 解析指令内容: [方向(1字节)] [速度高字节] [速度低字节]
            uint16_t speed_value = (rx_content[1] << 8) | rx_content[2];

            // 边界设置在3000以内
            if (speed_value > 3000)
                speed_value = 3000;
             // 检查限位状态（使用中断标志）
            uint8_t allow_motion = 1; // 默认允许运动

            // 检查限位
            switch (direction)
            {
                case 0x01: // 下降
                    if (lower_limit_triggered)
                        allow_motion = 0;  // 下限位已触发，不允许下降
                    break;
                case 0x00: // 上升
                    if (upper_limit_triggered)
                        allow_motion = 0;  // 上限位已触发，不允许上升
                    break;
                default:
                    allow_motion = 0; // 非法方向
                    break;
            }

            if (!allow_motion)
            {
                Motor_Disable();
                motor_enabled = 0;
                response_content[0] = 0x00; // 限位触发或非法方向
            }
            else
            {
                Motor_SetDirection(direction);  //设置电机方向和速度
                Motor_SetSpeedFromInput(speed_value);
                Motor_Enable();

                motor_enabled = 1;  //更新状态变量
                motor_current_speed = speed_value;
                motor_current_direction = direction;

                response_content[0] = 0x01; // 成功
            }

            send_response_frame(0x01, response_len, response_content);  //发送响应帧
        }
        break;

    // 0x02 — RFID位移指定脉冲距离
    case 0x02:
        if (rx_content_index >= 5)
        {
					  // 解析指令内容: [运动方向] [运动距离高字节] [运动距离低字节] [速度高字节] [速度低字节]
            uint8_t direction = rx_content[0];
            uint16_t distance = (rx_content[1] << 8) | rx_content[2];
            uint16_t speed_value = (rx_content[3] << 8) | rx_content[4];

            if (speed_value > 3000)
                speed_value = 3000;

            // 定距离运动,不检查限位，直接启动
            Motor_DistanceMove(direction, distance, speed_value);

            response_content[0] = 0x01;
            send_response_frame(0x02, response_len, response_content);  //发送响应帧
        }
        break;

    // 0x03 — 电磁铁开关控制
    case 0x03:
        if (rx_content_index >= 1)
        {
            switch (rx_content[0])
            {
                case 0x01:  // 打开电磁铁
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
                    response_content[0] = 0x01;
                    break;

                case 0x00:  // 关闭电磁铁
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
                    response_content[0] = 0x01;
                    break;

                default:
                    response_content[0] = 0x00; // 非法参数
                    break;
            }

            send_response_frame(0x03, response_len, response_content);
        }
        break;
				
		// 0x04 — 单步循环读取RFID信息并统计
    case 0x04:
        if (rx_content_index >= 2)
        {
            uint8_t read_count = rx_content[0];     // 读取次数
            uint8_t threshold_count = rx_content[1]; // 出现阈值
            uint8_t epc_data[EPC_DATA_LEN];         // 存储结果

            // 调用循环读取函数
            // 注意：read_count 和 threshold_count 都是 8bit，最大 255
            uint8_t result_status = RFID_CyclicRead(read_count, threshold_count, epc_data);
            
            // 响应帧为FE FE 13 02 04 [数据内容16个字节] FA
            // 如果成功 (0x01)，则返回 16 字节 EPC 数据
            if (result_status == 0x01) {
                response_len = EPC_DATA_LEN;
                memcpy(response_content, epc_data, EPC_DATA_LEN);
            } else {
                // 如果失败 (0xFF)，返回 16 个 0x00 
                response_len = EPC_DATA_LEN; 
                memset(response_content, 0x00, EPC_DATA_LEN);
            }

            // 返回长度为 16 字节
            send_response_frame(0x04, response_len, response_content);
        } else {
            // 内容长度不足，返回 16 个 0x00
            response_len = EPC_DATA_LEN; 
            memset(response_content, 0x00, EPC_DATA_LEN);
            send_response_frame(0x04, response_len, response_content);
        }
        break;
				
		// 0x05 — 读取RFID功率
    case 0x05:
        {
            // (上位机 -> F103 的指令: FE FE 03 02 05 FA)
            // 通过USART1与RFID通信
            uint8_t power_value = RFID_ReadPower(); //获取功率值
            response_content[0] = power_value; // 放入读取到的功率值
            response_len = 1;                  // 内容长度为1字节
            // (F103 -> 上位机 的响应: FE FE 04 02 05 [power_value] FA)
            send_response_frame(0x05, response_len, response_content);
        }
        break;
				
		// 0x06 — 设置RFID功率 
    case 0x06:
        if (rx_content_index >= 1)
        {
            uint8_t power_value = rx_content[0]; // 上位机指令内容就是功率值
            uint8_t result_code = RFID_SetPower(power_value);   // 通过USART1与RFID通信

            // 上位机期望成功返回 01: FE FE 04 02 06 01 FA
            if (result_code == 0x01) {
                // RFID设置成功
                response_content[0] = 0x01;
            } else {
                //设置失败 (超时 0xFF 或校验失败 0xFE)，返回 0x00 作为通用失败标志
                response_content[0] = 0x00; 
            }
            response_len = 1;

            send_response_frame(0x06, response_len, response_content);
        } else {
            // 内容长度不足 (未收到功率值)，返回失败
            response_content[0] = 0x00;
            response_len = 1;
            send_response_frame(0x06, response_len, response_content);
        }
        break;
				
				// 0x07 — 上位机到 RFID 透明传输 (新增指令)
    case 0x07:
        if (rx_content_index > 0)
        {
            uint8_t tx_len = rx_content_index;
            uint8_t rx_len_from_rfid = 0;
            // 临时缓冲区用于存储RFID的响应数据
            uint8_t rfid_response[64] = {0}; 
            const uint8_t MAX_RFID_RX_LEN = 64;

            // 调用透传函数
            uint8_t result_status = RFID_TransparentTransmit(
                rx_content,                   // 上位机指令内容作为透传数据
                tx_len,                       // 上位机指令内容长度
                rfid_response,                // 接收缓冲区
                MAX_RFID_RX_LEN,              // 缓冲区最大长度
                &rx_len_from_rfid             // 实际接收长度
            );

            // 检查透传结果
            if (result_status == 0x01) {
                // 成功收到数据，将 RFID 响应作为返回内容
                response_len = rx_len_from_rfid;
                // 注意：这里需要确保 rfid_response 的长度不会超过 response_content 的定义 (EPC_DATA_LEN + 1 = 17)
                // 由于 response_content 是在函数开头定义的 uint8_t response_content[EPC_DATA_LEN + 1] = {0};
                // 为了支持不定长透传，需要扩大 response_content 的定义，或者用一个局部的大缓冲区。
                
                // 假设 response_content 已经扩大到足以容纳 64 字节:
                // 由于 response_content 实际是 EPC_DATA_LEN + 1 = 17，这里需要临时使用 rfid_response。
                // 调整：response_content 是在函数开头定义的，如果需要支持 64 字节透传，必须修改它的定义。
                // 为保持兼容性，我们直接将 rfid_response 的内容和长度传递给 send_response_frame。

                // send_response_frame(cmd, return_len, return_content)
                send_response_frame(0x07, response_len, rfid_response);
            } else {
                // 失败 (超时或发送失败)，返回 1 字节失败标志 0x00
                response_len = 1;
                response_content[0] = 0x00; 
                send_response_frame(0x07, response_len, response_content);
            }
        } else {
            // 内容长度为 0 (无效指令)，返回 1 字节失败标志 0x00
            response_len = 1;
            response_content[0] = 0x00; 
            send_response_frame(0x07, response_len, response_content);
        }
        break;

    // 0x08 — RFID运动状态查询
    case 0x08:
        if (motor_enabled)  // 检查电机是否在运动（连续运动或定距离运动）
            response_content[0] = 0x01; // 运动中
        else
            response_content[0] = 0x00; // 停止

        send_response_frame(0x08, response_len, response_content);  // 发送响应帧
        break;

    // 未知指令处理
    default:
        response_content[0] = 0x00; // 未知指令响应
        send_response_frame(rx_cmd, response_len, response_content);
        break;
        }

        // 重置接收状态
        rx_content_index = 0;
}

// 串口接收完成回调
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    //处理 huart1 (RFID) 的中断接收
    if (huart->Instance == USART1)
    {
        // 将接收到的字节存入环形缓冲区 
        rfid_rx_buffer[rfid_rx_head] = rfid_dma_rx_byte;
        rfid_rx_head = (rfid_rx_head + 1) % RFID_RX_BUFFER_SIZE;

        // 重新启动中断接收下一个字节
        HAL_UART_Receive_IT(&huart1, &rfid_dma_rx_byte, 1);
        // --------------------------------------------------
        return; // 结束 USART1 的处理
    }

    //处理 huart2 (485) 的中断接收
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
	
	RFID_Init();
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
