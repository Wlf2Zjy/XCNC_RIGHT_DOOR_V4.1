#include "Rfid_control.h"
#include <string.h> 
#include "stm32f1xx_hal.h" 

// RFID 协议定义
#define RFID_FRAME_HEADER 0xAA
#define RFID_FRAME_END 0x55
#define RFID_CMD_READ_POWER 0x01
#define RFID_CMD_SET_POWER 0x02 

// RFID 响应超时时间 (毫秒)
#define RFID_RECEIVE_TIMEOUT 1000 // 软件超时

// 全局变量用于非阻塞接收 (huart1)
uint8_t g_rfid_rx_buffer[RFID_MAX_RX_LEN];
volatile uint8_t g_rfid_rx_len = 0;
volatile uint8_t g_rfid_rx_flag = 0; // 0: 接收中/空闲, 1: 接收完成

/**
 * @brief 在等待接收完成标志时进行阻塞延时。
 * @param timeout 超时时间 (毫秒)。
 * @retval 0xFF 超时, 0x00 接收成功。
 */
static uint8_t wait_for_rx_complete(uint32_t timeout)
{
    uint32_t startTick = HAL_GetTick();
    while (g_rfid_rx_flag == 0)
    {
        // 允许系统执行其他任务，但在此处忙等待
        if ((HAL_GetTick() - startTick) > timeout)
        {
            // 超时处理：在超时后，需要停止中断接收，否则下次调用会混乱
            HAL_UART_AbortReceive_IT(&huart1); 
            return 0xFF; // 超时
        }
    }
    return 0x00; // 接收成功
}

/**
 * @brief 从RFID模块读取功率值。
 * @retval uint8_t 功率值 (0x00-0xFF)。如果超时或接收失败，返回 0xFF。
 */
uint8_t RFID_ReadPower(void)
{
    const uint8_t RX_EXPECTED_LEN = 6;
    // F103 -> RFID: AA 02 01 55 (帧头 AA, 长度 02, 指令 01, 帧尾 55)
    uint8_t tx_buffer[] = {RFID_FRAME_HEADER, 0x02, RFID_CMD_READ_POWER, RFID_FRAME_END};
    
    // 重置接收状态
    g_rfid_rx_flag = 0;
    g_rfid_rx_len = 0;
    memset(g_rfid_rx_buffer, 0, RX_EXPECTED_LEN);

    // 启动非阻塞接收 (预期接收 RX_EXPECTED_LEN 字节)
    if (HAL_UART_Receive_IT(&huart1, g_rfid_rx_buffer, RX_EXPECTED_LEN) != HAL_OK)
    {
        return 0xFF; // 启动接收失败
    }

    // 发送读取功率指令
    HAL_UART_Transmit(&huart1, tx_buffer, sizeof(tx_buffer), 100); 

    // 等待接收完成 (使用软件超时)
    if (wait_for_rx_complete(RFID_RECEIVE_TIMEOUT) == 0xFF)
    {
        return 0xFF; // 超时
    }

    // 校验接收到的数据 (g_rfid_rx_buffer 存储了接收到的数据)
    // 预期: AA 04 01 00 [Power] 55
    if (g_rfid_rx_len == RX_EXPECTED_LEN &&          // 长度匹配
        g_rfid_rx_buffer[0] == RFID_FRAME_HEADER &&  // 帧头 AA
        g_rfid_rx_buffer[1] == 0x04 &&               // 长度 04
        g_rfid_rx_buffer[2] == RFID_CMD_READ_POWER && // 指令 01
        g_rfid_rx_buffer[3] == 0x00 &&               // 状态码 0x00 (成功)
        g_rfid_rx_buffer[5] == RFID_FRAME_END)       // 帧尾 55
    {
        // 校验成功，返回功率值 (位于索引 4)
        return g_rfid_rx_buffer[4]; 
    }
    else
    {
        // 校验失败 (长度不匹配、帧格式错误或RFID返回错误码)
        return 0xFE; 
    }
}

/**
 * @brief 设置RFID模块的功率值。
 * @param power 待设置的功率值 (由上位机传入)。
 * @return uint8_t 0x01 表示设置成功。0xFF表示超时/接收失败，0xFE表示校验失败/RFID返回错误。
 */
uint8_t RFID_SetPower(uint8_t power)
{
    const uint8_t RX_EXPECTED_LEN = 5;
    // F103 -> RFID: AA 04 02 01 [功率] 55 
    uint8_t tx_buffer[] = {RFID_FRAME_HEADER, 0x04, RFID_CMD_SET_POWER, 0x01, power, RFID_FRAME_END};
    
    //重置接收状态
    g_rfid_rx_flag = 0;
    g_rfid_rx_len = 0;
    memset(g_rfid_rx_buffer, 0, RX_EXPECTED_LEN);

    // 启动非阻塞接收 (预期接收 RX_EXPECTED_LEN 字节)
    if (HAL_UART_Receive_IT(&huart1, g_rfid_rx_buffer, RX_EXPECTED_LEN) != HAL_OK)
    {
        return 0xFF; // 启动接收失败
    }

    // 通过 USART1 发送设置功率指令
    HAL_UART_Transmit(&huart1, tx_buffer, sizeof(tx_buffer), 100); 

    // 等待接收完成 (使用软件超时)
    if (wait_for_rx_complete(RFID_RECEIVE_TIMEOUT) == 0xFF)
    {
        return 0xFF; // 超时
    }

    //校验接收到的数据 预期: AA 03 02 00 55
    if (g_rfid_rx_len == RX_EXPECTED_LEN &&          // 长度匹配
        g_rfid_rx_buffer[0] == RFID_FRAME_HEADER &&  // 帧头 AA
        g_rfid_rx_buffer[1] == 0x03 &&               // 长度 03
        g_rfid_rx_buffer[2] == RFID_CMD_SET_POWER && // 指令 02
        g_rfid_rx_buffer[3] == 0x00 &&               // 状态 00 (成功)
        g_rfid_rx_buffer[4] == RFID_FRAME_END)       // 帧尾 55
    {
        // 校验成功
        return 0x01; // 设置成功标志
    }
    else
    {
        // 内容校验失败 (例如 RFID 返回了非 0x00 的状态码)
        return 0xFE; 
    }
}
