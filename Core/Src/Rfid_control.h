#ifndef __RFID_CONTROL_H
#define __RFID_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "usart.h"

// 全局变量用于非阻塞接收
#define RFID_MAX_RX_LEN 10
extern uint8_t g_rfid_rx_buffer[RFID_MAX_RX_LEN];
extern volatile uint8_t g_rfid_rx_len;
extern volatile uint8_t g_rfid_rx_flag; // 0: 接收中/空闲, 1: 接收完成

/**
 * @brief 从RFID模块读取功率值。
 * 此函数将通过 USART1 发送读取指令，并等待中断接收响应。
 * @return uint8_t 功率值 (0x00-0xFF)。如果超时或接收失败，返回 0xFF。
 */
uint8_t RFID_ReadPower(void);

/**
 * @brief 设置RFID模块的功率值。
 * @param power 待设置的功率值 (由上位机传入)。
 * @return uint8_t 0x01 表示设置成功。0xFF表示超时/接收失败，0xFE表示校验失败/RFID返回错误。
 */
uint8_t RFID_SetPower(uint8_t power);


#ifdef __cplusplus
}
#endif

#endif // __RFID_CONTROL_H
