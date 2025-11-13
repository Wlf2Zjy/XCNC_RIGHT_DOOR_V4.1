#ifndef __RFID_CONTROL_H
#define __RFID_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "usart.h" // 确保 huart1 被包含进来

// RFID 命令定义
#define RFID_CMD_START_CYCLE 0x10 // 循环读取标签
#define RFID_CMD_STOP_CYCLE 0x12  // 停止循环读取

// EPC 标签数据长度 (16字节)
#define EPC_DATA_LEN 16

// 统计结构体最大数量 (修复：之前在 .c 文件中，现在移到 .h 文件)
#define MAX_UNIQUE_TAGS 5

// 全局接收缓冲区 (用于阻塞接收结果)
#define RFID_MAX_RX_LEN 25 // 预期最大接收长度 (Tag Data Frame: 23 bytes)
extern uint8_t g_rfid_rx_buffer[RFID_MAX_RX_LEN];

/**
 * @brief 用于存储和统计单个 EPC 标签数据及其出现次数。
 */
typedef struct {
    uint8_t epc_data[EPC_DATA_LEN];
    uint8_t count; // 出现次数
    uint8_t is_valid; // 是否已存储有效数据
} EPC_Data_t;


/**
 * @brief 从RFID模块读取功率值。
 * 此函数将通过 USART1 发送读取指令，并阻塞等待响应。
 * @return uint8_t 功率值 (0x00-0xFF)。如果超时或接收失败，返回 0xFF。
 */
uint8_t RFID_ReadPower(void);

/**
 * @brief 设置RFID模块的功率值。
 * @param power 待设置的功率值 (由上位机传入)。
 * @return uint8_t 0x01 表示设置成功。0xFF表示超时/接收失败，0xFE表示校验失败/RFID返回错误。
 */
uint8_t RFID_SetPower(uint8_t power);

/**
 * @brief 循环读取RFID信息，直到达到指定次数，并统计出现频率。
 * @param read_count 总共读取的数据条数。
 * @param threshold_count 返回出现出现次数大于此阈值的EPC。
 * @param result_epc_data 存储最终满足阈值条件的16字节EPC数据，如果没有满足条件的则全为0。
 * @return uint8_t 0x01: 成功，0xFF: 超时/失败。
 */
uint8_t RFID_CyclicRead(uint8_t read_count, uint8_t threshold_count, uint8_t result_epc_data[EPC_DATA_LEN]);


#ifdef __cplusplus
}
#endif

#endif // __RFID_CONTROL_H
