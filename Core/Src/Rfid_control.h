#ifndef __RFID_CONTROL_H
#define __RFID_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif
/*
读取功率：fe fe 07 02 07 aa 02 01 55 fa
设置功率：fe fe 09 02 07 aa 04 02 01 14 55 fa
*/

#include "main.h"
#include "usart.h" 

#define RFID_RX_BUFFER_SIZE 256 // 环形缓冲区大小


extern uint8_t rfid_rx_buffer[RFID_RX_BUFFER_SIZE];
extern volatile uint16_t rfid_rx_head; // 写入指针
extern volatile uint16_t rfid_rx_tail; // 读取指针
extern uint8_t rfid_dma_rx_byte;       // 单字节接收变量 (用于启动中断)

// RFID 命令定义 
#define RFID_CMD_START_CYCLE 0x10 // 循环读取标签
#define RFID_CMD_STOP_CYCLE 0x12  // 停止循环读取

// EPC 标签数据长度 (16字节)
#define EPC_DATA_LEN 16

// 统计结构体最大数量
#define MAX_UNIQUE_TAGS 5

// 全局接收缓冲区 (中断接收目标)
// extern uint8_t g_rfid_rx_buffer[RFID_MAX_RX_LEN]; 

/**
 * @brief 用于存储和统计单个 EPC 标签数据及其出现次数。
 */
typedef struct {
    uint8_t epc_data[EPC_DATA_LEN];
    uint8_t count; // 出现次数
    uint8_t is_valid; // 是否已存储有效数据
} EPC_Data_t;

/**
 * @brief 初始化 RFID 接收（启动中断接收）。
 */
void RFID_Init(void);

/**
 * @brief 从环形缓冲区中读取一个字节（非阻塞）。
 * @param data 存储读取到的字节。
 * @return 1: 成功读取, 0: 缓冲区为空。
 */
uint8_t RFID_ReadByte(uint8_t *data);

 //从RFID模块读取功率值。
uint8_t RFID_ReadPower(void);

 //设置RFID模块的功率值。
uint8_t RFID_SetPower(uint8_t power);

//循环读取RFID信息，直到达到指定次数，并统计出现频率。
uint8_t RFID_CyclicRead(uint8_t read_count, uint8_t threshold_count, uint8_t result_epc_data[EPC_DATA_LEN]);

uint8_t RFID_TransparentTransmit(uint8_t *tx_data, uint8_t tx_len, uint8_t *rx_buffer, uint8_t rx_buffer_max_len, uint8_t *actual_rx_len);


#ifdef __cplusplus
}
#endif

#endif // __RFID_CONTROL_H
