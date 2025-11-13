#include "Rfid_control.h"
#include <string.h> 
#include "stm32f1xx_hal.h" 

extern UART_HandleTypeDef huart1; 

// RFID 协议定义
#define RFID_FRAME_HEADER 0xAA
#define RFID_FRAME_END 0x55
#define RFID_CMD_READ_POWER 0x01
#define RFID_CMD_SET_POWER 0x02 
#define RFID_CMD_START_CYCLE 0x10 
#define RFID_CMD_STOP_CYCLE 0x12  

// RFID 响应超时时间 (毫秒)
#define RFID_ACK_TIMEOUT 500 
#define TAG_FRAME_TIMEOUT_MS 100 

// 全局接收缓冲区 (结果存储)
uint8_t g_rfid_rx_buffer[RFID_MAX_RX_LEN];

/**
 * @brief 尝试清除 UART 接收缓冲区中所有残留数据。
 * 使用非阻塞轮询读取，直到缓冲区为空或达到最大清除时间。
 */
static void flush_uart_rx_buffer(void)
{
    uint8_t dummy_data;
    uint32_t startTick = HAL_GetTick();
    const uint32_t MAX_FLUSH_TIME_MS = 50; // 最大清除时间
    
    // 循环尝试读取，使用最短的超时时间（例如1ms），直到 HAL_UART_Receive 返回非 HAL_OK
    while (HAL_UART_Receive(&huart1, &dummy_data, 1, 1) == HAL_OK)
    {
        // 检查是否超过最大清除时间，防止卡死
        if (HAL_GetTick() - startTick > MAX_FLUSH_TIME_MS) {
             break;
        }
    }
}


/**
 * @brief 发送指令并阻塞等待预期长度的响应 (使用 HAL_UART_Receive 轮询模式)。
 * @param tx_buffer 发送缓冲区
 * @param tx_len 发送长度
 * @param rx_expected_len 预期接收长度
 * @param rx_timeout 接收超时时间
 * @param actual_rx_len 存储实际接收到的字节数
 * @retval 0x00: 接收成功, 0xFF: 超时/失败。
 */
static uint8_t send_and_receive_blocking(uint8_t* tx_buffer, uint8_t tx_len, uint8_t rx_expected_len, uint32_t rx_timeout, uint8_t *actual_rx_len)
{
    // 1. 清空接收缓冲区，确保每次操作干净
    memset(g_rfid_rx_buffer, 0, RFID_MAX_RX_LEN);
    *actual_rx_len = 0;
    
    // 2. 发送指令 (阻塞发送)
    if (tx_buffer != NULL) {
        // 使用阻塞发送，设置一个短超时
        if (HAL_UART_Transmit(&huart1, tx_buffer, tx_len, 100) != HAL_OK)
        {
            return 0xFF; // 发送失败
        }
    }

    // 3. 阻塞接收预期长度数据 (使用 HAL_UART_Receive 轮询)
    HAL_StatusTypeDef status = HAL_UART_Receive(&huart1, g_rfid_rx_buffer, rx_expected_len, rx_timeout);
    
    if (status == HAL_OK)
    {
        // 轮询接收成功，实际接收长度就是预期长度
        *actual_rx_len = rx_expected_len; 
        return 0x00; // 接收成功
    }
    else if (status == HAL_TIMEOUT)
    {
        // 超时，此时实际接收长度不确定，可能为0或部分数据
        *actual_rx_len = rx_expected_len - huart1.RxXferCount; 
        return 0xFF; // 超时
    }
    
    // 其他错误 (如 HAL_BUSY, HAL_ERROR)
    return 0xFF; 
}


/**
 * @brief 尝试在短时间内接收一帧完整的标签数据 (使用 HAL_UART_Receive 轮询模式)。
 * @param rx_expected_len 预期接收长度 (标签数据帧应为 23 bytes)
 * @param timeout_ms 接收超时时间 (短时间，如 100ms)
 * @param actual_rx_len 存储实际接收到的字节数
 * @retval 0x00: 接收成功, 0x01: 超时/无数据, 0xFF: 启动接收失败。
 */
static uint8_t wait_for_tag_frame(uint8_t rx_expected_len, uint32_t timeout_ms, uint8_t *actual_rx_len)
{
    // 1. 清空接收缓冲区
    memset(g_rfid_rx_buffer, 0, RFID_MAX_RX_LEN);
    *actual_rx_len = 0;
    
    // 2. 阻塞接收预期长度数据 (使用短超时)
    HAL_StatusTypeDef status = HAL_UART_Receive(&huart1, g_rfid_rx_buffer, rx_expected_len, timeout_ms);
    
    if (status == HAL_OK)
    {
        // 接收成功
        *actual_rx_len = rx_expected_len;
        return 0x00; 
    }
    else if (status == HAL_TIMEOUT)
    {
        // 超时，检查是否接收到部分数据 
        *actual_rx_len = rx_expected_len - huart1.RxXferCount; 
        
        // 如果实际接收长度小于预期长度，视为超时/无数据
        if (*actual_rx_len < rx_expected_len) {
            return 0x01; 
        } else {
             // 理论上不应发生
             return 0xFF;
        }
    }
    
    // 其他错误 (如 HAL_BUSY, HAL_ERROR)
    return 0xFF; 
}


/**
 * @brief 从RFID模块读取功率值。
 * @retval uint8_t 功率值 (0x00-0xFF)。如果超时或接收失败，返回 0xFF。
 */
uint8_t RFID_ReadPower(void)
{
    uint8_t result = 0xFF;
    const uint8_t RX_EXPECTED_LEN = 6;
    uint8_t g_rfid_rx_len = 0; // 局部变量，存储实际接收长度
    // F103 -> RFID: AA 02 01 55 
    uint8_t tx_buffer[] = {RFID_FRAME_HEADER, 0x02, RFID_CMD_READ_POWER, RFID_FRAME_END};
    
    // 调用发送接收函数 (阻塞轮询)
    if (send_and_receive_blocking(tx_buffer, sizeof(tx_buffer), RX_EXPECTED_LEN, RFID_ACK_TIMEOUT, &g_rfid_rx_len) != 0x00)
    {
        result = 0xFF; // 超时或接收失败
    }
    // 校验接收到的数据 预期: AA 04 01 00 [Power] 55
    else if (g_rfid_rx_len == RX_EXPECTED_LEN &&          
        g_rfid_rx_buffer[0] == RFID_FRAME_HEADER &&  
        g_rfid_rx_buffer[1] == 0x04 &&               
        g_rfid_rx_buffer[2] == RFID_CMD_READ_POWER && 
        g_rfid_rx_buffer[3] == 0x00 &&               
        g_rfid_rx_buffer[5] == RFID_FRAME_END)       
    {
        // 校验成功，返回功率值 (位于索引 4)
        result = g_rfid_rx_buffer[4]; 
    }
    else
    {
        result = 0xFE; // 校验失败
    }
    
    // 强制清除 UART 缓冲区
    flush_uart_rx_buffer(); 
    return result;
}

/**
 * @brief 设置RFID模块的功率值。
 * @param power 待设置的功率值 (由上位机传入)。
 * @return uint8_t 0x01 表示设置成功。0xFF表示超时/接收失败，0xFE表示校验失败/RFID返回错误。
 */
uint8_t RFID_SetPower(uint8_t power)
{
    uint8_t result = 0xFF;
    const uint8_t RX_EXPECTED_LEN = 5;
    uint8_t g_rfid_rx_len = 0; // 局部变量，存储实际接收长度
    // F103 -> RFID: AA 04 02 01 [功率] 55 
    uint8_t tx_buffer[] = {RFID_FRAME_HEADER, 0x04, RFID_CMD_SET_POWER, 0x01, power, RFID_FRAME_END};
    
    if (send_and_receive_blocking(tx_buffer, sizeof(tx_buffer), RX_EXPECTED_LEN, RFID_ACK_TIMEOUT, &g_rfid_rx_len) != 0x00)
    {
        result = 0xFF; // 超时或接收失败
    }
    // 校验接收到的数据 预期: AA 03 02 00 55
    else if (g_rfid_rx_len == RX_EXPECTED_LEN &&          
        g_rfid_rx_buffer[0] == RFID_FRAME_HEADER &&  
        g_rfid_rx_buffer[1] == 0x03 &&               
        g_rfid_rx_buffer[2] == RFID_CMD_SET_POWER && 
        g_rfid_rx_buffer[3] == 0x00 &&               
        g_rfid_rx_buffer[4] == RFID_FRAME_END)       
    {
        result = 0x01; // 设置成功标志
    }
    else
    {
        result = 0xFE; // 校验失败
    }
    
    // 强制清除 UART 缓冲区
    flush_uart_rx_buffer(); 
    return result;
}


/**
 * @brief 校验并提取EPC数据。
 * @param buffer 接收缓冲区。
 * @param len 接收长度。
 * @param epc_data 存储提取到的16字节EPC数据。
 * @retval 0x00: 提取成功, 0xFF: 校验失败。
 */
static uint8_t parse_and_extract_epc(uint8_t* buffer, uint8_t len, uint8_t epc_data[EPC_DATA_LEN])
{
    const uint8_t RX_EXPECTED_LEN = 23;
    const uint8_t EPC_START_INDEX = 6; 
    
    // 预期: AA 15 10 00 [PC 2B] [EPC 16B] 55
    if (len == RX_EXPECTED_LEN &&          
        buffer[0] == RFID_FRAME_HEADER &&  
        buffer[1] == 0x15 &&               // 长度 0x15 (21 bytes)
        buffer[2] == RFID_CMD_START_CYCLE && // 指令 10
        buffer[3] == 0x00 &&               // 状态码 0x00 (成功)
        buffer[RX_EXPECTED_LEN - 1] == RFID_FRAME_END) // 帧尾 55
    {
        // 校验成功，提取EPC数据 (从索引 6 开始，共 16 字节)
        memcpy(epc_data, &buffer[EPC_START_INDEX], EPC_DATA_LEN);
        return 0x00; 
    }
    else
    {
        return 0xFF; // 校验失败
    }
}

/**
 * @brief 循环读取RFID信息，直到达到指定次数，并统计出现频率。
 * @param read_count 总共读取的数据条数。
 * @param threshold_count 返回出现出现次数大于此阈值的EPC。
 * @param result_epc_data 存储最终满足阈值条件的16字节EPC数据，如果没有满足条件的则全为0。
 * @return uint8_t 0x01: 成功，0xFF: 超时/失败。
 */
uint8_t RFID_CyclicRead(uint8_t read_count, uint8_t threshold_count, uint8_t result_epc_data[EPC_DATA_LEN])
{
    // 将所有变量声明移至函数开头，避免 goto 导致的初始化警告
    EPC_Data_t stats[MAX_UNIQUE_TAGS];
    uint8_t unique_tag_count = 0;
    uint16_t successful_reads = 0; 
    uint8_t epc_buffer[EPC_DATA_LEN];
    uint8_t i;
    uint8_t result_status = 0xFF;
    uint8_t g_rfid_rx_len = 0; // 局部变量，存储实际接收长度
    
    // 1. 初始化统计数组和结果
    memset(stats, 0, sizeof(stats));
    memset(result_epc_data, 0, EPC_DATA_LEN);

    // 2. 发送启动循环读取指令: AA 02 10 55
    uint8_t start_tx[] = {RFID_FRAME_HEADER, 0x02, RFID_CMD_START_CYCLE, RFID_FRAME_END};
    
    // 预期确认帧: AA 03 10 01 55 (5 bytes)
    const uint8_t CONFIRM_RX_LEN = 5;
    if (send_and_receive_blocking(start_tx, sizeof(start_tx), CONFIRM_RX_LEN, RFID_ACK_TIMEOUT, &g_rfid_rx_len) != 0x00)
    {
        // 启动失败或确认帧超时
        result_status = 0xFF; 
        goto cleanup;
    }
    // 确认确认帧格式: AA 03 10 01 55
    if (!(g_rfid_rx_len == CONFIRM_RX_LEN && 
          g_rfid_rx_buffer[0] == RFID_FRAME_HEADER &&
          g_rfid_rx_buffer[2] == RFID_CMD_START_CYCLE &&
          g_rfid_rx_buffer[3] == 0x01 && // 状态码 01 (确认)
          g_rfid_rx_buffer[4] == RFID_FRAME_END))
    {
        // 确认帧校验失败
        result_status = 0xFF;
        goto cleanup;
    }

    // 3. 循环读取标签数据 (流式接收)
    const uint8_t TAG_RX_LEN = 23;

    while (successful_reads < read_count)
    {
        // 尝试等待下一帧标签数据，设置短超时 (阻塞轮询)
        uint8_t wait_status = wait_for_tag_frame(TAG_RX_LEN, TAG_FRAME_TIMEOUT_MS, &g_rfid_rx_len);
        
        if (wait_status == 0x00) // 接收成功
        {
            // 校验并提取 EPC
            if (parse_and_extract_epc(g_rfid_rx_buffer, g_rfid_rx_len, epc_buffer) == 0x00)
            {
                successful_reads++;

                // 统计 EPC 出现频率
                uint8_t found = 0;
                for (i = 0; i < unique_tag_count; i++)
                {
                    if (memcmp(stats[i].epc_data, epc_buffer, EPC_DATA_LEN) == 0)
                    {
                        if (stats[i].count < 255) { stats[i].count++; } // 防止溢出
                        found = 1;
                        break;
                    }
                }

                if (!found && unique_tag_count < MAX_UNIQUE_TAGS)
                {
                    // 存储新的唯一 EPC (如果统计数组未满)
                    // 注意：这里的结构体字段名应为 epc_data
                    memcpy(stats[unique_tag_count].epc_data, epc_buffer, EPC_DATA_LEN);
                    stats[unique_tag_count].count = 1;
                    stats[unique_tag_count].is_valid = 1;
                    unique_tag_count++;
                }
            }
        }
        else if (wait_status == 0x01) // 超时/无数据
        {
            // 模块停止流式发送，或暂时无标签，退出循环
            // 如果成功读取到数据，则视为成功退出
            if (successful_reads > 0) {
                 result_status = 0x01; 
            }
            goto stop_and_exit; 
        }
        else // 0xFF: 接收失败
        {
            result_status = 0xFF;
            goto stop_and_exit; 
        }
    }
    
    result_status = 0x01; // 达到读取次数，成功完成循环

// 4. 发送停止循环读取指令: AA 02 12 55
stop_and_exit: ;
    uint8_t stop_tx[] = {RFID_FRAME_HEADER, 0x02, RFID_CMD_STOP_CYCLE, RFID_FRAME_END};
    
    // 预期确认帧: AA 03 12 00 55 (5 bytes)
    const uint8_t STOP_CONFIRM_RX_LEN = 5;
    
    // 尝试发送停止指令并等待确认 (阻塞轮询)
    if (send_and_receive_blocking(stop_tx, sizeof(stop_tx), STOP_CONFIRM_RX_LEN, RFID_ACK_TIMEOUT, &g_rfid_rx_len) == 0x00)
    {
        // 确认停止帧格式: AA 03 12 00 55
        if (!(g_rfid_rx_len == STOP_CONFIRM_RX_LEN && 
              g_rfid_rx_buffer[0] == RFID_FRAME_HEADER &&
              g_rfid_rx_buffer[2] == RFID_CMD_STOP_CYCLE &&
              g_rfid_rx_buffer[3] == 0x00 && // 状态码 00 (成功)
              g_rfid_rx_buffer[4] == RFID_FRAME_END))
        {
             // 停止确认帧校验失败，但如果之前读取成功，不影响结果状态
             if (result_status != 0x01) {
                 result_status = 0xFF; 
             }
        }
    }
    else
    {
        // 停止指令发送失败或确认超时
        if (result_status != 0x01) {
             result_status = 0xFF; 
        }
    }
    
// 5. 查找满足阈值条件的 EPC
    uint8_t found_threshold = 0;
    for (i = 0; i < unique_tag_count; i++)
    {
        if (stats[i].count >= threshold_count)
        {
            // 找到第一个满足条件的 EPC，复制到结果缓冲区
            memcpy(result_epc_data, stats[i].epc_data, EPC_DATA_LEN);
            found_threshold = 1;
            break; 
        }
    }
    
    if (found_threshold || result_status == 0x01)
    {
        // 如果成功完成读取次数，或者找到了满足条件的EPC，返回成功
        result_status = 0x01;
    } else {
        result_status = 0xFF; // 确保失败时返回 0xFF
    }

cleanup:
    // 强制清除 UART 缓冲区，确保下次通信环境干净
    flush_uart_rx_buffer(); 
    return result_status;
}
