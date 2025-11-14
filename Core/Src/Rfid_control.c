#include "Rfid_control.h"
#include <string.h> 
#include "stm32f1xx_hal.h" 

extern UART_HandleTypeDef huart1; 

// RFID 协议定义
#define RFID_FRAME_HEADER 0xAA  //RFID帧头
#define RFID_FRAME_END 0x55  //RFID帧尾
#define RFID_CMD_READ_POWER 0x01  //RFID的读取功率指令
#define RFID_CMD_SET_POWER 0x02   //RFID的设置功率指令
#define RFID_CMD_START_CYCLE 0x10   //RFID的单步循环读取数据指令
#define RFID_CMD_STOP_CYCLE 0x12   //RFID的停止指令

// RFID 响应超时时间 (毫秒)
#define RFID_ACK_TIMEOUT 500 
#define TAG_FRAME_TIMEOUT_MS 100 
#define RFID_TRANSPARENT_RX_TIMEOUT 1000 // 定义透传接收超时
#define RFID_TRANSPARENT_IDLE_TIME 5    // 5ms 静默时间

// 临时缓冲区（用于存储待校验帧）
#define RFID_MAX_RX_LEN 25 
static uint8_t s_frame_rx_buffer[RFID_MAX_RX_LEN];

// --- 环形缓冲区实现 ---
uint8_t rfid_rx_buffer[RFID_RX_BUFFER_SIZE];
volatile uint16_t rfid_rx_head = 0; // 写入指针
volatile uint16_t rfid_rx_tail = 0; // 读取指针
uint8_t rfid_dma_rx_byte;           // 单字节接收变量 (用于启动中断)

/**
 * @brief 初始化 RFID 接收（启动中断接收）。
 * 必须在 MX_USART1_UART_Init() 之后调用。
 */
void RFID_Init(void)
{
    // 清空环形缓冲区
    rfid_rx_head = 0;
    rfid_rx_tail = 0;
    // 启动单个字节的 UART 接收中断
    // 当接收到一个字节后，会进入 HAL_UART_RxCpltCallback
    HAL_UART_Receive_IT(&huart1, &rfid_dma_rx_byte, 1);
}

/**
 * @brief 从环形缓冲区中读取一个字节（非阻塞）。
 * @param data 存储读取到的字节。
 * @return 1: 成功读取, 0: 缓冲区为空。
 */
uint8_t RFID_ReadByte(uint8_t *data)
{
    if (rfid_rx_head == rfid_rx_tail)
    {
        return 0; // 缓冲区为空
    }
    
    // 读取数据
    *data = rfid_rx_buffer[rfid_rx_tail];
    // 更新读取指针
    rfid_rx_tail = (rfid_rx_tail + 1) % RFID_RX_BUFFER_SIZE;
    
    return 1; // 读取成功
}

/**
 * @brief 清空环形缓冲区中的所有数据。
 */
static void flush_rfid_rx_buffer(void)
{
    __disable_irq(); // 禁用中断，防止读写指针被同时修改
    rfid_rx_head = rfid_rx_tail;
    __enable_irq();  // 启用中断
}


/**
 * @brief 尝试从环形缓冲区中读取一帧完整的 RFID 响应（非阻塞）。
 * 这是一个阻塞等待函数，但从非阻塞的环形缓冲区读取。
 * @param rx_expected_len 预期接收长度
 * @param rx_timeout 接收超时时间
 * @return 0x00: 接收成功, 0xFF: 超时/失败, 0xFE: 校验失败。
 */
static uint8_t read_frame_from_buffer(uint8_t rx_expected_len, uint32_t rx_timeout)
{
    uint32_t startTick = HAL_GetTick();
    uint8_t byte;
    uint8_t current_len = 0;
    uint8_t frame_started = 0;
    
    // 清空临时缓冲区
    memset(s_frame_rx_buffer, 0, RFID_MAX_RX_LEN);
    
    while (HAL_GetTick() - startTick < rx_timeout)
    {
        if (RFID_ReadByte(&byte)) // 尝试从环形缓冲区读取一个字节
        {
            // 帧同步：查找帧头 0xAA
            if (!frame_started)
            {
                if (byte == RFID_FRAME_HEADER)
                {
                    frame_started = 1;
                    s_frame_rx_buffer[current_len++] = byte;
                }
                // 否则丢弃非帧头数据
            }
            else // 已找到帧头
            {
                if (current_len < RFID_MAX_RX_LEN)
                {
                    s_frame_rx_buffer[current_len++] = byte;
                }
                
                // 检查是否接收到预期长度
                if (current_len == rx_expected_len)
                {
                    // 检查帧尾
                    if (s_frame_rx_buffer[current_len - 1] == RFID_FRAME_END)
                    {
                        return 0x00; // 接收成功
                    }
                    else
                    {
                        // 帧尾不匹配，这是一个错误的帧，重置等待下一帧
                        current_len = 0;
                        frame_started = 0;
                        // 继续循环等待，看后续字节是否是新的帧头
                    }
                }
                // 检查是否超出临时缓冲区大小（防止溢出）
                else if (current_len >= RFID_MAX_RX_LEN)
                {
                    // 超出缓冲区，视为错误帧，重置等待
                    current_len = 0;
                    frame_started = 0;
                }
            }
        }
        else
        {
            // 缓冲区为空，等待1ms
            HAL_Delay(1);
        }
    }
    
    // 超时
    return 0xFF;
}

/**
 * @brief 发送指令并等待预期长度的响应（使用环形缓冲区非阻塞读取）。
 * @param tx_buffer 发送缓冲区
 * @param tx_len 发送长度
 * @param rx_expected_len 预期接收长度
 * @param rx_timeout 接收超时时间
 * @return 0x00: 接收成功, 0xFF: 超时/失败。
 */
static uint8_t send_and_receive_nonblocking(uint8_t* tx_buffer, uint8_t tx_len, uint8_t rx_expected_len, uint32_t rx_timeout)
{
    //清空环形缓冲区，确保每次操作干净
    flush_rfid_rx_buffer();
    
    //发送指令
    if (HAL_UART_Transmit(&huart1, tx_buffer, tx_len, 100) != HAL_OK)
    {
        return 0xFF; // 发送失败
    }

    //从环形缓冲区非阻塞等待接收预期长度数据
    return read_frame_from_buffer(rx_expected_len, rx_timeout);
}

uint8_t RFID_ReadPower(void)
{
    uint8_t result = 0xFF;
    const uint8_t RX_EXPECTED_LEN = 6;
    // F103 -> RFID: AA 02 01 55 
    uint8_t tx_buffer[] = {RFID_FRAME_HEADER, 0x02, RFID_CMD_READ_POWER, RFID_FRAME_END};
    
    // 调用发送接收函数 (非阻塞读取)
    if (send_and_receive_nonblocking(tx_buffer, sizeof(tx_buffer), RX_EXPECTED_LEN, RFID_ACK_TIMEOUT) != 0x00)
    {
        result = 0xFF; // 超时或接收失败
    }
    // 校验接收到的数据 预期: AA 04 01 00 [Power] 55
    else if (s_frame_rx_buffer[0] == RFID_FRAME_HEADER && 
        s_frame_rx_buffer[1] == 0x04 && 
        s_frame_rx_buffer[2] == RFID_CMD_READ_POWER &&
        s_frame_rx_buffer[3] == 0x00 && 
        s_frame_rx_buffer[5] == RFID_FRAME_END) 
    {
        // 校验成功，返回功率值 (位于索引 4)
        result = s_frame_rx_buffer[4]; 
    }
    else
    {
        result = 0xFE; // 校验失败
    }
    
    // 强制清除环形缓冲区
    flush_rfid_rx_buffer(); 
    return result;
}


uint8_t RFID_SetPower(uint8_t power)
{
    uint8_t result = 0xFF;
    const uint8_t RX_EXPECTED_LEN = 5;
    // F103 -> RFID: AA 04 02 01 [功率] 55 
    uint8_t tx_buffer[] = {RFID_FRAME_HEADER, 0x04, RFID_CMD_SET_POWER, 0x01, power, RFID_FRAME_END};
    
    if (send_and_receive_nonblocking(tx_buffer, sizeof(tx_buffer), RX_EXPECTED_LEN, RFID_ACK_TIMEOUT) != 0x00)
    {
        result = 0xFF; // 超时或接收失败
    }
    // 校验接收到的数据 预期: AA 03 02 00 55
    else if (s_frame_rx_buffer[0] == RFID_FRAME_HEADER && 
        s_frame_rx_buffer[1] == 0x03 && 
        s_frame_rx_buffer[2] == RFID_CMD_SET_POWER &&
        s_frame_rx_buffer[3] == 0x00 && 
        s_frame_rx_buffer[4] == RFID_FRAME_END) 
    {
        result = 0x01; // 设置成功标志
    }
    else
    {
        result = 0xFE; // 校验失败
    }
    // 强制清除环形缓冲区
    flush_rfid_rx_buffer(); 
    return result;
}

// 校验并提取EPC数据 (从 s_frame_rx_buffer 读取)
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
 * @brief 从环形缓冲区中读取一帧完整的标签数据（非阻塞）。
 * @param timeout_ms 接收超时时间 (短时间，100ms)
 * @retval 0x00: 接收成功, 0x01: 超时/无数据, 0xFF: 接收失败。
 */
static uint8_t read_tag_frame_nonblocking(uint32_t timeout_ms)
{
    const uint8_t TAG_RX_LEN = 23;
    uint32_t startTick = HAL_GetTick();
    uint8_t byte;
    uint8_t current_len = 0;
    
    // 清空临时缓冲区
    memset(s_frame_rx_buffer, 0, RFID_MAX_RX_LEN);
    
    while (HAL_GetTick() - startTick < timeout_ms)
    {
        if (RFID_ReadByte(&byte)) // 尝试从环形缓冲区读取一个字节
        {
            // 帧同步：查找帧头 0xAA
            if (current_len == 0)
            {
                if (byte == RFID_FRAME_HEADER)
                {
                    s_frame_rx_buffer[current_len++] = byte;
                }
                // 否则丢弃
            }
            else
            {
                if (current_len < TAG_RX_LEN)
                {
                    s_frame_rx_buffer[current_len++] = byte;
                }
                
                // 检查是否接收到预期长度
                if (current_len == TAG_RX_LEN)
                {
                    // 检查帧尾
                    if (s_frame_rx_buffer[TAG_RX_LEN - 1] == RFID_FRAME_END)
                    {
                        return 0x00; // 接收成功
                    }
                    else
                    {
                        //帧尾不匹配，这是一个错误的帧，需要通过查找下一个帧头 0xAA 来重新同步
                        //重置长度，并将当前字节（如果它是 0xAA）作为新帧头
                        if (byte == RFID_FRAME_HEADER)
                        {
                            s_frame_rx_buffer[0] = byte;
                            current_len = 1;
                        } else {
                            current_len = 0;
                        }
                    }
                }
            }
        }
        else
        {
            // 缓冲区为空，等待1ms
            HAL_Delay(1);
        }
    }
    
    // 超时或未读完
    return 0x01;
}

//循环读取RFID的数据标签
uint8_t RFID_CyclicRead(uint8_t read_count, uint8_t threshold_count, uint8_t result_epc_data[EPC_DATA_LEN])
{
    EPC_Data_t stats[MAX_UNIQUE_TAGS];
    uint8_t unique_tag_count = 0;
    uint16_t successful_reads = 0; 
    uint8_t epc_buffer[EPC_DATA_LEN];
    uint8_t i;
    uint8_t result_status = 0xFF;
    
    //初始化统计数组和结果
    memset(stats, 0, sizeof(stats));
    memset(result_epc_data, 0, EPC_DATA_LEN);
    
    // 清空缓冲区
    flush_rfid_rx_buffer();

    //发送启动循环读取指令: AA 02 10 55
    uint8_t start_tx[] = {RFID_FRAME_HEADER, 0x02, RFID_CMD_START_CYCLE, RFID_FRAME_END};
    
    // 预期确认帧: AA 03 10 01 55 (5 bytes)
    const uint8_t CONFIRM_RX_LEN = 5;
    if (send_and_receive_nonblocking(start_tx, sizeof(start_tx), CONFIRM_RX_LEN, RFID_ACK_TIMEOUT) != 0x00)
    {
        // 启动失败或确认帧超时
        result_status = 0xFF; 
        goto cleanup;
    }
    // 确认确认帧格式: AA 03 10 01 55
    if (!(s_frame_rx_buffer[0] == RFID_FRAME_HEADER &&
          s_frame_rx_buffer[2] == RFID_CMD_START_CYCLE &&
          s_frame_rx_buffer[3] == 0x01 && // 状态码 01 (确认)
          s_frame_rx_buffer[4] == RFID_FRAME_END))
    {
        // 确认帧校验失败
        result_status = 0xFF;
        goto cleanup;
    }

    //循环读取标签数据 (流式接收)
    while (successful_reads < read_count)
    {
        //从环形缓冲区中尝试读取下一帧标签数据
        uint8_t wait_status = read_tag_frame_nonblocking(TAG_FRAME_TIMEOUT_MS);
        
        if (wait_status == 0x00) // 接收成功
        {
            //校验并提取EPC(从 s_frame_rx_buffer 读取)
            if (parse_and_extract_epc(s_frame_rx_buffer, 23, epc_buffer) == 0x00)
            {
                successful_reads++;

                //统计 EPC 出现频率 (保持不变)
                uint8_t found = 0;
                for (i = 0; i < unique_tag_count; i++)
                {
                    if (memcmp(stats[i].epc_data, epc_buffer, EPC_DATA_LEN) == 0)
                    {
                        if (stats[i].count < 255) { stats[i].count++; } 
                        found = 1;
                        break;
                    }
                }
                if (!found && unique_tag_count < MAX_UNIQUE_TAGS)
                {
                    memcpy(stats[unique_tag_count].epc_data, epc_buffer, EPC_DATA_LEN);
                    stats[unique_tag_count].count = 1;
                    stats[unique_tag_count].is_valid = 1;
                    unique_tag_count++;
                }
            }
        }
        else if (wait_status == 0x01) // 超时/无数据
        {
            // 在指定时间内未收到完整标签帧，停止读取
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

  //发送停止循环读取指令: AA 02 12 55
  stop_and_exit: ;  //goto 用法，当流程中途失败时，跳到 stop_and_exit，确保发送停止指令
    uint8_t stop_tx[] = {RFID_FRAME_HEADER, 0x02, RFID_CMD_STOP_CYCLE, RFID_FRAME_END};
    
    // 预期确认帧: AA 03 12 00 55 (5 bytes)
    const uint8_t STOP_CONFIRM_RX_LEN = 5;
    
    // 尝试发送停止指令并等待确认 (非阻塞读取)
    if (send_and_receive_nonblocking(stop_tx, sizeof(stop_tx), STOP_CONFIRM_RX_LEN, RFID_ACK_TIMEOUT) == 0x00)
    {
        // 确认停止帧格式: AA 03 12 00 55
        if (!(s_frame_rx_buffer[0] == RFID_FRAME_HEADER &&
              s_frame_rx_buffer[2] == RFID_CMD_STOP_CYCLE &&
              s_frame_rx_buffer[3] == 0x00 && // 状态码 00 (成功)
              s_frame_rx_buffer[4] == RFID_FRAME_END))
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
    
     //查找满足阈值条件的 EPC
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
        result_status = 0x01;
    } else {
        result_status = 0xFF;
    }

    cleanup:  //无论成功失败，都跳到 cleanup 清空缓冲区，统一退出位置
    // 强制清除环形缓冲区
    flush_rfid_rx_buffer(); 
    return result_status;
}

uint8_t RFID_TransparentTransmit(uint8_t *tx_data, uint8_t tx_len, uint8_t *rx_buffer, uint8_t rx_buffer_max_len, uint8_t *actual_rx_len)
{
    uint32_t startTick = HAL_GetTick();
    uint32_t last_byte_tick = HAL_GetTick(); // 记录上一次收到字节的时间
    uint8_t byte;
    uint16_t current_rx_len = 0;
    
    // 清空环形缓冲区，确保干净
    flush_rfid_rx_buffer();
    *actual_rx_len = 0;

    // 阻塞发送上位机传入的原始数据给 RFID
    if (HAL_UART_Transmit(&huart1, tx_data, tx_len, 100) != HAL_OK)
    {
        return 0xFF; // 发送失败
    }

    // 等待 RFID 响应并转发
    while (HAL_GetTick() - startTick < RFID_TRANSPARENT_RX_TIMEOUT)
    {
        if (RFID_ReadByte(&byte)) // 尝试从环形缓冲区读取一个字节
        {
            // 更新最后接收时间
            last_byte_tick = HAL_GetTick(); 
            
            if (current_rx_len < rx_buffer_max_len)
            {
                rx_buffer[current_rx_len++] = byte;
            }
            else
            {
                // 接收缓冲区溢出，直接退出
                *actual_rx_len = (uint8_t)current_rx_len;
                return 0x01; 
            }
        }
        else // 环形缓冲区为空
        {
            // 检查是否已经收到数据，并且是否超过了静默时间
            if (current_rx_len > 0)
            {
                if (HAL_GetTick() - last_byte_tick >= RFID_TRANSPARENT_IDLE_TIME)
                {
                    // 超过静默时间，认为数据流结束，提前退出
                    *actual_rx_len = (uint8_t)current_rx_len;
                    return 0x01; 
                }
            }
            
            // 没有收到数据，也没有超过静默时间，小等待
            HAL_Delay(1);
        }
    }
    
    // 总超时退出
    *actual_rx_len = (uint8_t)current_rx_len;
    
    // 如果总超时前收到了数据，也视为成功返回
    if (current_rx_len > 0)
    {
        return 0x01;
    }
    
    return 0xFF; // 总超时且未收到任何数据
}
