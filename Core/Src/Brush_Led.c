#include "Brush_Led.h"

//#define BRUSH1_PIN     GPIO_PIN_4    //毛刷检测
//#define BRUSH1_PORT    GPIOA

#define SW_RFID_U_Pin      GPIO_PIN_3  // 探针上限位
#define SW_RFID_U_GPIO_Port GPIOB

#define SW_RFID_D_Pin      GPIO_PIN_4  // 探针下限位
#define SW_RFID_D_GPIO_Port GPIOB


//uint8_t Brush1_GetStatus(void)
//{
//    // 读取 PA4 引脚状态
//    GPIO_PinState state = HAL_GPIO_ReadPin(BRUSH_GPIO_Port, BRUSH_Pin);

//    // 根据逻辑返回：高电平为未脱落，低电平为脱落
//    if (state == GPIO_PIN_SET)
//    {
//        return 1; // 未脱落 
//    }
//    else
//    {
//        return 0; // 已脱落 
//    }
//}

//未触发限位返回1，触发是0
uint8_t Brush2_GetStatus(void)
{
    GPIO_PinState state = HAL_GPIO_ReadPin(SW_RFID_U_GPIO_Port, SW_RFID_U_Pin);

    if (state == GPIO_PIN_SET)
        return 1;  //
    else
        return 0;  //触发上限位
}
uint8_t Brush3_GetStatus(void)
{
    GPIO_PinState state = HAL_GPIO_ReadPin(SW_RFID_D_GPIO_Port, SW_RFID_D_Pin);

    if (state == GPIO_PIN_SET)
        return 1;  
    else
        return 0;  //触发下限位
}

//void CheckBrush1(void)
//{
//    if (Brush1_GetStatus())
//    {
//        printf("1\r\n");
//    }
//    else
//    {
//        printf("0\r\n");

//    }
//}

//void CheckBrush2(void)
//{
//    if (Brush2_GetStatus())
//    {
//        printf("1\r\n");
//    }
//    else
//    {
//        printf("0\r\n");

//    }
//}

//void CheckBrush3(void)
//{
//    if (Brush3_GetStatus())
//    {
//        printf("1\r\n");
//    }
//    else
//    {
//        printf("0\r\n");

//    }
//}
