#include "video.h"
//图传链路接收分析 包含自定义控制器


//图传接收
uint8_t uart7_rx_buff[40];

void Verify_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);

void VIDEO_INIT(void)
{
    HAL_UARTEx_ReceiveToIdle_IT(&huart7,uart7_rx_buff, sizeof(uart7_rx_buff));
}

void VIDEO_TASK(void)//控制舵机
{
    

}

