#ifndef VIDEO_H
#define VIDEO_H


#include "usart.h"

#include "robot_def.h"
#include "bsp_dwt.h"
#include "RC_protocol.h"

// 用于遥控器数据读取,遥控器数据是一个大小为2的数组
#define LAST 1
#define TEMP 0

#define RE_RX_BUFFER_SIZE 40u 

extern uint8_t uart7_rx_buff[RE_RX_BUFFER_SIZE];
void VIDEO_INIT(void);

#endif // ARM_H
