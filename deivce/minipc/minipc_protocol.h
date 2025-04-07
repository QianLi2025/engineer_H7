#ifndef MINIPC_PROTOCOL_H
#define MINIPC_PROTOCOL_H

#include "stdint.h"
#include "string.h"
#include "CRC8_CRC16.h"
#include "bsp_dwt.h"

typedef struct
{
    uint8_t mcu2minipc_buf[100];
    float no_data_time;
    float pack_loss_rate;
    __packed struct
    {
        uint8_t header; // = 0x5A;
        uint8_t is_rune;
        uint8_t detect_color; // 5-red
        uint8_t is_reset;
        float roll;  // r:
        float yaw;   // rad
        float pitch; //
        float v0;
        float motor_speed;
        uint16_t checksum; // = 0; // c!
    } mcu2minipc;
    __packed struct
    {
        uint8_t header; // = 0xA5;
        uint8_t is_tracking;
        uint8_t is_can_hit;
        float yaw;
        float pitch;
        float distance;
        uint16_t checksum; // = 0;
    } minipc2mcu;
    uint32_t minipc_count;      //用于记时
    uint8_t rx_pack_state[100]; //记录当前及100个数据包中的接收状态，0什么都不表示，1表示接收正常，2表示丢包
} minipc_t;

void minipc_rec(minipc_t *pc, uint8_t pc_data_buf[]);
void minipc_upgrade(minipc_t *pc);

#endif // MINIPC_PROTOCOL_H
