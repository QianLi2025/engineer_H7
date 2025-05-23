#pragma once

#include "struct_typedef.h"
#include "main.h"
#include "CRC8_CRC16.h"

#define NEW_LENGTH 21

typedef struct {
    uint8_t frame_header1;
    uint8_t frame_header2;
    int16_t right_level;
    int16_t right_vertical;
    int16_t left_level;
    int16_t left_vertical;
    uint8_t dangwei;
    uint8_t halt;
    uint8_t custom_left;
    uint8_t custom_right;
    int16_t dail;
    uint8_t boom;
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_left;
    uint8_t mouse_right;
    uint8_t mouse_middle;
    uint16_t key_code;
    uint16_t check_sum;
} DecodedRemote_t;

extern DecodedRemote_t NEW_Remote0;


void decode_remote_data(uint8_t *data, DecodedRemote_t *remote);

