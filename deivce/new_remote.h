#pragma once

#include "struct_typedef.h"
#include "main.h"

#define NEW_LENGTH 21
typedef __packed struct
{
	
	
	uint8_t frame_header1;
	uint8_t frame_header2;
	
	 __packed struct
	{
		int16_t right_level:11;
		int16_t rigth_vertical:11;
		int16_t left_level:11;
		int16_t left_vertical:11;
	
	}channel;
	
	__packed struct
	{
		uint8_t dangwei:2;
		uint8_t halt:1;
		uint8_t custom_left:1;
		uint8_t custom_right:1;

	}trigger;
	
	int16_t dail:11;
	
	uint8_t boom:1;
	
	__packed struct
	{
	int16_t mouse_x;
	int16_t mouse_y;
	int16_t mouse_z;
		uint8_t left:2;
		uint8_t right:2;
		uint8_t middle:2;
		
	}mouse;
	
	uint16_t key_code;

	uint16_t check_sum;
	
}NEW_remote;






