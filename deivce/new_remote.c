#include "new_remote.h"


DecodedRemote_t NEW_Remote0;

#define NEW_OFFSET 1024

void decode_remote_data(uint8_t *data, DecodedRemote_t *remote)
{
    remote->frame_header1 = data[0];
    remote->frame_header2 = data[1];
	
	  if(remote->frame_header1==0xa9&&remote->frame_header2==0x53){
		
		if (Verify_CRC16_Check_Sum(data, 21))
  {

    uint64_t temp_channel = 0;

// 从 data[2] 到 data[7] 共6字节拼成一个 48位数据
   for (int i = 0; i < 6; i++) {
    temp_channel |= ((uint64_t)data[2 + i]) << (8 * i);
   }

// 提取每个 11位的 channel 数据
    remote->right_level    = (temp_channel >> 0)  & 0x7FF;
    remote->right_vertical = (temp_channel >> 11) & 0x7FF;
    remote->left_vertical    = (temp_channel >> 22) & 0x7FF;
    remote->left_level      = (temp_channel >> 33) & 0x7FF;
	 
	  remote->right_level    -=NEW_OFFSET;
    remote->right_vertical -=NEW_OFFSET;
    remote->left_level     -=NEW_OFFSET;
    remote->left_vertical  -=NEW_OFFSET;

    // trigger + dail + boom
    uint16_t trigger_dail = (data[7] >> 4) | (data[8] << 4);
    remote->dangwei = (trigger_dail >> 0) & 0x03;
    remote->halt = (trigger_dail >> 2) & 0x01;
    remote->custom_left = (trigger_dail >> 3) & 0x01;
    remote->custom_right = (trigger_dail >> 4) & 0x01;
	 
	 
	 
    remote->dail = (uint16_t)( (data[8]>>1)|(data[9]<<7));
    remote->boom = (remote->dail >> 11) & 0x01;
		remote->dail = remote->dail&0x7FF;
		remote->dail -=NEW_OFFSET;

    // mouse movement
    remote->mouse_x = (int16_t)(data[9] | (data[10] << 8));
    remote->mouse_y = (int16_t)(data[11] | (data[12] << 8));
    remote->mouse_z = (int16_t)(data[13] | (data[14] << 8));

    // mouse buttons
    uint8_t mouse_btn = data[15];
    remote->mouse_left   = (mouse_btn >> 0) & 0x03;
    remote->mouse_right  = (mouse_btn >> 2) & 0x03;
    remote->mouse_middle = (mouse_btn >> 4) & 0x03;

    // keys
    remote->key_code = (uint16_t)(data[16] | (data[17] << 8));
    remote->check_sum = (uint16_t)(data[18] | (data[19] << 8));
	 }
	}
}