#ifndef RC_PROTOCAL_H
#define RC_PROTOCAL_H
#include "struct_typedef.h"
#include "main.h"
#include "referee.h"
#include "robot_def.h"
extern referee_t video_cmd;

#define REMOTE_BUFF_SIZE 18

typedef __packed struct
{
    __packed struct
    {
        int16_t ch[5];
        char s[2];
			  int16_t dial;
    } rc;
    __packed struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;
    __packed struct
    {
        uint16_t v;
    } key;

} RC_ctrl_t;

void Get_RC_ctrl_t(volatile uint8_t *rxBuf);

extern RC_ctrl_t rc_ctrl;
extern uint16_t w_flag;
extern uint16_t s_flag;
extern uint16_t a_flag;
extern uint16_t d_flag;
extern uint16_t q_flag;
extern uint16_t e_flag;
extern uint16_t shift_flag;
extern uint16_t ctrl_flag;
extern uint8_t press_left;
extern uint8_t press_right;
extern uint16_t r_flag;
extern uint16_t f_flag;
extern uint16_t g_flag;
extern uint16_t z_flag;
extern uint16_t x_flag;
extern uint16_t c_flag;
extern uint16_t v_flag;
extern uint16_t b_flag;





// 全局变量定义
extern KeyComboCounter_t v_counter ;

extern KeyComboCounter_t b_counter;
extern KeyComboCounter_t g_counter ;

extern KeyComboCounter_t x_counter ;

extern KeyComboCounter_t z_counter;                    

extern KeyComboCounter_t c_counter ; 
extern KeyComboCounter_t r_counter ;
extern KeyComboCounter_t f_counter ;
#endif
