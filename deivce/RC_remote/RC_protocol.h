#ifndef RC_PROTOCAL_H
#define RC_PROTOCAL_H
#include "struct_typedef.h"
#include "main.h"

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


#define NONE     100
#define key_s     1
#define key_a     2
#define key_d     3
#define key_shift 4
#define key_ctrl  5
#define key_q     6
#define key_e     7
#define key_r     8
#define key_f     9
#define key_g     10
#define key_z     11
#define key_x     12
#define key_c     13
#define key_v     14
#define key_b     15

typedef enum {

    Key_S = 1,
    Key_A = 2,
    Key_D = 3,
    Key_Shift = 4,
    Key_Ctrl = 5,
    Key_Q = 6,
    Key_E = 7,
    Key_R = 8,
    Key_F = 9,
    Key_G = 10,
    Key_Z = 11,
    Key_X = 12,
    Key_C = 13,
    Key_V = 14,
    Key_B = 15
    
} KeyBit_t;



typedef struct {
    uint16_t key_code;       // 当前键码
    uint16_t key_code_last;  // 上一次键码
} KeyCode_BUFF_t;

typedef struct {
    uint16_t single_press_count;
	  uint16_t shift_press_count;
	  uint16_t ctrl_press_count;
	  
    KeyBit_t primary;//基础按键
    KeyBit_t shift_key;//修饰按键
    KeyBit_t ctrl_key;
	
	  uint8_t single_press_flag;
	  uint8_t shift_press_flag;
	  uint8_t ctrl_press_flag;
	
} KeyComboCounter_t;//修饰计数

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
