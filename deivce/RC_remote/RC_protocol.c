#include "RC_protocol.h"

uint16_t w_flag;
uint16_t s_flag;
uint16_t a_flag;
uint16_t d_flag;
uint16_t q_flag;
uint16_t e_flag;
uint16_t shift_flag;
uint16_t ctrl_flag;
uint8_t press_left;
uint8_t press_right;
uint16_t r_flag;
uint16_t f_flag;
uint16_t g_flag;
uint16_t z_flag;
uint16_t x_flag;
uint16_t c_flag;
uint16_t v_flag;
uint16_t b_flag;
RC_ctrl_t rc_ctrl;


//自己定义键盘
uint16_t v_press_count = 0;
uint16_t ctrl_v_press_count = 0;
uint16_t shift_v_press_count = 0;

uint8_t v_flag_last;
uint8_t ctrl_flag_last;
uint8_t shift_flag_last;

KeyCode_BUFF_t key_code_buff;//


// 全局变量定义
KeyComboCounter_t v_counter = {0,0,0, key_v, key_shift, key_ctrl,0,0,0};                      // 单独v键

KeyComboCounter_t b_counter = {0,0,0, key_b, key_shift, key_ctrl,0,0,0};                      // 单独b键

KeyComboCounter_t g_counter = {0,0,0, key_g, key_shift, key_ctrl,0,0,0};                      // 单独v键

KeyComboCounter_t x_counter = {0,0,0, key_x, key_shift, key_ctrl,0,0,0};                      // 单独b键


KeyComboCounter_t z_counter = {0,0,0, key_z, key_shift, key_ctrl,0,0,0};                      // 单独b键

KeyComboCounter_t c_counter = {0,0,0, key_c, key_shift, key_ctrl,0,0,0};                      // 单独b键

KeyComboCounter_t r_counter = {0,0,0, key_r, key_shift, key_ctrl,0,0,0};                      // 单独b键

KeyComboCounter_t f_counter = {0,0,0, key_f, key_shift, key_ctrl,0,0,0};                      // 单独b键

//是否被按下
uint8_t is_key_pressed(uint16_t now, uint16_t last, KeyBit_t bit) {
    return ((now >> bit) & 0x01) && !((last >> bit) & 0x01);
}

//是否保持

static uint8_t is_key_held(uint16_t now, KeyBit_t bit) {
	
    return (now >> bit) & 0x01;
}

static void update_combo_counter(KeyCode_BUFF_t *status, KeyComboCounter_t *counter) {
    if (
        is_key_pressed(status->key_code, status->key_code_last, counter->primary) &&
        !is_key_held(status->key_code, counter->shift_key) &&
        !is_key_held(status->key_code, counter->ctrl_key)
    ) {
        counter->single_press_count++;
			  counter->single_press_flag=1;
    }
		else{
			counter->single_press_flag=0;//没有被单独按下
		}
		
		if (
        is_key_pressed(status->key_code, status->key_code_last, counter->primary) &&
        is_key_held(status->key_code, counter->shift_key) &&
        !is_key_held(status->key_code, counter->ctrl_key)
    ) {
        counter->shift_press_count++;
		  	counter->shift_press_flag=1;
    }
		else{
			counter->shift_press_flag=0;
		}
		
		if (
        is_key_pressed(status->key_code, status->key_code_last, counter->primary) &&
        !is_key_held(status->key_code, counter->shift_key) &&
        is_key_held(status->key_code, counter->ctrl_key)
    ) {
        counter->ctrl_press_count++;
			counter->ctrl_press_flag=1;
    }
		else
		{
			counter->ctrl_press_flag=0;
		}

		
}

#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
void Get_RC_ctrl_t(volatile uint8_t *rxBuf)
{
    rc_ctrl.rc.ch[0] = (rxBuf[0] | (rxBuf[1] << 8)) & 0x07ff;                 //!< Channel 0  中值为1024，最大值1684，最小值364，波动范围：660
    rc_ctrl.rc.ch[1] = (((rxBuf[1] >> 3) & 0xff) | (rxBuf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl.rc.ch[2] = (((rxBuf[2] >> 6) & 0xff) | (rxBuf[3] << 2) |          //!< Channel 2
                        (rxBuf[4] << 10)) &
                       0x07ff;
    rc_ctrl.rc.ch[3] = (((rxBuf[4] >> 1) & 0xff) | (rxBuf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl.rc.s[0] = ((rxBuf[5] >> 4) & 0x0003);                             //!< Switch left！！！这尼玛是右
    rc_ctrl.rc.s[1] = ((rxBuf[5] >> 4) & 0x000C) >> 2;                        //!< Switch right！！！这才是左
    rc_ctrl.mouse.x = rxBuf[6] | (rxBuf[7] << 8);                             //!< Mouse X axis
    rc_ctrl.mouse.y = rxBuf[8] | (rxBuf[9] << 8);                             //!< Mouse Y axis
    rc_ctrl.mouse.z = rxBuf[10] | (rxBuf[11] << 8);                           //!< Mouse Z axis
    rc_ctrl.mouse.press_l = rxBuf[12];                                        //!< Mouse Left Is Press ?
    rc_ctrl.mouse.press_r = rxBuf[13];                                        //!< Mouse Right Is Press ?
    rc_ctrl.key.v = rxBuf[14] | (rxBuf[15] << 8);                             //!< KeyBoard value
    rc_ctrl.rc.ch[4] = rxBuf[16] | (rxBuf[17] << 8);                          // NULL
	  rc_ctrl.rc.dial= ((rxBuf[16] | (rxBuf[17] << 8)) & 0x07FF) - RC_CH_VALUE_OFFSET;     

	
	

    rc_ctrl.rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[4] -= RC_CH_VALUE_OFFSET;

    // Some flag of keyboard
    w_flag = (rxBuf[14] & 0x01);
    s_flag = (rxBuf[14] & 0x02);
    a_flag = (rxBuf[14] & 0x04);
    d_flag = (rxBuf[14] & 0x08);
    q_flag = (rxBuf[14] & 0x40);
    e_flag = (rxBuf[14] & 0x80);
    shift_flag = (rxBuf[14] & 0x10);
    ctrl_flag = (rxBuf[14] & 0x20);
		
    press_left = rc_ctrl.mouse.press_l;
    press_right = rc_ctrl.mouse.press_r;
		
		
    r_flag = rc_ctrl.key.v & (0x00 | 0x01 << 8);
    f_flag = rc_ctrl.key.v & (0x00 | 0x02 << 8);
    g_flag = rc_ctrl.key.v & (0x00 | 0x04 << 8);
    z_flag = rc_ctrl.key.v & (0x00 | 0x08 << 8);
    x_flag = rc_ctrl.key.v & (0x00 | 0x10 << 8);
    c_flag = rc_ctrl.key.v & (0x00 | 0x20 << 8);
    v_flag = rc_ctrl.key.v & (0x00 | 0x40 << 8);
    b_flag = rc_ctrl.key.v & (0x00 | 0x80 << 8);
		
		
    if (w_flag != 0)
        w_flag = 1;
    if (s_flag != 0)
        s_flag = 1;
    if (a_flag != 0)
        a_flag = 1;
    if (d_flag != 0)
        d_flag = 1;
    if (q_flag != 0)
        q_flag = 1;
    if (e_flag != 0)
        e_flag = 1;
    if (shift_flag != 0)
        shift_flag = 1;
    if (ctrl_flag != 0)
        ctrl_flag = 1;
    if (press_left != 0)
        press_left = 1;
    if (r_flag != 0)
        r_flag = 1;
    if (f_flag != 0)
        f_flag = 1;
    if (g_flag != 0)
        g_flag = 1;
    if (z_flag != 0)
        z_flag = 1;
    if (x_flag != 0)
        x_flag = 1;
    if (c_flag != 0)
        c_flag = 1;
    if (v_flag != 0)
        v_flag = 1;
    if (b_flag != 0)
        b_flag = 1;//这个才是滤除掩码的最终效果
		

		key_code_buff.key_code_last = key_code_buff.key_code;//更新上次的

		key_code_buff.key_code = rc_ctrl.key.v;//使用遥控器键鼠
		
	

    update_combo_counter(&key_code_buff, &v_counter);

    update_combo_counter(&key_code_buff, &b_counter);
		
		update_combo_counter(&key_code_buff, &g_counter);

    update_combo_counter(&key_code_buff, &x_counter);
		update_combo_counter(&key_code_buff, &c_counter);

    update_combo_counter(&key_code_buff, &z_counter);
 
		
		update_combo_counter(&key_code_buff, &r_counter);
		update_combo_counter(&key_code_buff, &f_counter);
		
		
}
//按键按下的时候是0