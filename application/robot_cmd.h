#ifndef CMD_H
#define CMD_H
#include "robot_def.h"
#include "main.h"
#include "usart.h"
#include "RC_protocol.h"
#include "pid.h"
#include "kf_imu.h"
#include "usart.h"
#include "minipc_protocol.h"
#include "arm.h"
#include "scara_kinematics.h"
#include "chassis.h"
#include "bsp_fdcan.h"
#include "referee.h"
#include "cm_device.h"
#include "usbd_cdc_if.h"
#include "UI_task.h"
#include "video.h"
#include "new_remote.h"
 typedef struct {
	 float total_distance_x;
	 float total_distance_y;
	 uint8_t start_count_flag;//开启计数标志
	 
	 
	 
 
}distance_t;
 
 extern auto_mode_e AUTO_STATE;


 
 
// uint16_t 类型的按键标志


// uint8_t 类型的鼠标按键标志

//#include "arm_math.h"
extern uint8_t rx_data2[8];
extern uint16_t rec_id2;
#define PI               3.14159265358979f
extern ARM_CMD_data_t ARM_CMD_data;
void ROBOT_CMD_INIT(void);

void ROBOT_CMD_TASK(void);

void minipc_send(minipc_t* pc);
extern float rc_mode_xy_after_check[2];

#endif // CMD_H
