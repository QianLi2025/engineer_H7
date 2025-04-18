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


//#include "arm_math.h"
extern uint8_t rx_data2[8];
extern uint16_t rec_id2;
#define PI               3.14159265358979f
void ROBOT_CMD_INIT(void);

void ROBOT_CMD_TASK(void);

#endif // CMD_H
