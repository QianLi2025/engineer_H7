#ifndef ARM_H
#define ARM_H

#include "pid.h"
#include "fdcan.h"
#include "DamiaoMotors.h"
#include "bsp_fdcan.h"
#include "M2006motors.h"
#include "M3508motors.h"
#include "robot_def.h"
#include "bsp_dwt.h"
#include "RC_protocol.h"
void ARM_INIT(void);

void ARM_TASK(void);

//离线检测
void offline_check(void);

//挂起所有标志位
void set_all_start_flag(void);

#define ROLL_OFFSET 35.07f//变速比

#endif // ARM_H

