#include "robot_cmd.h"


int ref_test=0;
uint8_t uart5_rx_buff[18]={0};

extern RC_ctrl_t rc_ctrl;//遥控器数据
Chassis_CMD_data_t Chassis_CMD_data;
ARM_CMD_data_t ARM_CMD_data;

float RC_SPEED_RATIO=0.6;

void lift_cmd(ARM_CMD_data_t *arm_cmd, int mode,double ref);//上升模式控制

void ROBOT_CMD_INIT(void)
{
    HAL_UARTEx_ReceiveToIdle_IT(&huart5,uart5_rx_buff, sizeof(uart5_rx_buff));
}


void ROBOT_CMD_TASK(void)
{
    //接收遥控器数据
    #ifdef REMOTE_CONTROL
    Chassis_CMD_data.vx=rc_ctrl.rc.ch[0]*RC_SPEED_RATIO;
    Chassis_CMD_data.vy=rc_ctrl.rc.ch[1]*(RC_SPEED_RATIO);
    Chassis_CMD_data.vw=rc_ctrl.rc.ch[3]*(RC_SPEED_RATIO);
    #endif

     if(rc_ctrl.rc.s[0]==3)//走直线模式x
    {
        Chassis_CMD_data.Chassis_straight_mode=STRAIGHT_X_ON;
    }
    else if(rc_ctrl.rc.s[0]==2)//走直线模式y
    {
        Chassis_CMD_data.Chassis_straight_mode=STRAIGHT_Y_ON;
    }
    else
    {
        Chassis_CMD_data.Chassis_straight_mode=STRAIGHT_OFF;
    }
		

		
	if(rc_ctrl.rc.s[1]==3)//高度保持
	{
	    // ARM_CMD_data.lift_mode=LIFT_KEEP_MODE;
        lift_cmd(&ARM_CMD_data, LIFT_KEEP_MODE,0);
	}
	else if(rc_ctrl.rc.s[1]==2)//上
	{
		lift_cmd(&ARM_CMD_data, LIFT_SPEED_MODE,ref_test);
	}
	else
	{
		lift_cmd(&ARM_CMD_data, LIFT_SPEED_MODE,-ref_test);
	}

}


//上升模式控制
void lift_cmd(ARM_CMD_data_t *arm_cmd, int mode,double ref)
{

    switch (mode)
    {
    case LIFT_KEEP_MODE:
        arm_cmd->lift_mode=LIFT_KEEP_MODE;
        break;
    case LIFT_HEIGHT_MODE:
        arm_cmd->lift_mode=LIFT_HEIGHT_MODE;
        arm_cmd->lift_height=ref;
        break;
    case LIFT_SPEED_MODE:
        arm_cmd->lift_mode=LIFT_SPEED_MODE;
        arm_cmd->lift_speed=ref;
        
        break;
    
    default:
        break;
    }
	
}



/**
 * @brief 视觉控制
 *
 */
static void VisionContorl(void)
{
    
}


/**
 * @brief 图传链路以及自定义控制器的模式和控制量设置
 *
 */

static void VideoControlSet(void)
{
}



//模式预备 好几个if
//取金矿 推矿模式
//自动放东西 取东西

//自动取银矿 放银矿

//0为手动模式1为自动取东西模式2为自动放东西模式3为自动取银矿

