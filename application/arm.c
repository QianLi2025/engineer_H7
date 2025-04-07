#include "arm.h"
//速度只调PID 位置只调PD

//加限位 各个关节的 抬升机构的

//末三轴 fdcan3
//max min lift trans fdcan2


//过去
//arm板can1 max min lift can2 末三轴
//chassis板can2 轮电机 can1 传送带
extern ARM_CMD_data_t ARM_CMD_data;

//是否使能
int enable_roll=0;
int enable_lift=0;

//大臂不同关节PID
pid_type_def max_arm_pid;
pid_type_def mini_arm_pid;
pid_type_def finesse_pid;
pid_type_def pitch_arm_pid;
pid_type_def roll_pid_angle,roll_pid_speed;
pid_type_def lift_pid_angle,lift_pid_speed;

pid_type_def trans_pid_speed;//传送带速度

pid_type_def pid_lift_height;//目标高度

double max_ref,min_ref,finesse_ref,pitch_ref,roll_ref;

dmmotor_t max_motor,min_motor,finesse_motor,pitch_motor;

M2006motor_t roll,trans;//roll和传送带


M3508motor_t lift_motor;//抬升用的3508

//roll的速度环输出
double roll_speed_output;
//roll的角度环输出
double roll_angle_output;

//lift的速度环输出
double lift_speed_output;
//lift的角度环输出
double lift_angle_output;
//lift的高度环输出
double lift_height_output;
double lift_speed_ref;


//传送带速度输出
double trans_speed_output;

//roll的目标角度
double target_roll=0;

uint8_t roll_send[8]={};

//lift的目标高度
double target_lift=0;
//lift发送数组
uint8_t lift_send[8]={0};

//目标传送带速度
double trans_speed_ref=700;

double lift_height_ref=100;


// static int8_t is_init, lift_init_flag = 0;

//roll和lift的初始角度
double roll_init_angle,lift_motor_init_angle;

//抬升机构初始高度
double lift_init_height;

//机械臂高度
double scara_height,height;//需要好好确定一下

//roll实际值
double roll_real;

//高度目标值
double height_ref=200;

static void splitAndStoreSignals(int16_t signal1, int16_t signal2, int16_t signal3, int16_t signal4, uint8_t *outputArray);


double msp(double x, double in_min, double in_max, double out_min, double out_max)//映射函数，将编码器的值（0~8191）转换为弧度制的角度值（-pi~pi）
{
	return (x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
}


// 记录最近一次接收到 ID=0x03 或 0x04 的时间（单位ms）
static float last_motor_feedback_time_ms = 0;


extern uint8_t rx_data2[8];
extern uint16_t rec_id2;

extern uint8_t rx_data3[8];
extern uint16_t rec_id3;

static void enable_allmotor(void);//使能所有电机
static void recieve_all_data(void);//接收并解码
static void control_all_arm(double max_ref,double min_ref,double finesse_ref,double pitch_ref);//控制所有电机角度 有待全改成弧度制

static void lift_height_control(double lift_height_ref);//设置上升高度

static void trans_speed_control(double trans_speed_ref);//设置传送带速度

//计算实际高度
static void Height_Calculation(void);

//控制实际高度
static void lift_height_control(double lift_height_ref);

//计算roll的实际角度
static void Roll_Calculation(void);

//记录初始角度
static void roll_height_init(void);

//从cmd获取所有的电机控制信息
static void get_arm_cmd(void);

static void lift_speed_control(double lift_speed_ref);
static void control_roll_angle(double roll_ref);

static int last_lift_mode=-1;
static int last_roll_mode=-1;

static float temp_roll;  // 保留变量值在每次进入此模式期间
static float temp_height;  // 保留变量值在每次进入此模式期间
void offline_check(void);
void ARM_INIT(void)
{

    M2006_init(&roll,  1,  0.18 * 1 / 36);
	M2006_init(&trans,  2,  0.18 * 1 / 36);
	
	M3508_init(&lift_motor,1,0.3 * 187 / 3591);
    PID_init(&roll_pid_angle,PID_POSITION,0,0,1,10000,150);//6
	PID_init(&roll_pid_speed,PID_POSITION,0,0,0,10000,200);

	PID_init(&lift_pid_angle,PID_POSITION,5,0,0,10000,15000);
	
    PID_init(&trans_pid_speed,PID_POSITION,5,0,0,10000,1500);
	
    PID_init(&lift_pid_speed,PID_POSITION,3,20,0,10000,1700);
    PID_init(&pid_lift_height,PID_POSITION,5,0.01, 0, 45000, 100);//不要积分也许也可以？

    enable_allmotor();

    // roll_height_init();
	

}

//can3直接插
//can2要反向
//抬升机构一共三种模式 速度 保持 位置
//roll一共两种角度 保持
void ARM_TASK(void)
{
    recieve_all_data();//收取所有电机信息	 

    //所有大电机控制
    control_all_arm( ARM_CMD_data.maximal_arm_angle, 
        ARM_CMD_data.minimal_arm_angle, 
        ARM_CMD_data.finesse_angle, 
        ARM_CMD_data.pitch_arm_angle);//所有关节角度
    
    //传送带控制
    switch (ARM_CMD_data.trans_mode)
    {
    case GET://取
        trans_speed_control(0);//控制传送带速度
        break;
    case OUTPUT://兑矿
        trans_speed_control(0);//控制传送带速度
        break;

    default:
        break;
    }

    //高度控制
    Height_Calculation();//计算当前高度
    switch (ARM_CMD_data.lift_mode)
    {
    case LIFT_KEEP_MODE://停止
			
        if (last_lift_mode != LIFT_KEEP_MODE) // 如果是第一次进入该模式
        {
            temp_height = height; // 仅第一次更新
        }
        lift_height_control(temp_height);
        break;
        
    case LIFT_SPEED_MODE://增量式到达某处
        lift_speed_control(ARM_CMD_data.lift_speed);
        break;
		
    case LIFT_HEIGHT_MODE://达到指定高度    
        lift_height_control(ARM_CMD_data.lift_height);
		    break;

    default:
        break;
    }
    last_lift_mode = ARM_CMD_data.lift_mode;


	
	
    //提前计算高度方便对高度闭环 可用可不用
    //keep模式直接闭环 speed模式增量式抬升

    //需要做限幅 在发送消息时

    //roll轴控制
    Roll_Calculation();
    switch (ARM_CMD_data.roll_mode)
    {
    case ROLL_KEEP_MODE://停止

        if (last_lift_mode != LIFT_KEEP_MODE) // 如果是第一次进入该模式
        {
            temp_roll = roll_real; // 这里的计算还需要好好改
        }
        control_roll_angle(temp_roll);
        break;

    case ROLL_ANGLE_MODE://增量式到达某处
        control_roll_angle(ARM_CMD_data.roll_angle);
        break;

    default:
        break;
    }
    last_roll_mode = ARM_CMD_data.roll_mode;
    


}





static void splitAndStoreSignals(int16_t signal1, int16_t signal2, int16_t signal3, int16_t signal4, uint8_t *outputArray) {
    // 拆分每个信号的高八位和低八位
    outputArray[0] = (signal1 >> 8) & 0xFF;  // signal1的高八位
    outputArray[1] = signal1 & 0xFF;        // signal1的低八位
    outputArray[2] = (signal2 >> 8) & 0xFF;  // signal2的高八位
    outputArray[3] = signal2 & 0xFF;        // signal2的低八位
    outputArray[4] = (signal3 >> 8) & 0xFF;  // signal3的高八位
    outputArray[5] = signal3 & 0xFF;        // signal3的低八位
    outputArray[6] = (signal4 >> 8) & 0xFF;  // signal4的高八位
    outputArray[7] = signal4 & 0xFF;        // signal4的低八位
}



static void enable_allmotor(void)
{
	  
    enable_motor_mode( &hfdcan2, 1, MIT_MODE);//此处是txid
	 
    enable_motor_mode( &hfdcan2, 2, POS_MODE);
	 
    enable_motor_mode( &hfdcan3, 1, POS_MODE );
  
    enable_motor_mode( &hfdcan3, 2, POS_MODE);//除了最末端的2006


} 


//挂起所有电机失能标志位
 void set_all_start_flag(void)
{
    max_motor.start_flag=1;
    min_motor.start_flag=1;
    finesse_motor.start_flag=1;
    pitch_motor.start_flag=1;

}

//检查是否失能 在5HZ任务中进行下去
void offline_check(void)
{
    if(max_motor.start_flag==1)
    enable_motor_mode( &hfdcan2, 1, MIT_MODE);//此处是txid
    if(min_motor.start_flag==1)
    enable_motor_mode( &hfdcan2, 2, POS_MODE);
    if(finesse_motor.start_flag==1)
    enable_motor_mode( &hfdcan3, 1, POS_MODE );
    if(pitch_motor.start_flag==1)
    enable_motor_mode( &hfdcan3, 2, POS_MODE);//除了最末端的2006

}

static void recieve_all_data(void)
{
    //接收电机数据
    if(rec_id2==0x03){
    max_motor.start_flag=0;
    dm_fdkdata(&max_motor,rx_data2);
    }
    if(rec_id2==0x04){
    min_motor.start_flag=0;
    dm_fdkdata(&min_motor,rx_data2);//这些事判断是否会使能的
    }

    if(rec_id2==0x201)
    M3508_fbkdata(&lift_motor,rx_data2);
    if(rec_id2==0x202)
    M2006_fbkdata(&trans,rx_data2);

    if(rec_id3==0x03){
    finesse_motor.start_flag=0;
    dm_fdkdata(&finesse_motor,rx_data3);
    }
    if(rec_id3==0x04){
    pitch_motor.start_flag=0;
    dm_fdkdata(&pitch_motor,rx_data3);
    }
    if(rec_id3==0x201)
    M2006_fbkdata(&roll,rx_data3);
}

static void get_arm_cmd(void)
{
    max_ref=ARM_CMD_data.maximal_arm_angle;
    min_ref=ARM_CMD_data.minimal_arm_angle;
    finesse_ref=ARM_CMD_data.finesse_angle;
    pitch_ref=ARM_CMD_data.pitch_arm_angle;
}

//控制roll以外所有电机（z轴）
static void control_all_arm(double max_ref,double min_ref,double finesse_ref,double pitch_ref)
{
    	
	mit_ctrl(&hfdcan2,  1,  max_ref-0.2, 1 ,  30,  5,  0);//12 2.8
    pos_speed_ctrl(&hfdcan2,  2,  min_ref, 4);//从min_arm开始
    pos_speed_ctrl(&hfdcan3,  1,  finesse_ref+0.1, 1.5);//finesse
    pos_speed_ctrl(&hfdcan3,  2,  pitch_ref, 3.5);//pitch
	 


}

static void control_roll_angle(double roll_ref)//这里是把目标值变换了 由于计算的是总圈数
{
    //roll的双环pid
	roll_angle_output=PID_calc(&roll_pid_angle,  roll_real,  roll_ref*ROLL_OFFSET); //这里改了很多 要注意
    roll_speed_output=PID_calc(&roll_pid_speed,roll.para.vel_fbk,roll_angle_output);
    splitAndStoreSignals(roll_speed_output,0,0,0,roll_send); 
    fdcanx_send_data(&hfdcan3,0x200,roll_send,8);//发送roll控制帧

}






//需要把其ID改一改 改成2
static void trans_speed_control(double trans_speed_ref)
{
    // 计算传送带速度的PID输出
    trans_speed_output = PID_calc(&trans_pid_speed, trans.para.vel_fbk, trans_speed_ref);   

}

static void Height_Calculation(void)
{
    height       = -(lift_motor.total_angle - 0)  ;//height是初始化后的
}



static void lift_height_control(double lift_height_ref)
{
    // 计算升降角度的PID输出
    double lift_height_output = -PID_calc(&pid_lift_height, height, lift_height_ref);   
    // 计算升降速度的PID输出
    double lift_speed_output = PID_calc(&lift_pid_speed, lift_motor.para.vel_fbk, lift_height_output);  
    
    // 分离并存储信号
    splitAndStoreSignals(lift_speed_output, trans_speed_output, 0, 0, lift_send);//顺便把传送带的也给发了
    fdcanx_send_data(&hfdcan2,0x200,lift_send,8);//发送roll控制帧
}

static void lift_speed_control(double lift_speed_ref)
{
   
    // 计算升降速度的PID输出
    double lift_speed_output = PID_calc(&lift_pid_speed, lift_motor.para.vel_fbk, lift_height_output);  
    
    // 分离并存储信号
    splitAndStoreSignals(lift_speed_output, trans_speed_output, 0, 0, lift_send);//顺便把传送带的也给发了
    fdcanx_send_data(&hfdcan2,0x200,lift_send,8);//发送roll控制帧
}


static void Roll_Calculation(void)
{
    roll_real = (roll.total_angle - 0) ;
}

static void roll_height_init(void)
{
    roll_init_angle       = roll.total_angle; // min = -3460 - 165 max =4973 - 165
    lift_motor_init_angle = lift_motor.total_angle;//没必要
}


//重置高度和roll
void reset_roll_and_height(void)
{
    roll.not_first=0;
    lift_motor.not_first=0;

}


