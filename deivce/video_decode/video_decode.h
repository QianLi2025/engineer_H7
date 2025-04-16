#ifndef VIDEO_H
#define VIDEO_H


#include "usart.h"

#include "robot_def.h"
#include "bsp_dwt.h"
#include "RC_protocol.h"

#include "string.h"
#include "CRC8_CRC16.h"


// ����ң�������ݶ�ȡ,ң����������һ����СΪ2������
#define LAST 1
#define TEMP 0



//extern uint8_t uart7_rx_buff[RE_RX_BUFFER_SIZE];
void VIDEO_INIT(void);

void VIDEO_TASK(void);



#define REFEREE_SOF 0xA5 // ��ʼ�ֽ�,Э��̶�Ϊ0xA5
#define Robot_Red 0
#define Robot_Blue 1
#define Communicate_Data_LEN 5 // �Զ��彻�����ݳ��ȣ��ó��Ⱦ������ҷ����ͺ��������գ��Զ��彻������Э�����ʱֻ��Ҫ���Ĵ˺궨�弴��

/****************************ͨ��Э���ʽ****************************/

/* ͨ��Э���ʽƫ�ƣ�ö������,����#define���� */
typedef enum
{
	FRAME_HEADER_Offset = 0,
	CMD_ID_Offset = 5,
	DATA_Offset = 7,
} JudgeFrameOffset_e;

/* ͨ��Э�鳤�� */
typedef enum
{
	LEN_HEADER = 5, // ֡ͷ��
	LEN_CMDID = 2,	// �����볤��
	LEN_TAIL = 2,	// ֡βCRC16

	LEN_CRC8 = 4, // ֡ͷCRC8У�鳤��=֡ͷ+���ݳ�+�����
} JudgeFrameLength_e;

/****************************֡ͷ****************************/
/****************************֡ͷ****************************/

/* ֡ͷƫ�� */
typedef enum
{
	SOF = 0,		 // ��ʼλ
	DATA_LENGTH = 1, // ֡�����ݳ���,�����������ȡ���ݳ���
	SEQ = 3,		 // �����
	CRC8 = 4		 // CRC8
} FrameHeaderOffset_e;

/* ֡ͷ���� */
typedef struct
{
	uint8_t SOF;
	uint16_t DataLength;
	uint8_t Seq;
	uint8_t CRC8;
} xFrameHeader;


typedef struct
{
    float maximal_arm_target; // ��۵�Ŀ��ֵ
    float minimal_arm_target; // С�۵�Ŀ��ֵ
    float finesse_target;     // �����Ŀ��ֵ
    float pitch_arm_target;   // pitch��Ŀ��ֵ
    //float roll_arm_target;    // roll��Ŀ��ֵ
    // float maximal_arm_target; // ��۵�Ŀ��ֵ
    // float minimal_arm_target; // С�۵�Ŀ��ֵ
    // float finesse_target;     // �����Ŀ��ֵ
    // float pitch_arm_target;   // pitch��Ŀ��ֵ
    // float roll_arm_target;    // roll��Ŀ��ֵ
    //float height;           // z���Ŀ��ֵ
} Custom_contorl_t;

typedef struct
{
    float delta_x;
    float delta_y;
    float delta_z;
    float delta_yaw;
    float delta_pitch;
    float delta_roll;
} slightly_controll_data;

/* ID: 0x0302   Byte: 30    �Զ��������������˽������� */
//typedef struct
//{
//	uint8_t data[30];
//} custom_robot_data_t;

///* ID: 0x0304   Byte: 12    ͼ����·����ң������ */
//typedef struct
//{
//	int16_t mouse_x;
//	int16_t mouse_y;
//	int16_t mouse_z;
//	uint8_t left_button_down;
//	uint8_t right_button_down;
//	uint16_t keyboard_value;
//	uint16_t reserved;
//} remote_control_t;

typedef struct
{
    xFrameHeader FrameHeader;        // ���յ���֡ͷ��Ϣ
    uint16_t CmdID;                  // ������
    custom_robot_data_t custom_data; // �Զ�������
    uint8_t custom_control_mode;     // �Զ������ģʽ
    Custom_contorl_t cus;            // �������Զ�������
    slightly_controll_data scd;      // ��΢��������
    remote_control_t key_data;       // ң��������

//    Key_t key[3]; // ��Ϊλ���ļ�������,�ռ����8��,�ٶ�����16~��

    uint8_t key_count[3][16];
} Video_ctrl_t;


/* ������ID,�����жϽ��յ���ʲô���� */
typedef enum
{
	ID_game_state = 0x0001,				   // ����״̬����
	ID_game_result = 0x0002,			   // �����������
	ID_game_robot_survivors = 0x0003,	   // ����������Ѫ������
	ID_event_data = 0x0101,				   // �����¼�����
	ID_supply_projectile_action = 0x0102,  // ���ز���վ������ʶ����
	ID_supply_projectile_booking = 0x0103, // ���ز���վԤԼ�ӵ�����
	ID_game_warning = 0x0104,			   // ���о�����Ϣ
	ID_dart_info = 0x0105,				   // ���ڷ�������
	ID_game_robot_state = 0x0201,		   // ������״̬����
	ID_power_heat_data = 0x0202,		   // ʵʱ������������
	ID_game_robot_pos = 0x0203,			   // ������λ������
	ID_buff_musk = 0x0204,				   // ��������������
	ID_aerial_robot_energy = 0x0205,	   // ���л���������״̬����
	ID_robot_hurt = 0x0206,				   // �˺�״̬����
	ID_shoot_data = 0x0207,				   // ʵʱ�������
	ID_projectile_allowance = 0x0208,	   // ������������
	ID_rfid_status = 0x0209,			   // RFID״̬����
	// ��demo��Ĭ��Ϊ���������ˣ����δ�����������ݵĽ���������Ҫ����
	// ���������
	ID_dart_client_cmd = 0x020A,	   // ����ѡ�ֶ�ָ������
	ID_ground_robot_position = 0x020B, // ���������λ������
	ID_radar_mark_data = 0x020C,	   // �״��ǽ�������
	ID_sentry_info = 0x020D,		   // �ڱ�����������Ϣͬ��
	ID_radar_info = 0x020E,			   // �״�����������Ϣͬ��
	ID_student_interactive = 0x0301,   // �����˼佻������
	// ����Ϊͼ����·���ݽ���
	ID_custom_robot_data = 0x0302,	 // �Զ������������
	ID_remote_control_data = 0x0304, // ͼ����·��������
} CmdID_e;


/* ���������ݶγ�,���ݹٷ�Э�������峤�ȣ������Զ������ݳ��� */
typedef enum
{
	LEN_game_state = 11,			  // 0x0001
	LEN_game_result = 1,			  // 0x0002
	LEN_game_robot_HP = 32,			  // 0x0003
	LEN_event_data = 4,				  // 0x0101
	LEN_supply_projectile_action = 4, // 0x0102
	LEN_game_warning = 3,			  // 0x0104
	LEN_dart_info = 3,				  // 0x0105
	LEN_game_robot_state = 13,		  // 0x0201
	LEN_power_heat_data = 16,		  // 0x0202
	LEN_game_robot_pos = 16,		  // 0x0203
	LEN_buff_musk = 6,				  // 0x0204
	LEN_aerial_robot_energy = 2,	  // 0x0205
	LEN_robot_hurt = 1,				  // 0x0206
	LEN_shoot_data = 7,				  // 0x0207
	LEN_projectile_allowance = 6,	  // 0x0208
	LEN_rfid_status = 4,			  // 0x0209
	// ��demo��Ĭ��Ϊ���������ˣ����δ�����������ݵĽ���������Ҫ����
	// ���������
	LEN_dart_client_cmd = 6,					 // 0x020A
	LEN_ground_robot_position = 40,				 // 0x020B
	LEN_radar_mark_data = 6,					 // 0x020C
	LEN_sentry_info = 4,						 // 0x020D
	LEN_radar_info = 1,							 // 0x020E
	LEN_receive_data = 6 + Communicate_Data_LEN, // 0x0301
	// ����Ϊͼ����·���ݽ���
	LEN_custom_robot_data = 30,	  // 0x0302
	LEN_remote_control_data = 12, // 0x0304

} JudgeDataLength_e;

#endif // ARM_H
