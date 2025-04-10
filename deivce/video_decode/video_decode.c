#include "video_decode.h"
//ͼ����·���շ��� �����Զ��������


Video_ctrl_t video_ctrl[2]; // ���ڴ洢ͼ����·�Ŀ�������,[0]:��ǰ����TEMP,[1]:��һ�ε�����LAST.���ڰ����������º��л����ж�
//ͼ������

static void VideoRead(uint8_t *buff);

/**
 * @brief ͼ�����ݽ�������
 *
 * @param buff ͼ������
 */
static void VideoRead(uint8_t *buff)
{
    uint16_t judge_length; // ͳ��һ֡���ݳ���
    if (buff == NULL)      // �����ݰ��������κδ���
        return;
    // д��֡ͷ����(5-byte),�����ж��Ƿ�ʼ�洢��������
    memcpy(&video_ctrl[TEMP].FrameHeader, buff, LEN_HEADER);
    // �ж�֡ͷ����(0)�Ƿ�Ϊ0xA5
    if (buff[SOF] == REFEREE_SOF) {
        // ֡ͷCRC8У��
        if (Verify_CRC8_Check_Sum(buff, LEN_HEADER) == TRUE) {
            // ͳ��һ֡���ݳ���(byte),����CR16У��
            judge_length = buff[DATA_LENGTH] + LEN_HEADER + LEN_CMDID + LEN_TAIL;
            // ֡βCRC16У��
            if (Verify_CRC16_Check_Sum(buff, judge_length) == TRUE) {
                // 2��8λƴ��16λint
                video_ctrl[TEMP].CmdID = (buff[6] << 8 | buff[5]);
                // ��������������,�����ݿ�������Ӧ�ṹ����(ע�⿽�����ݵĳ���)
                // ��8���ֽڿ�ʼ�������� data=7
                switch (video_ctrl[TEMP].CmdID) 
                {
                    case ID_custom_robot_data: // �Զ�������
                        video_ctrl[TEMP].custom_control_mode = buff[DATA_Offset];
                        memcpy(&video_ctrl[TEMP].custom_data, (buff + DATA_Offset), LEN_custom_robot_data);
                        memcpy(&video_ctrl[TEMP].cus, &video_ctrl[TEMP].custom_data, 18);

                        break;
                    case ID_remote_control_data: // ͼ����·��������
                        // memcpy(&video_ctrl[TEMP].key_data, (buff + DATA_Offset), LEN_remote_control_data);
                        // *(uint16_t *)&video_ctrl[TEMP].key[KEY_PRESS] = video_ctrl[TEMP].key_data.keyboard_value;
                        // VideoDataContorl();
                        break;
                    default:
                        break;
                }
            }
        }
    }
}




