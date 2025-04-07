#include "referee.h"

#define RED_HERO 1
#define RED_ENGINEER 2
#define RED_INFANTRY

__attribute__((at(0x2400CCC6 + 2600))) uint8_t uart1_dma_tx_buf[200] = {0};
static interaction_figure_t figs[100] = {0}; //静态存储100个图形
static uint8_t figs_state[100] = {0};        // 1表示占用，0表示未被占用

void rf_ui_upgrade(referee_t *rf)
{
    rf->ui.ui_send.ui_header.SOF = 0xa5;
    rf->ui.ui_send.ui_header.data_length = rf->ui.ui_send.ui_data_len;
    Append_CRC8_Check_Sum(&(rf->ui.ui_send.ui_header), rf->ui.ui_send.ui_data_len);
    memcpy(&uart1_dma_tx_buf[0], &rf->ui.ui_send.ui_header, sizeof(frame_header_t));
    rf->ui.ui_send.cmd_id = 0x301;
    memcpy(&uart1_dma_tx_buf[sizeof(frame_header_t)], rf->ui.ui_send.cmd_id, sizeof(uint16_t));
    memcpy(&uart1_dma_tx_buf[sizeof(frame_header_t) + sizeof(uint16_t)], rf->ui.ui_send.ui_data_buf, rf->ui.ui_send.ui_data_len);
    Append_CRC16_Check_Sum(uart1_dma_tx_buf, sizeof(frame_header_t) + sizeof(uint16_t) + rf->ui.ui_send.ui_data_len + sizeof(uint16_t)); // header+cmd_id+data[]+crc16
    HAL_UART_Transmit_DMA(&huart3, uart1_dma_tx_buf, sizeof(frame_header_t) + sizeof(uint16_t) + rf->ui.ui_send.ui_data_len + sizeof(uint16_t));
}
// figs_num0~99 color0~9 0红/蓝（己方颜色） 1：黄色 2：绿色3：橙色 4：紫红色 5：粉色 6：青色7：黑色8：白色
void rf_ui_write_string(referee_t *rf, char string_[], uint16_t len, uint16_t size, uint8_t color, int x, int y, int figs_num, ROBOT_ID id)
{
    if (figs_state[figs_num] == 0) //创建
    {
        figs_state[figs_num] = 1;
        figs[figs_num].figure_name[0] = figs_num; //名称默认figs_num
        figs[figs_num].figure_name[1] = figs_num; //名称默认figs_num
        figs[figs_num].figure_name[2] = figs_num; //名称默认figs_num
        figs[figs_num].operate_tpye = 1;          //增加图形
        figs[figs_num].figure_tpye = 7;           //字符图像
        figs[figs_num].layer = 0;                 //字符默认0层
        figs[figs_num].details_a = size;          //字体大小
        figs[figs_num].details_b = len;           //字符串长度
        figs[figs_num].width = size;              //建议10:1的字体大小和线宽比
        figs[figs_num].start_x = x;
        figs[figs_num].start_y = y;
    }
    if (figs_state[figs_num] == 1) //修改
    {
        figs_state[figs_num] = 2;
        figs[figs_num].operate_tpye = 1; //增加图形
        figs[figs_num].figure_tpye = 7;  //字符图像
        figs[figs_num].layer = 0;        //字符默认0层
        figs[figs_num].details_a = size; //字体大小
        figs[figs_num].details_b = len;  //字符串长度
        figs[figs_num].width = size;     //建议10:1的字体大小和线宽比
        figs[figs_num].start_x = x;
        figs[figs_num].start_y = y;
    }
    rf->ui.ui_send.data_cmd_id = 0x110;
    if (id == RED_1)
    {
        rf->ui.ui_send.sender_id = 1;
        rf->ui.ui_send.sender_id = 0x101;
    }

    rf->ui.ui_send.ui_data_len = 45;
    memcpy(rf->ui.ui_send.ui_data_buf, &figs[figs_num], sizeof(interaction_figure_t));
    memcpy(&rf->ui.ui_send.ui_data_buf[15], string_, len);
}

void referee_fbkdata(referee_t *rf, uint8_t buf[])
{
    int data_len = 0;
    int data_pack_len = 0;
    uint16_t cmd_id = 0;
    if (Verify_CRC8_Check_Sum(buf, sizeof(frame_header_t)))
    {
        memcpy((void *)&(rf->frame_header), (const void *)buf, sizeof(frame_header_t));
        if (rf->frame_header.SOF == 0xa5)
        {
            data_len = rf->frame_header.data_length;
            data_pack_len = 5 + 2 + data_len + 2;
            if (Verify_CRC16_Check_Sum(buf, data_pack_len))
            {
                memcpy((void *)&(cmd_id), (const void *)&buf[sizeof(frame_header_t)], sizeof(uint16_t));
                switch (cmd_id)
                {
                case 0x001:
                    memcpy((void *)&(rf->game_status), (const void *)&buf[sizeof(frame_header_t) + sizeof(uint16_t)], sizeof(rf->game_status));
                    break;
                case 0x002:
                    memcpy((void *)&(rf->game_result), (const void *)&buf[sizeof(frame_header_t) + sizeof(uint16_t)], sizeof(rf->game_result));
                    break;
                case 0x003:
                    memcpy((void *)&(rf->game_robot_HP), (const void *)&buf[sizeof(frame_header_t) + sizeof(uint16_t)], sizeof(rf->game_robot_HP));
                    break;
                case 0x101:
                    memcpy((void *)&(rf->event_data), (const void *)&buf[sizeof(frame_header_t) + sizeof(uint16_t)], sizeof(rf->event_data));
                    break;
                case 0x104:
                    memcpy((void *)&(rf->referee_warning), (const void *)&buf[sizeof(frame_header_t) + sizeof(uint16_t)], sizeof(rf->referee_warning));
                    break;
                case 0x105:
                    memcpy((void *)&(rf->dart_info), (const void *)&buf[sizeof(frame_header_t) + sizeof(uint16_t)], sizeof(rf->dart_info));
                    break;
                case 0x201:
                    memcpy((void *)&(rf->robot_status), (const void *)&buf[sizeof(frame_header_t) + sizeof(uint16_t)], sizeof(rf->robot_status));
                    break;
                case 0x202:
                    memcpy((void *)&(rf->power_heat_data), (const void *)&buf[sizeof(frame_header_t) + sizeof(uint16_t)], sizeof(rf->power_heat_data));
                    break;
                case 0x203:
                    memcpy((void *)&(rf->robot_pos), (const void *)&buf[sizeof(frame_header_t) + sizeof(uint16_t)], sizeof(rf->robot_pos));
                    break;
                case 0x204:
                    memcpy((void *)&(rf->buff), (const void *)&buf[sizeof(frame_header_t) + sizeof(uint16_t)], sizeof(rf->buff));
                    break;
                case 0x206:
                    memcpy((void *)&(rf->hurt_data), (const void *)&buf[sizeof(frame_header_t) + sizeof(uint16_t)], sizeof(rf->hurt_data));
                    break;
                case 0x207:
                    memcpy((void *)&(rf->shoot_data), (const void *)&buf[sizeof(frame_header_t) + sizeof(uint16_t)], sizeof(rf->shoot_data));
                    break;
                case 0x208:
                    memcpy((void *)&(rf->projectile_allowance), (const void *)&buf[sizeof(frame_header_t) + sizeof(uint16_t)], sizeof(rf->projectile_allowance));
                    break;
                case 0x209:
                    memcpy((void *)&(rf->rfid_status), (const void *)&buf[sizeof(frame_header_t) + sizeof(uint16_t)], sizeof(rf->rfid_status));
                    break;
                case 0x20a:
                    memcpy((void *)&(rf->dart_client_cmd), (const void *)&buf[sizeof(frame_header_t) + sizeof(uint16_t)], sizeof(rf->dart_client_cmd));
                    break;
                case 0x20b:
                    memcpy((void *)&(rf->ground_robot_position), (const void *)&buf[sizeof(frame_header_t) + sizeof(uint16_t)], sizeof(rf->ground_robot_position));
                    break;
                case 0x20c:
                    memcpy((void *)&(rf->radar_mark_data), (const void *)&buf[sizeof(frame_header_t) + sizeof(uint16_t)], sizeof(rf->radar_mark_data));
                    break;
                case 0x20d:
                    memcpy((void *)&(rf->sentry_info), (const void *)&buf[sizeof(frame_header_t) + sizeof(uint16_t)], sizeof(rf->sentry_info));
                    break;
                case 0x20e:
                    memcpy((void *)&(rf->radar_info), (const void *)&buf[sizeof(frame_header_t) + sizeof(uint16_t)], sizeof(rf->radar_info));
                    break;
                case 0x301:
                    memcpy((void *)&(rf->robot_interaction_data), (const void *)&buf[sizeof(frame_header_t) + sizeof(uint16_t)], sizeof(rf->robot_interaction_data));
                    break;
                case 0x302:
                    memcpy((void *)&(rf->custom_robot_data), (const void *)&buf[sizeof(frame_header_t) + sizeof(uint16_t)], sizeof(rf->custom_robot_data));
                    break;
                case 0x303:
                    memcpy((void *)&(rf->map_command), (const void *)&buf[sizeof(frame_header_t) + sizeof(uint16_t)], sizeof(rf->map_command));
                    break;
                case 0x304:
                    memcpy((void *)&(rf->remote_control), (const void *)&buf[sizeof(frame_header_t) + sizeof(uint16_t)], sizeof(rf->remote_control));
                    break;
                case 0x305:
                    memcpy((void *)&(rf->map_robot_data), (const void *)&buf[sizeof(frame_header_t) + sizeof(uint16_t)], sizeof(rf->map_robot_data));
                    break;
                case 0x306:
                    memcpy((void *)&(rf->custom_client_data), (const void *)&buf[sizeof(frame_header_t) + sizeof(uint16_t)], sizeof(rf->custom_client_data));
                    break;
                case 0x307:
                    memcpy((void *)&(rf->map_data), (const void *)&buf[sizeof(frame_header_t) + sizeof(uint16_t)], sizeof(rf->map_data));
                    break;
                case 0x308:
                    memcpy((void *)&(rf->custom_info), (const void *)&buf[sizeof(frame_header_t) + sizeof(uint16_t)], sizeof(rf->custom_info));
                    break;
                case 0x309:
                    memcpy((void *)&(rf->robot_custom_data), (const void *)&buf[sizeof(frame_header_t) + sizeof(uint16_t)], sizeof(rf->robot_custom_data));
                    break;
                default:
                    break;
                }
            }
        }
    }
}


