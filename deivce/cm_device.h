#ifndef CM_DEVICE_H
#define CM_DEVICE_H

#include "bsp_dwt.h"

#ifndef MAX_DEVICE_NUM
#define MAX_DEVICE_NUM (100)
#endif

typedef enum
{
    DEVICE_OK = 1,
    DEVICE_NOT_CONNECTED = 2
} DEVICE_STATE;

typedef struct
{
    uint32_t device_count; //����dwt����ʱ��
    double no_data_time;
    double time_out;
    uint16_t device_index; //�豸״̬�б��ϵ��1~MAX_DEVICE_NUM
    DEVICE_STATE state;
} CM_t; // connection monitor

extern DEVICE_STATE device_state_list[MAX_DEVICE_NUM];

extern CM_t video_cm;//ͼ������


void device_init(CM_t *cm, double time_out, uint16_t device_index);
void device_fbk(CM_t *cm);
void device_refresh(CM_t *cm);

#endif
