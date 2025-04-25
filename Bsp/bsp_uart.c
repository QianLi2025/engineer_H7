#include "bsp_uart.h"

// rx缓冲区长度
#ifndef UART_RX_BUF_LEN
#define UART_RX_BUF_LEN (150)
#endif
// tx缓冲区长度
#ifndef UART_TX_BUF_LEN
#define UART_TX_BUF_LEN (150)
#endif
// uart——tx发送队列最大值
#ifndef UART_TX_QUEUE_MAX
#define UART_TX_QUEUE_MAX (20)
#endif
//两包数据发送的最小时间间隔为0.0005s 防止接收方uart线路没有空闲段，该值一定不能大于refresh循环的频率，否则串口一直繁忙一直无法发送
#ifndef UART_TX_MIN_TIMEOUT
#define UART_TX_MIN_TIMEOUT (0.0005f)
#endif
// uart发送缓冲区
/*0x24002CC6*/
__attribute__((at(0x24002CC6 + UART_TX_BUF_LEN * 0))) uint8_t uart1_tx_buf[UART_TX_BUF_LEN] = {0};
__attribute__((at(0x24002CC6 + UART_TX_BUF_LEN * 1))) static uint8_t uart2_tx_buf[UART_TX_BUF_LEN] = {0};
__attribute__((at(0x24002CC6 + UART_TX_BUF_LEN * 2))) static uint8_t uart3_tx_buf[UART_TX_BUF_LEN] = {0};
__attribute__((at(0x24002CC6 + UART_TX_BUF_LEN * 3))) static uint8_t uart4_tx_buf[UART_TX_BUF_LEN] = {0};
__attribute__((at(0x24002CC6 + UART_TX_BUF_LEN * 4))) static uint8_t uart5_tx_buf[UART_TX_BUF_LEN] = {0};
__attribute__((at(0x24002CC6 + UART_TX_BUF_LEN * 5))) static uint8_t uart6_tx_buf[UART_TX_BUF_LEN] = {0};
__attribute__((at(0x24002CC6 + UART_TX_BUF_LEN * 6))) static uint8_t uart7_tx_buf[UART_TX_BUF_LEN] = {0};
__attribute__((at(0x24002CC6 + UART_TX_BUF_LEN * 7))) static uint8_t uart8_tx_buf[UART_TX_BUF_LEN] = {0};
__attribute__((at(0x24002CC6 + UART_TX_BUF_LEN * 8))) static uint8_t uart9_tx_buf[UART_TX_BUF_LEN] = {0};
__attribute__((at(0x24002CC6 + UART_TX_BUF_LEN * 9))) static uint8_t uart10_tx_buf[UART_TX_BUF_LEN] = {0};
// uart接收缓冲区
static uint8_t uart1_rx_buf[UART_RX_BUF_LEN] = {0};
static uint8_t uart2_rx_buf[UART_RX_BUF_LEN] = {0};
static uint8_t uart3_rx_buf[UART_RX_BUF_LEN] = {0};
static uint8_t uart4_rx_buf[UART_RX_BUF_LEN] = {0};
static uint8_t uart5_rx_buf[UART_RX_BUF_LEN] = {0};
static uint8_t uart6_rx_buf[UART_RX_BUF_LEN] = {0};
static uint8_t uart7_rx_buf[UART_RX_BUF_LEN] = {0};
static uint8_t uart8_rx_buf[UART_RX_BUF_LEN] = {0};
static uint8_t uart9_rx_buf[UART_RX_BUF_LEN] = {0};
static uint8_t uart10_rx_buf[UART_RX_BUF_LEN] = {0};
//发送管理结构体
typedef struct
{
    uint8_t uart_tx_packages[UART_TX_QUEUE_MAX][UART_TX_BUF_LEN];
    int uart_tx_packages_len[UART_TX_QUEUE_MAX];
    uint8_t package_index;
    UART_TX_STATE uart_tx_state;
    uint32_t uart_tx_count; //用于dwt记录时间
} uart_tx_manager_t;

uart_tx_manager_t uart1_tx_manager = {0};
uart_tx_manager_t uart2_tx_manager = {0};
uart_tx_manager_t uart3_tx_manager = {0};
uart_tx_manager_t uart5_tx_manager = {0};
uart_tx_manager_t uart7_tx_manager = {0};
uart_tx_manager_t uart10_tx_manager = {0};

//内部函数声明
void uart_send(UART_HandleTypeDef *huart, uint8_t data[], int len);

// bsp_uart初始化
void bsp_uart_init(void)
{
//    HAL_UARTEx_ReceiveToIdle_IT(&huart1, uart1_rx_buf, UART_RX_BUF_LEN);
//    HAL_UARTEx_ReceiveToIdle_IT(&huart2, uart2_rx_buf, UART_RX_BUF_LEN);
//    HAL_UARTEx_ReceiveToIdle_IT(&huart3, uart3_rx_buf, UART_RX_BUF_LEN);
//    HAL_UARTEx_ReceiveToIdle_IT(&huart5, uart5_rx_buf, UART_RX_BUF_LEN);
    HAL_UARTEx_ReceiveToIdle_IT(&huart7, uart7_rx_buf, UART_RX_BUF_LEN);
    HAL_UARTEx_ReceiveToIdle_IT(&huart10, uart10_rx_buf, UART_RX_BUF_LEN);
}
/*************************************************************************************************/
/****************************uart1*******************************/
//UART_TX_STATE uart1_add_package(uint8_t data[], int len)
//{
//    if (uart1_tx_manager.package_index >= UART_TX_QUEUE_MAX)
//        return TX_FULL;
//    else if (len > UART_TX_BUF_LEN)
//        return TX_LEN_ERROR;
//    else
//    {
//        memcpy(uart1_tx_manager.uart_tx_packages[uart1_tx_manager.package_index], data, len);
//        uart1_tx_manager.uart_tx_packages_len[uart1_tx_manager.package_index] = len;
//        uart1_tx_manager.package_index++;
//        return TX_OK;
//    }
//}
//UART_TX_STATE uart1_tx_refresh(void)
//{
//    if (uart1_tx_manager.uart_tx_state == TX_BUSY)
//        return TX_BUSY;
//    else
//    {
//        float dt = DWT_GetDeltaT(&uart1_tx_manager.uart_tx_count);
//        if (dt <= UART_TX_MIN_TIMEOUT)
//            return TX_BUSY;
//        else
//        {
//            if (uart1_tx_manager.package_index >= 1) //代表有数据
//            {
//                memcpy(uart1_tx_buf, uart1_tx_manager.uart_tx_packages[0], UART_TX_BUF_LEN);
//                uart_send(&huart1, uart1_tx_buf, uart1_tx_manager.uart_tx_packages_len[0]);
//                uart1_tx_manager.package_index--;
//                uart1_tx_manager.uart_tx_state = TX_BUSY;
//                for (int i = 0; i < UART_TX_QUEUE_MAX - 1; i++)
//                {
//                    memcpy(uart1_tx_manager.uart_tx_packages[i], uart1_tx_manager.uart_tx_packages[i + 1], UART_TX_BUF_LEN);
//                    uart1_tx_manager.uart_tx_packages_len[i] = uart1_tx_manager.uart_tx_packages_len[i + 1];
//                }
//                return TX_OK;
//            }
//            else
//                return TX_NO_DATA;
//        }
//    }
//}
///*************************************************************************************************/
///****************************uart2*******************************/
//UART_TX_STATE uart2_add_package(uint8_t data[], int len)
//{
//    if (uart2_tx_manager.package_index >= UART_TX_QUEUE_MAX)
//        return TX_FULL;
//    else if (len > UART_TX_BUF_LEN)
//        return TX_LEN_ERROR;
//    else
//    {
//        memcpy(uart2_tx_manager.uart_tx_packages[uart2_tx_manager.package_index], data, len);
//        uart2_tx_manager.uart_tx_packages_len[uart2_tx_manager.package_index] = len;
//        uart2_tx_manager.package_index++;
//        return TX_OK;
//    }
//}
//UART_TX_STATE uart2_tx_refresh(void)
//{
//    if (uart2_tx_manager.uart_tx_state == TX_BUSY)
//        return TX_BUSY;
//    else
//    {
//        float dt = DWT_GetDeltaT(&uart2_tx_manager.uart_tx_count);
//        if (dt <= UART_TX_MIN_TIMEOUT)
//            return TX_BUSY;
//        else
//        {
//            if (uart2_tx_manager.package_index >= 1) //代表有数据
//            {
//                memcpy(uart2_tx_buf, uart2_tx_manager.uart_tx_packages[0], UART_TX_BUF_LEN);
//                uart_send(&huart2, uart2_tx_buf, uart2_tx_manager.uart_tx_packages_len[0]);
//                uart2_tx_manager.package_index--;
//                uart2_tx_manager.uart_tx_state = TX_BUSY;
//                for (int i = 0; i < UART_TX_QUEUE_MAX - 1; i++)
//                {
//                    memcpy(uart2_tx_manager.uart_tx_packages[i], uart2_tx_manager.uart_tx_packages[i + 1], UART_TX_BUF_LEN);
//                    uart2_tx_manager.uart_tx_packages_len[i] = uart2_tx_manager.uart_tx_packages_len[i + 1];
//                }
//                return TX_OK;
//            }
//            else
//                return TX_NO_DATA;
//        }
//    }
//}
///*************************************************************************************************/
///****************************uart3*******************************/
//UART_TX_STATE uart3_add_package(uint8_t data[], int len)
//{
//    if (uart3_tx_manager.package_index >= UART_TX_QUEUE_MAX)
//        return TX_FULL;
//    else if (len > UART_TX_BUF_LEN)
//        return TX_LEN_ERROR;
//    else
//    {
//        memcpy(uart3_tx_manager.uart_tx_packages[uart3_tx_manager.package_index], data, len);
//        uart3_tx_manager.uart_tx_packages_len[uart3_tx_manager.package_index] = len;
//        uart3_tx_manager.package_index++;
//        return TX_OK;
//    }
//}
//UART_TX_STATE uart3_tx_refresh(void)
//{
//    if (uart3_tx_manager.uart_tx_state == TX_BUSY)
//        return TX_BUSY;
//    else
//    {
//        float dt = DWT_GetDeltaT(&uart3_tx_manager.uart_tx_count);
//        if (dt <= UART_TX_MIN_TIMEOUT)
//            return TX_BUSY;
//        else
//        {
//            if (uart3_tx_manager.package_index >= 1) //代表有数据
//            {
//                memcpy(uart3_tx_buf, uart3_tx_manager.uart_tx_packages[0], UART_TX_BUF_LEN);
//                uart_send(&huart3, uart3_tx_buf, uart3_tx_manager.uart_tx_packages_len[0]);
//                uart3_tx_manager.package_index--;
//                uart3_tx_manager.uart_tx_state = TX_BUSY;
//                for (int i = 0; i < UART_TX_QUEUE_MAX - 1; i++)
//                {
//                    memcpy(uart3_tx_manager.uart_tx_packages[i], uart3_tx_manager.uart_tx_packages[i + 1], UART_TX_BUF_LEN);
//                    uart3_tx_manager.uart_tx_packages_len[i] = uart3_tx_manager.uart_tx_packages_len[i + 1];
//                }
//                return TX_OK;
//            }
//            else
//                return TX_NO_DATA;
//        }
//    }
//}
///*************************************************************************************************/
///****************************uart5*******************************/
//UART_TX_STATE uart5_add_package(uint8_t data[], int len)
//{
//    if (uart5_tx_manager.package_index >= UART_TX_QUEUE_MAX)
//        return TX_FULL;
//    else if (len > UART_TX_BUF_LEN)
//        return TX_LEN_ERROR;
//    else
//    {
//        memcpy(uart5_tx_manager.uart_tx_packages[uart5_tx_manager.package_index], data, len);
//        uart5_tx_manager.uart_tx_packages_len[uart5_tx_manager.package_index] = len;
//        uart5_tx_manager.package_index++;
//        return TX_OK;
//    }
//}
//UART_TX_STATE uart5_tx_refresh(void)
//{
//    if (uart5_tx_manager.uart_tx_state == TX_BUSY)
//        return TX_BUSY;
//    else
//    {
//        float dt = DWT_GetDeltaT(&uart5_tx_manager.uart_tx_count);
//        if (dt <= UART_TX_MIN_TIMEOUT)
//            return TX_BUSY;
//        else
//        {
//            if (uart5_tx_manager.package_index >= 1) //代表有数据
//            {
//                memcpy(uart5_tx_buf, uart5_tx_manager.uart_tx_packages[0], UART_TX_BUF_LEN);
//                uart_send(&huart5, uart5_tx_buf, uart5_tx_manager.uart_tx_packages_len[0]);
//                uart5_tx_manager.package_index--;
//                uart5_tx_manager.uart_tx_state = TX_BUSY;
//                for (int i = 0; i < UART_TX_QUEUE_MAX - 1; i++)
//                {
//                    memcpy(uart5_tx_manager.uart_tx_packages[i], uart5_tx_manager.uart_tx_packages[i + 1], UART_TX_BUF_LEN);
//                    uart5_tx_manager.uart_tx_packages_len[i] = uart5_tx_manager.uart_tx_packages_len[i + 1];
//                }
//                return TX_OK;
//            }
//            else
//                return TX_NO_DATA;
//        }
//    }
//}
/*************************************************************************************************/
/****************************uart7*******************************/
UART_TX_STATE uart7_add_package(uint8_t data[], int len)
{
    if (uart7_tx_manager.package_index >= UART_TX_QUEUE_MAX)
        return TX_FULL;
    else if (len > UART_TX_BUF_LEN)
        return TX_LEN_ERROR;
    else
    {
        memcpy(uart7_tx_manager.uart_tx_packages[uart7_tx_manager.package_index], data, len);
        uart7_tx_manager.uart_tx_packages_len[uart7_tx_manager.package_index] = len;
        uart7_tx_manager.package_index++;
        return TX_OK;
    }
}
UART_TX_STATE uart7_tx_refresh(void)
{
    if (uart7_tx_manager.uart_tx_state == TX_BUSY)
        return TX_BUSY;
    else
    {
        float dt = DWT_GetDeltaT(&uart7_tx_manager.uart_tx_count);
        if (dt <= UART_TX_MIN_TIMEOUT)
            return TX_BUSY;
        else
        {
            if (uart7_tx_manager.package_index >= 1) //代表有数据
            {
                memcpy(uart7_tx_buf, uart7_tx_manager.uart_tx_packages[0], UART_TX_BUF_LEN);
                uart_send(&huart7, uart7_tx_buf, uart7_tx_manager.uart_tx_packages_len[0]);
                uart7_tx_manager.package_index--;
                uart7_tx_manager.uart_tx_state = TX_BUSY;
                for (int i = 0; i < UART_TX_QUEUE_MAX - 1; i++)
                {
                    memcpy(uart7_tx_manager.uart_tx_packages[i], uart7_tx_manager.uart_tx_packages[i + 1], UART_TX_BUF_LEN);
                    uart7_tx_manager.uart_tx_packages_len[i] = uart7_tx_manager.uart_tx_packages_len[i + 1];
                }
                return TX_OK;
            }
            else
                return TX_NO_DATA;
        }
    }
}
/*************************************************************************************************/
/****************************uart10*******************************/
UART_TX_STATE uart10_add_package(uint8_t data[], int len)
{
    if (uart10_tx_manager.package_index >= UART_TX_QUEUE_MAX)
        return TX_FULL;
    else if (len > UART_TX_BUF_LEN)
        return TX_LEN_ERROR;
    else
    {
        memcpy(uart10_tx_manager.uart_tx_packages[uart10_tx_manager.package_index], data, len);
        uart10_tx_manager.uart_tx_packages_len[uart10_tx_manager.package_index] = len;
        uart10_tx_manager.package_index++;
        return TX_OK;
    }
}
UART_TX_STATE uart10_tx_refresh(void)
{
    if (uart10_tx_manager.uart_tx_state == TX_BUSY)
        return TX_BUSY;
    else
    {
        float dt = DWT_GetDeltaT(&uart10_tx_manager.uart_tx_count);
        if (dt <= UART_TX_MIN_TIMEOUT)
            return TX_BUSY;
        else
        {
            if (uart10_tx_manager.package_index >= 1) //代表有数据
            {
                memcpy(uart10_tx_buf, uart10_tx_manager.uart_tx_packages[0], UART_TX_BUF_LEN);
                uart_send(&huart10, uart10_tx_buf, uart10_tx_manager.uart_tx_packages_len[0]);
                uart10_tx_manager.package_index--;
                uart10_tx_manager.uart_tx_state = TX_BUSY;
                for (int i = 0; i < UART_TX_QUEUE_MAX - 1; i++)
                {
                    memcpy(uart10_tx_manager.uart_tx_packages[i], uart10_tx_manager.uart_tx_packages[i + 1], UART_TX_BUF_LEN);
                    uart10_tx_manager.uart_tx_packages_len[i] = uart10_tx_manager.uart_tx_packages_len[i + 1];
                }
                return TX_OK;
            }
            else
                return TX_NO_DATA;
        }
    }
}
/*************************************************************************************************/
/************************************midlleware  方便切换发送模式**********************************/
void uart_send(UART_HandleTypeDef *huart, uint8_t data[], int len)
{
    HAL_UART_Transmit_DMA(huart, data, len);
}
//发送完成中断，清除busy标志位
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
//    if (huart == &huart1)
//    {
//        uart1_tx_manager.uart_tx_state = TX_OK;         //表示uart外设完成一包数据发送
//        DWT_GetDeltaT(&uart1_tx_manager.uart_tx_count); //标记时间
//    }
//    if (huart == &huart2)
//    {
//        uart2_tx_manager.uart_tx_state = TX_OK;         //表示uart外设完成一包数据发送
//        DWT_GetDeltaT(&uart2_tx_manager.uart_tx_count); //标记时间
//    }
//    if (huart == &huart3)
//    {
//        uart3_tx_manager.uart_tx_state = TX_OK;         //表示uart外设完成一包数据发送
//        DWT_GetDeltaT(&uart3_tx_manager.uart_tx_count); //标记时间
//    }
//    if (huart == &huart5)
//    {
//        uart5_tx_manager.uart_tx_state = TX_OK;         //表示uart外设完成一包数据发送
//        DWT_GetDeltaT(&uart5_tx_manager.uart_tx_count); //标记时间
//    }
    if (huart == &huart7)
    {
        uart7_tx_manager.uart_tx_state = TX_OK;         //表示uart外设完成一包数据发送
        DWT_GetDeltaT(&uart7_tx_manager.uart_tx_count); //标记时间
    }
    if (huart == &huart10)
    {
        uart10_tx_manager.uart_tx_state = TX_OK;         //表示uart外设完成一包数据发送
        DWT_GetDeltaT(&uart10_tx_manager.uart_tx_count); //标记时间
    }
}
/*************************************************************************************************/
/*****************************************数据接收中断处理******************************************/
/*user data struct start*/
// #include "referee.h"
// extern referee_t referee;
/*user data struct end*/
//void uart1_IRQHandler(void)
//{
//    // referee_fbkdata(&referee, uart1_rx_buf);
//    HAL_UARTEx_ReceiveToIdle_IT(&huart1, uart1_rx_buf, UART_RX_BUF_LEN);
//}
//void uart2_IRQHandler(void)
//{
//    HAL_UARTEx_ReceiveToIdle_IT(&huart2, uart2_rx_buf, UART_RX_BUF_LEN);
//}
//void uart3_IRQHandler(void)
//{
//    HAL_UARTEx_ReceiveToIdle_IT(&huart3, uart3_rx_buf, UART_RX_BUF_LEN);
//}
//void uart5_IRQHandler(void)
//{
//    HAL_UARTEx_ReceiveToIdle_IT(&huart5, uart5_rx_buf, UART_RX_BUF_LEN);
//}
void uart7_IRQHandler(void)
{
    HAL_UARTEx_ReceiveToIdle_IT(&huart7, uart7_rx_buf, UART_RX_BUF_LEN);
}
void uart10_IRQHandler(void)
{
    HAL_UARTEx_ReceiveToIdle_IT(&huart10, uart10_rx_buf, UART_RX_BUF_LEN);
}
