#ifndef BSP_UART_H
#define BSP_UART_H

#include "usart.h"
#include "main.h"
#include "bsp_dwt.h"
#include "string.h"

typedef enum
{
    TX_OK = 0,
    TX_BUSY = 1,
    TX_FULL = 2,
    TX_LEN_ERROR = 3,
    TX_NO_DATA = 4
} UART_TX_STATE;

void bsp_uart_init(void);
/****************************uart1*******************************/
UART_TX_STATE uart1_add_package(uint8_t data[], int len);
UART_TX_STATE uart1_tx_refresh(void);
void uart1_IRQHandler(void);
/****************************uart2*******************************/
UART_TX_STATE uart2_add_package(uint8_t data[], int len);
UART_TX_STATE uart2_tx_refresh(void);
void uart2_IRQHandler(void);
/****************************uart3*******************************/
UART_TX_STATE uart3_add_package(uint8_t data[], int len);
UART_TX_STATE uart3_tx_refresh(void);
void uart3_IRQHandler(void);
/****************************uart5*******************************/
UART_TX_STATE uart5_add_package(uint8_t data[], int len);
UART_TX_STATE uart5_tx_refresh(void);
void uart5_IRQHandler(void);
/****************************uart7*******************************/
UART_TX_STATE uart7_add_package(uint8_t data[], int len);
UART_TX_STATE uart7_tx_refresh(void);
void uart7_IRQHandler(void);
/****************************uart10*******************************/
UART_TX_STATE uart10_add_package(uint8_t data[], int len);
UART_TX_STATE uart10_tx_refresh(void);
void uart10_IRQHandler(void);

#endif // BSP_UART_H
