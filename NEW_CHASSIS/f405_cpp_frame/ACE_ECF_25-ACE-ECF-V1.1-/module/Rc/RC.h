#ifndef __BSP_DR16_H
#define __BSP_DR16_H
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 开放给it.c中断向量表的中断服务函数
 */
void ECF_Dr16_UART_Handler(void);
void ECF_RC_Init(void);
void ECF_DT7_UART_Handler();
void ECF_REFFEREE_UART_Handler(void);
#ifdef __cplusplus
}
    
#endif
#endif