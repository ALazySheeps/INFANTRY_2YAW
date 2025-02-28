#ifndef __REAL_MAIN_H
#define __REAL_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "usb_device.h"

    //时钟初始化
    void SystemClock_Config(void);
    //RTOS初始化
    void MX_FREERTOS_Init(void);
    // void MX_GPIO_Init(void);
    // void MX_DMA_Init(void);
    // void MX_CAN1_Init(void);
    // void MX_USART3_UART_Init(void);
    // void MX_CAN2_Init(void);


#ifdef __cplusplus
}
#endif



#endif
