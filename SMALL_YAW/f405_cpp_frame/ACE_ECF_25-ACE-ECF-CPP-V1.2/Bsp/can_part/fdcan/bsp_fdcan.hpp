#ifndef __BSP_FDCAN_HPP
#define __BSP_FDCAN_HPP

#include "bsp_can_template.hpp"

#ifdef __cplusplus
extern "C" {
#endif

#include "fdcan.h"
#include "main.h"

#ifdef __cplusplus
}
#endif

namespace BSP_CAN_Part_n
{

struct FDCAN_Init_Config_s;

//只做继承用
class FDCANInstance_c:public CAN_Main_Class_c
{
    private:
        static FDCANInstance_c *fdcan_instance_[MX_REGISTER_CNT]; // CAN实例指针数组
        FDCAN_HandleTypeDef *fdcan_handle_;                      // fdcan句柄       
        FDCAN_TxHeaderTypeDef txconf;                            // FDCAN报文发送配置
        // 接收的回调函数,用于解析接收到的数据
    public:
    void (*fdcan_module_callback)(FDCANInstance_c* register_instance);
    FDCANInstance_c(
            FDCAN_HandleTypeDef *fdcan_handle,
            uint32_t tx_id,
            uint32_t rx_id,
            uint32_t SAND_IDE
        );
    FDCANInstance_c(
            FDCAN_HandleTypeDef *fdcan_handle,
            uint32_t tx_id,
            uint32_t rx_id,
            uint32_t SAND_IDE,
            void (*fdcan_module_callback)(FDCANInstance_c* register_instance)
        );
    FDCANInstance_c(
        uint32_t tx_id,
        uint32_t rx_id);

        FDCANInstance_c(FDCAN_Init_Config_s fdcan_config);
        void FDCAN_Motor_Init(FDCAN_HandleTypeDef *fdcan_handle,uint32_t SAND_IDE);
        FDCAN_HandleTypeDef* ECF_GetFDCanhandle();   
        static void ECF_FIFOxCallback(FDCAN_HandleTypeDef *_hfdcan, uint32_t fifox); // 自己写的FDCAN接收回调函数
        void ECF_SetRxCallBack(void (*fdcan_module_callback)(FDCANInstance_c* register_instance));
        void ECF_SetDLC(uint32_t length);
        CAN_State_e ECF_Transmit(float timeout);
        void Filter_Config(FDCANInstance_c *instance);
        
};

/* FDCAN实例初始化结构体,将此结构体指针传入注册函数 */
struct FDCAN_Init_Config_s
{
    FDCAN_HandleTypeDef *fdcan_handle;          // fdcan句柄
    uint32_t tx_id;                             // 发送id
    uint32_t rx_id;                             // 接收id
    void (*fdcan_module_callback)(FDCANInstance_c* register_instance); 
    uint32_t SAND_IDE;                          // 标准帧还是拓展帧
};

}

#endif /* BSP_FDCAN_HPP */