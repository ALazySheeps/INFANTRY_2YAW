#ifndef __BSP_CAN_HPP
#define __BSP_CAN_HPP


#include "bsp_can_template.hpp"
#include "real_main.hpp"
#ifdef __cplusplus
extern "C" {
#endif

#include "can.h"


#ifdef __cplusplus
}
#endif
#include <functional>

namespace BSP_CAN_Part_n
{

struct CAN_Init_Config_s;

class CANInstance_c:public CAN_Main_Class_c
{
    public:
    CANInstance_c(
            CAN_HandleTypeDef *can_handle,
            uint32_t tx_id,
            uint32_t rx_id,
            uint32_t SAND_IDE
        );
        CANInstance_c(
            CAN_HandleTypeDef *can_handle,
            uint32_t tx_id,
            uint32_t rx_id,
            uint32_t SAND_IDE,
            void (*can_module_callback)(CANInstance_c* register_instance)
        );
        CANInstance_c(
            uint32_t tx_id,
            uint32_t rx_id);

        CANInstance_c(CAN_Init_Config_s can_config);
        // 外部调用函数
        static void ECF_FIFOxCallback(CAN_HandleTypeDef *_hcan, uint32_t fifox); // 自己写的CAN接收回调函数
        void ECF_SetDLC(uint32_t length);
        CAN_State_e ECF_Transmit(float timeout);
        void ECF_SetRxCallBack(void (*can_module_callback)(CANInstance_c* register_instance));     
        CAN_HandleTypeDef* ECF_GetCanhandle();   
        void CAN_Motor_Init(CAN_HandleTypeDef *can_handle,uint32_t SAND_CanIDE); // 电机发送对象的初始化函数
        void Chance_Ext_Tx_ID(uint32_t ext_id);
        
    protected:
        static CANInstance_c *can_instance_[MX_REGISTER_CNT]; // CAN实例指针数组
        void Filter_Config(CANInstance_c *instance);
        CAN_TxHeaderTypeDef txconf;               // CAN报文发送配置
        CAN_HandleTypeDef   *can_handle_;           // can句柄  
        void* private_data;                  
        // 接收的回调函数,用于解析接收到的数据
        void (*can_module_callback)(CANInstance_c* register_instance); // callback needs an instance to tell among registered ones
};

/* CAN实例初始化结构体,将此结构体指针传入注册函数 */
// typedef struct
struct CAN_Init_Config_s
{
    CAN_HandleTypeDef *can_handle;              // can句柄
    uint32_t tx_id;                             // 发送id
    uint32_t rx_id;                             // 接收id
    void (*can_module_callback)(CANInstance_c* register_instance); 
    uint32_t SAND_IDE;                          // 标准帧还是拓展帧
};

}
#endif /* BSP_CAN_HPP */