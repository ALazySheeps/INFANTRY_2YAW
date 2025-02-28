#ifndef  __BSP_CAN_TEMP_HPP
#define  __BSP_CAN_TEMP_HPP

#define MX_REGISTER_CNT 28u//18u     // 这个数量取决于CAN总线的负载(不能等于这个数)
#define MX_FILTER_CNT (2 * 14)  // 最多可以使用的CAN过滤器数量,目前远不会用到这么多
#define DEVICE 2                // 根据板子设定,F407IG有CAN1,CAN2,因此为2;F334只有一个,则设为1

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#ifdef __cplusplus
}
#endif

namespace BSP_CAN_Part_n
{

typedef enum
{
   UNDEFINE    = 0,
   DEFINE
}Define_State_e;

typedef enum 
{
   CAN_OK,
   CAN_ERROR
}CAN_State_e;

class CAN_Main_Class_c
{
    protected:
     static uint8_t idx_;                      // 全局CAN实例索引,每次有新的模块注册会自增             
     uint32_t tx_mailbox;                      // CAN消息填入的邮箱号
     uint8_t rx_len_;                          // 接收长度,可能为0-8
      
    public: 
     uint32_t tx_id_;                          // 发送id
     uint32_t rx_id_;                          // 接收id
     uint8_t rx_buff[8];                       // 数据接收
     uint8_t tx_buff[8];                       // 发送缓存,发送消息长度可以通过ECF_CAN_SetDLC()设定,最大为8
     float tx_wait_time_;                      // block时间
     virtual void ECF_SetDLC(uint32_t length) = 0;
     virtual CAN_State_e ECF_Transmit(float timeout) = 0;
     CAN_Main_Class_c(uint32_t tx_id,uint32_t rx_id):tx_id_ (tx_id), rx_id_(rx_id){}
};

}


#endif /* BSP_CAN_TEMP_HPP */