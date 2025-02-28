#ifndef __DATA_RECIVE_HPP
#define __DATA_RECIVE_HPP

#include "real_main.hpp"
#include "dr16.hpp"
#include "bsp_can.hpp"


#define GIMBAL_ID 0x411
#define CHASSIS_ID 0x112

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "can.h"

extern CAN_HandleTypeDef hcan2;

#ifdef __cplusplus
}
#endif

//通信类
// class communication_c{
//     public:
//         communication_c();
//         DR16_n::DR16_c *dr16_data;
//         static communication_c* gimbal2chassis; //唯一实例
//         BSP_CAN_Part_n::CANInstance_c gimbal_and_chassis;
//         //定义一个发送函数

//         static communication_c* Get_communication_instance();

// };


#endif
