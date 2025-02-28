//不知道为啥放在这里面会有问题，所以放到chassis_task.cpp里面了

// //云台底盘通信处理
// #include "communication.hpp"

// static void DecodeGimbalData(BSP_CAN_Part_n::CANInstance_c* register_instance);

// communication_c* communication_c::gimbal2chassis = new communication_c();

// communication_c::communication_c():gimbal_and_chassis(&hcan2, CHASSIS_ID, GIMBAL_ID, CAN_ID_STD,NULL)
// {
//     this->dr16_data = DR16_n::DR16_c::dr16_->GetClassPtr();
//     this->gimbal_and_chassis.ECF_SetRxCallBack(DecodeGimbalData);
// }

// communication_c* communication_c::Get_communication_instance()
// {
//     return communication_c::gimbal2chassis;
// }


// //(暂时不知道这些干嘛的)
// uint8_t id,fric_onoff,gimbal_mode,fire_mode,aim_mode;
// int16_t temp_pitch_comp;
// float pitch_comp;
// //底盘解析云台数据回调函数
// static void DecodeGimbalData(BSP_CAN_Part_n::CANInstance_c* register_instance)
// {
//     if(register_instance->rx_id_ != GIMBAL_ID)
//     {
//         return;
//     }

//     DR16_Online();//保持在线状态

//     uint8_t *data = register_instance->rx_buff;

//     communication_c::gimbal2chassis->dr16_data->RC_HandleData.rc.ch[2] = (((int16_t)((data[0] | data[1]<<8))&0x7FF)-1024);
//     communication_c::gimbal2chassis->dr16_data->RC_HandleData.rc.ch[3] = (((int16_t)(((data[1] >> 3) & 0x1F)| (data[2]<<5))&0x7FF)-1024);
//     communication_c::gimbal2chassis->dr16_data->RC_HandleData.rc.s1 = (uint8_t)(data[2]>>6)&0x03;
//     communication_c::gimbal2chassis->dr16_data->RC_HandleData.kb.key_code = (uint16_t)(((data[3] << 8) | data[4]));
//     //temp_pitch_comp=(int16_t)((((data[5]|data[6]<<8)&0x3FFF)-8192))/1000.0f;
//     communication_c::gimbal2chassis->dr16_data->RC_HandleData.rc.s2 = (uint8_t)(data[6]>>6)&0x03;
//     // id=data[7]&0x01;
//     // fric_onoff=(data[7]>>1)&0x01;
//     // gimbal_mode=(data[7]>>2)&0x07;
//     // fire_mode=(data[7]>>5)&0x01;
//     // aim_mode=(data[7]>>6)&0x03;

// }

