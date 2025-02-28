#include "chassis_task.hpp"
#include "chassis_ctrl.hpp"
#include "chassis_confit.hpp"
#include "bsp_dwt.hpp"
#include "communication.hpp"
#include "dr16.hpp"

extern "C"{
    #include "chassis_task.h"
    #include <string.h>
    #include <stdint.h>
    #include "FreeRTOS.h"
    #include "task.h"
}


using namespace DR16_n;
using namespace chassis_ctrl_n;
BSP_DWT_n::BSP_DWT_c *chassis_dwt = BSP_DWT_n::BSP_DWT_c::ECF_Get_DwtInstance(); 


//前置声明
void DecodeGimbalData(BSP_CAN_Part_n::CANInstance_c* register_instance);
void SendChassisData(BSP_CAN_Part_n::CANInstance_c* Interboard_com,const chassis_ctrl_c* chassis_ins);
DR16_c *dr16_data = DR16_c::dr16_->GetClassPtr();



void Chassis_Task(void const *argument)
{
    //遥控器信息云台传到底盘通道
    BSP_CAN_Part_n::CANInstance_c gimbal2chassis(&hcan2, CHASSIS_ID, GIMBAL_ID, CAN_ID_STD, DecodeGimbalData);
    chassis_ctrl_c::Get_Chassis_Instance()->Chassis_Init();
    ECF_referee_uart_init();
    
    
    uint8_t control_Hz = 0;
    while(1)
    {
        if(control_Hz++ % 5 == 0)
        {
            //发送裁判系统数据到云台
            SendChassisData(&gimbal2chassis, chassis_ctrl_c::Get_Chassis_Instance());
        }
        chassis_ctrl_c::Get_Chassis_Instance()->Chassis_Ctrl_Loop();
        //发送电机控制数据
        DJI_Motor_n::DJIMotorControl();
        vTaskDelay(1);
    }
    
}




//(暂时不知道这些干嘛的)
uint8_t id,fric_onoff,gimbal_mode,fire_mode,aim_mode;
int16_t temp_pitch_comp;
float pitch_comp;
//底盘解析云台数据回调函数
static void DecodeGimbalData(BSP_CAN_Part_n::CANInstance_c* register_instance)
{
    if(register_instance->rx_id_ != GIMBAL_ID)
    {
        return;
    }

    DR16_Online();//保持在线状态

    uint8_t *data = register_instance->rx_buff;

    if(data[7] != 0xFF)
    {
        dr16_data->RC_HandleData.rc.ch[2] = (((int16_t)((data[0] | data[1]<<8))&0x7FF)-1024);
        dr16_data->RC_HandleData.rc.ch[3] = (((int16_t)(((data[1] >> 3) & 0x1F)| (data[2]<<5))&0x7FF)-1024);
        dr16_data->RC_HandleData.rc.s1 = (uint8_t)(data[2]>>6)&0x03;
        dr16_data->RC_HandleData.kb.key_code = (uint16_t)(((data[3] << 8) | data[4]));
        //temp_pitch_comp=(int16_t)((((data[5]|data[6]<<8)&0x3FFF)-8192))/1000.0f;
        dr16_data->RC_HandleData.rc.s2 = (uint8_t)(data[6]>>6)&0x03;
        // id=data[7]&0x01;
        // fric_onoff=(data[7]>>1)&0x01;
        // gimbal_mode=(data[7]>>2)&0x07;
        // fire_mode=(data[7]>>5)&0x01;
        // aim_mode=(data[7]>>6)&0x03;

        //防止解析错误
        user_value_limit(dr16_data->RC_HandleData.rc.ch[2], -660, 660);
        user_value_limit(dr16_data->RC_HandleData.rc.ch[3], -660, 660);
    }
    else //导航模式
    {
        navigation_control_data navigation_p;
        // 从 data 中恢复 uint16_t 数据
        uint16_t f1_high = (data[0] << 8) | data[1];
        uint16_t f2_high = (data[2] << 8) | data[3];
        //uint16_t f3_high = (data[4] << 8) | data[5];

        // 将 uint16_t 转换为 uint32_t（低 16 位补零）
        uint32_t f1_bits = *((uint32_t*)&(f1_high)) << 16;
        uint32_t f2_bits = *((uint32_t*)&(f2_high)) << 16;
        //uint32_t f3_bits = *((uint32_t*)&(f3_high)) << 16;

        // 将 uint32_t 转换为 float
        navigation_p.x_speed = *((float*)&f1_bits);
        navigation_p.y_speed = *((float*)&f2_bits);
        //navigation_p.z_speed = *((float*)&f3_bits);

        navigation_p.start_rotate = data[4];

        chassis_ctrl_c::Get_Chassis_Instance()->Set_Chassis_navigation(navigation_p);

        dr16_data->RC_HandleData.rc.s2 = data[6];
    }

}


static void SendChassisData(BSP_CAN_Part_n::CANInstance_c* Interboard_com,const chassis_ctrl_c* chassis_ins)
{
    static uint8_t aim_color=0;
	int16_t temp_bullet_spd = chassis_ins->referee->Shoot_Data.bullet_speed*100;
    //己方为红方；瞄蓝色 ； 反之亦然
    aim_color=(chassis_ins->referee->Robot_Status.robot_id<11)?1:0;

    
    Interboard_com->tx_buff[0] = (chassis_ins->referee->Power_Heat.shooter_id1_17mm_cooling_heat >> 8);//当前热量
    Interboard_com->tx_buff[1] = (chassis_ins->referee->Power_Heat.shooter_id1_17mm_cooling_heat);
    Interboard_com->tx_buff[2] = (chassis_ins->referee->Robot_Status.shooter_barrel_heat_limit >> 8);//当前热量限制
    Interboard_com->tx_buff[3] = (chassis_ins->referee->Robot_Status.shooter_barrel_heat_limit);
    Interboard_com->tx_buff[4] = (temp_bullet_spd >> 8);//当前射速
    Interboard_com->tx_buff[5] = (temp_bullet_spd);
    Interboard_com->tx_buff[6] = (chassis_ins->referee->Robot_Status.robot_level);//当前等级
    Interboard_com->tx_buff[7] = (uint8_t)(aim_color);
    Interboard_com->ECF_Transmit(1);
}

