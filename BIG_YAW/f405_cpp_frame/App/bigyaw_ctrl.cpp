#include "bigyaw_ctrl.hpp"


/**
 * @brief  大Yaw构造函数
 */
big_yaw_ctrl_c::big_yaw_ctrl_c():yaw_motor_ctrl_c(1,2,0,0),pluck_ctrl_c(3,&hcan1),
bigyaw_to_chassis(&hcan2,BIGYAW_ID,CHASSIS_ID,CAN_ID_STD,DecodeChassisData),
big_yaw_to_smallyaw(&hcan2,BIGYAW_ID,SMALLYAW_ID,CAN_ID_STD,DecodeSmallYawData)
{
    memset(&dr16_data,0,sizeof(dr16_data));
    memset(&temp_shoot_info,0,sizeof(temp_shoot_info));

    Yaw_init();
    Pluck_init();
}


/**
 * @brief  大Yaw主函数
 */
void big_yaw_ctrl_c::Big_Yaw_ctrl_loop()
{
    if(!allow_fire_)
    {
        Pluck_stop_fire();
    }
    else
    {
        Pluck_allow_fire();
    }
    Yaw_ctrl_loop(gimbal_behaviour_,dr16_data);
    Pluck_ctrl_loop(dr16_data,temp_shoot_info);

    DJI_Motor_n::DJIMotorControl();
}


/**
 * @brief  大Yaw底盘数据解析函数
 * @note   解析裁判系统数据
 */
void big_yaw_ctrl_c::DecodeChassisData(BSP_CAN_Part_n::CANInstance_c* register_instance)
{
    if(register_instance->rx_id_!=CHASSIS_ID)
    {
        return;
    }

    // 直接使用 rx_buff，避免额外的内存拷贝
    uint8_t* data = register_instance->rx_buff;

    // 减少重复调用 Get_big_yaw_instance()
    auto& instance = *big_yaw_ctrl_c::Get_big_yaw_instance();
    
    instance.temp_shoot_info.shoot_barrel_heat_current = (uint16_t)(((data[0] << 8) | data[1]));
    instance.temp_shoot_info.shoot_barrel_heat_limit   = (uint16_t)((data[2] << 8) | data[3]);
    instance.temp_shoot_info.shoot_bullet_speed        = (int16_t)((data[4] << 8) | data[5]);
    instance.temp_shoot_info.shoot_bullet_speed /= 100;
    instance.temp_shoot_info.robot_level               = data[6];
    // aim_color = data[7];
}

/**
 * @brief  大Yaw解析小yaw数据函数
 * @note   解析遥控器数据、允许发射标志位和云台行为模式
 */
void big_yaw_ctrl_c::DecodeSmallYawData(BSP_CAN_Part_n::CANInstance_c* register_instance)
{
    if(register_instance->rx_id_ != SMALLYAW_ID)
    {
        return;
    }

    // 直接使用 rx_buff，避免额外的内存拷贝
    uint8_t* data = register_instance->rx_buff;

    // 减少重复调用 Get_big_yaw_instance()
    auto& instance = *big_yaw_ctrl_c::Get_big_yaw_instance();

    instance.dr16_data.rc.ch[4] = ((static_cast<uint16_t>(data[1] & 0x03) << 8) | data[0]) - 1024;
    instance.dr16_data.rc.ch[0] = ((static_cast<uint16_t>(data[1]) << 3) | (data[2] & 0x3f)) - 1024;
    instance.dr16_data.rc.s2 = (data[2] >> 6) & 0x03;
    instance.dr16_data.mouse.x = (static_cast<uint16_t>(data[3]) << 8) | (data[4]);
    instance.dr16_data.kb.key_code = (static_cast<uint16_t>(data[5]) << 8) | (data[6]);
    instance.gimbal_behaviour_ = static_cast<gimbal_behaviour_e>(data[7] & 0x07); // 云台行为模式3位
    instance.allow_fire_ = (data[7] >> 6) & 0x01; // 允许发射标志位1位
    instance.dr16_data.mouse.press_l = (data[7] >> 7) & 0x01; // 鼠标左键
}


big_yaw_ctrl_c* big_yaw_ctrl_c::Get_big_yaw_instance()
{
    if(big_yaw_instance_==nullptr)
    {
        big_yaw_instance_=new big_yaw_ctrl_c();
    }
    return big_yaw_instance_;
}

big_yaw_ctrl_c* big_yaw_ctrl_c::big_yaw_instance_ = nullptr;
