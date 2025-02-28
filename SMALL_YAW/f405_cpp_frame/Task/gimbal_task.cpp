#include "gimbal_task.hpp"

//底盘发来的发射机构信息
shoot_msg_t temp_shoot_info;

/******************************************************外部声明************************************************/


/******************************************************函数声明************************************************/
void Gimbal2Chassis(BSP_CAN_Part_n::CANInstance_c* gimbal2chassis,const DR16_n::RC_ctrl_t* dr16_data);
void SmallYaw2BigYaw(BSP_CAN_Part_n::CANInstance_c* gimbal2chassis,const DR16_n::RC_ctrl_t* dr16_data,uint8_t gimbal_mode,uint8_t allow_fire);
void Decode_Chassis_Data(BSP_CAN_Part_n::CANInstance_c *register_instance);


void Gimbal_Task(void const *argument)
{
    uint8_t control_Hz=0;

    taskENTER_CRITICAL();
    //云台初始化
    gimbal_ctrl_c::Get_Gimbal_Instance()->Gimbal_Init();

    //开启遥控器传输(有点神奇明明已经清空了但是不连接遥控器上电还是小陀螺)
    DR16_n::DR16_c *dr16 = DR16_n::DR16_c::dr16_->GetClassPtr();
    if(dr16->GetDR16State() == DR16_n::LOST)
    {
        dr16->RC_Data_init_Clear();
        dr16->RC_HandleData.rc.s1 = RC_SW_DOWN;
        dr16->RC_HandleData.rc.s2 = RC_SW_MID;
    }
    dr16->dr16_rece_->USART_rx_start(); //开始传输
    //开启板间通信
    BSP_CAN_Part_n::CANInstance_c smallyaw_to_chassis(&hcan2, BIGYAW_ID, CHASSIS_ID, CAN_ID_STD, Decode_Chassis_Data);
    BSP_CAN_Part_n::CANInstance_c smallyaw_to_bigyaw(&hcan2, BIGYAW_ID, SMALLYAW_ID, CAN_ID_STD, nullptr);
    taskEXIT_CRITICAL();

    for(;;)
    {
      //taskENTER_CRITICAL();
      if(control_Hz++%5==0)//200Hz控制频率
      {
          Gimbal2Chassis(&smallyaw_to_chassis,&dr16->RC_HandleData);
          SmallYaw2BigYaw(&smallyaw_to_bigyaw,&dr16->RC_HandleData,gimbal_ctrl_c::Get_Gimbal_Instance()->gimbal_mode,gimbal_ctrl_c::Get_Gimbal_Instance()->allow_fire_);
      }
      gimbal_ctrl_c::Get_Gimbal_Instance()->Gimbal_CtrlLoop(temp_shoot_info);
      //taskEXIT_CRITICAL();


      //taskENTER_CRITICAL();
      //DJI_Motor_n::DJIMotorControl();
      //taskEXIT_CRITICAL();
      Virtual_send(gimbal_ctrl_c::Get_Gimbal_Instance()->imu->Pitch,gimbal_ctrl_c::Get_Gimbal_Instance()->imu->Yaw,
      gimbal_ctrl_c::Get_Gimbal_Instance()->imu->Roll,gimbal_ctrl_c::Get_Gimbal_Instance()->virtual_tx_data.bullet_speed,
      gimbal_ctrl_c::Get_Gimbal_Instance()->virtual_tx_data.reset_tracker,gimbal_ctrl_c::Get_Gimbal_Instance()->virtual_tx_data.now_mode);

      vTaskDelay(1);
    }
}



//云台数据发送至底盘
void Gimbal2Chassis(BSP_CAN_Part_n::CANInstance_c* gimbal2chassis,const DR16_n::RC_ctrl_t* dr16_data)
{
    gimbal2chassis->tx_buff[0] = (dr16_data->rc.ch[2] + 1024);//ch2+1024 低8位 2^11
    gimbal2chassis->tx_buff[1] = (((dr16_data->rc.ch[2] + 1024)>>8)|((dr16_data->rc.ch[3]+1024)<<3));//ch3+1024 低五位|ch2+1024 11-10-9位 
    gimbal2chassis->tx_buff[2] = (((dr16_data->rc.ch[3] + 1024 )>> 5)|(dr16_data->rc.s1 << 6));//ch3+1024 高六位 | s1
    gimbal2chassis->tx_buff[3] = (dr16_data->kb.key_code >> 8);
    gimbal2chassis->tx_buff[4] = (dr16_data->kb.key_code);
    gimbal2chassis->tx_buff[5] = 0;
    //gimbal2chassis->tx_buff[6] = (((pitch_comp+8192)>>8) | (dr16_data->rc.s2)<<6);
    gimbal2chassis->tx_buff[6] = (0 | (dr16_data->rc.s2)<<6);
    //gimbal2chassis->tx_buff[7] = (id | fric_state <<1 | gimbal_mode<<2 | fire_mode <<5 | aim_mode << 6);
    gimbal2chassis->tx_buff[7] = 0;
    
    gimbal2chassis->ECF_Transmit(1);
}

//小Yaw发送至大Yaw
void SmallYaw2BigYaw(BSP_CAN_Part_n::CANInstance_c* gimbal2chassis,const DR16_n::RC_ctrl_t* dr16_data,uint8_t gimbal_mode,uint8_t allow_fire)
{
    gimbal2chassis->tx_buff[0] = (dr16_data->rc.ch[4] + 1024); // ch4+1024 低8位 2^11
    gimbal2chassis->tx_buff[1] = (((dr16_data->rc.ch[4] + 1024) << 3) | ((dr16_data->rc.ch[0] + 1024) >> 8)); // ch0+1024 低五位|ch4+1024 11-10-9位 
    gimbal2chassis->tx_buff[2] = (((dr16_data->rc.ch[0] + 1024) >> 5) | (dr16_data->rc.s2 << 6)); // ch0+1024 高六位 | s2
    gimbal2chassis->tx_buff[3] = (dr16_data->mouse.x >> 8);
    gimbal2chassis->tx_buff[4] = (dr16_data->mouse.x);
    gimbal2chassis->tx_buff[5] = (dr16_data->kb.key_code >> 8);
    gimbal2chassis->tx_buff[6] = (dr16_data->kb.key_code);
    // 云台模式3位，允许发射标志位1位,鼠标左键是否按下一位
    gimbal2chassis->tx_buff[7] = (gimbal_mode & 0x07) | ((allow_fire & 0x01) << 6) | ((dr16_data->mouse.press_l & 0x01) << 7); // 确保了云台模式和允许发射标志位的正确设置
    
    gimbal2chassis->ECF_Transmit(1);
}

//小yaw接收底盘回调
void Decode_Chassis_Data(BSP_CAN_Part_n::CANInstance_c *register_instance)
{
    if(register_instance->rx_id_ != CHASSIS_ID)
        return;

    uint8_t data[8] = {0};
    memcpy(&data,register_instance->rx_buff,8);

    temp_shoot_info.shoot_barrel_heat_current = (uint16_t)(((data[0] << 8) | data[1]));
    temp_shoot_info.shoot_barrel_heat_limit   = (uint16_t)( (data[2] << 8) | data[3]);
    temp_shoot_info.shoot_bullet_speed        = (int16_t) ( (data[4] << 8) | data[5]);//接收后需要处理以下
    temp_shoot_info.shoot_bullet_speed/=100;      
    temp_shoot_info.robot_level = (uint8_t)(data[6]);
    temp_shoot_info.aim_color=(uint8_t)(data[7]);
}



