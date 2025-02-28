/*************************** Dongguan-University of Technology -ACE**************************
 * @file    dm_mit_mode.cpp
 * @author  KazuHa12441
 * @version V1.0
 * @date    2024/10/2
 * @brief   达妙电机mit模式
 * 
 * @todo: 小凳有需要的话来完善吧CAN吧
********************************************************************************************
 * @attention 
 * 1.创建一个参数结构体DM_ModePrame_s
 * 2.创建一个对象dm_mit_mode_c 
 * 3.调用SetCallBack函数将回调函数写入对象中
 * 4.调用StartStateSet进行初始状态选择
 * 5.调用对象的方法Transimit发送电机数据()
 * 6.接收的数据会在类变量get_data_中
 * 7.如果想调用内部数据 可以调用DM_GetMotorData_s *ReturnMotorData(void)，它将返回一个数据指针
**********************************************************************************************/
#include "dm_mit_mode.hpp"

// 极致
namespace DM_Motor_n
{
    /// @brief mit模式构造
    /// @param fdcan_handle 使用fdcan句柄 
    /// @param param DM_参数结构体
    /// @param tx_id DM电机的发送id
    /// @param rx_id DM电机的接收id
    /// @param SAND_IDE 数据格式(标准帧，拓展帧) 
    /// @param fdcan_module_callback fdcan的接收回调
    DM_Mit_Mode_c::DM_Mit_Mode_c(CAN_HandleTypeDef *can_handle,
                                 DM_ModePrame_s *param,
                                 uint32_t tx_id,
                                 uint32_t rx_id,
                                 uint32_t SAND_IDE
                                 )
        : DM_Motor_Main_c(param),
          CANInstance_c(can_handle, tx_id, rx_id, SAND_IDE)
    {
        get_data_.nowState = UNABLE;
        mode_ = MIT;
        dm_idx_++; 
    }

    /// @brief  mit发送数据解码
    /// @param _pos 给定位置
    /// @param _vel 给定速度
    /// @param _Kp  给定kp
    /// @param _Kd  给定kd
    /// @param _torq 给定力矩
    void DM_Mit_Mode_c::TxDataProcessing(float _pos, float _vel, float _Kp, float _Kd, float _torq)
    {
        uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
        pos_tmp = float_to_uint(_pos, param_->p_min, param_->p_max, param_->postion_bits);
        vel_tmp = float_to_uint(_vel, param_->v_min, param_->v_max, param_->velocity_bits);
        kp_tmp  = float_to_uint(_Kp, param_->kp_min, param_->kp_max, param_->kp_bits);
        kd_tmp  = float_to_uint(_Kd, param_->kd_min, param_->kd_max, param_->kd_bits);
        tor_tmp = float_to_uint(_torq, param_->t_min, param_->t_max, param_->toeque_bits);

        txbuffer_[0] = pos_tmp >> 8;
        txbuffer_[1] = pos_tmp;
        txbuffer_[2] = vel_tmp >> 4;
        txbuffer_[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
        txbuffer_[4] = kp_tmp;
        txbuffer_[5] = kd_tmp >> 4;
        txbuffer_[6] = ((kd_tmp & 0xf) << 4) | (tor_tmp >> 8);
        txbuffer_[7] = tor_tmp;
    }
     
    void DM_Mit_Mode_c::StartStateSet(DM_StartState_e State)
    {
        DMMotorStateSet(State);
        TxDataToBuffer(tx_buff, txbuffer_);
        ECF_Transmit(TXTIME);
    }
    
    /// @brief mit发送函数
    /// @param _pos 位置给定
    /// @param _vel 速度给定
    /// @param _Kp  p参数
    /// @param _Kd  d参数
    /// @param _torq 力矩给定
    void DM_Mit_Mode_c::Transmit(float _pos, float _vel, float _Kp, float _Kd, float _torq)
    {
        {
            TxDataProcessing(_pos, _vel, _Kp, _Kd, _torq);
            TxDataToBuffer(tx_buff, txbuffer_);
            ECF_Transmit(TXTIME);
        }
    }
}