/*************************** Dongguan-University of Technology -ACE**************************
 * @file    dr16_h7.cpp
 * @author  KazuHa12441
 * @version V1.0
 * @date    2024/10/15
 * @brief
 *   获取dr16信息
 * @todo  安全任务实现
 ********************************************************************************************
 * @verbatim
 *     
 *     1.对象已经提前创建
 *     2.先创建一个类指针DR16_c *u
 *     3.u = DR16_n::DR16_c::dr16_->GetClassPtr();获取实列指针
 *     4.调用u->dr16_rece_->USART_rx_start();开始传输
 *     5.用DR16_c::GetDataStructPtr()获取数据
 *     6.DR16_c::GetDR16State()获取dt7状态(正常/失联)(可以获取或者不获取)
 *     7.失联处理
 *     8.在 it文件中调用DR16_Online(void)保持dr16在线状态
 ************************** Dongguan-University of Technology -ACE***************************/

//直接在回调函数里面Online？
#include "dr16.hpp"
#include "safe_task.hpp"


namespace DR16_n
{
    uint8_t double_buffer_[2][DR16_RX_BUFFER_NUM*2]; // 双缓冲区
    USART_N::usart_init_t dr16_uart_param =
    {
        .usart_handle_   = &huart3,
        .rxbuf_size_     = DR16_RX_BUFFER_NUM*2,           // 接收区大小
        .rx_type_        = USART_N::USART_RX_DMA_IDLE_D, // 接收类型
        //.tx_type_        = USART_N::USART_TX_DMA,        // 发送类型
        .usart_rx_callback_ptr_ = &callback,
        .usart_data_length_ =  RC_FRAME_LENGTH,
        .rx_buff_ptr_    = double_buffer_[0],
        .secondebuf_ptr_ = double_buffer_[1]
    };

     DR16_c *DR16_c::dr16_ = new DR16_c(DEADLINE);
     Safe_task_c *dr16_lost_ = new Safe_task_c("dr16",5000,&(DR16_n::LostHandle),nullptr); 
    
    /// @brief dr16的父类构造
    /// @param dead_line_ 
    DR16_c::DR16_c(int16_t dead_line_): dead_line_(dead_line_), lost_time_(0)
    {
        UARTParam  = &dr16_uart_param;
        dr16_rece_ =  new USART_N::usart_c(dr16_uart_param);
    }

    /// @brief 数据处理
    /// @param pData 缓冲数组
    void DR16_c::DataHandle(volatile const uint8_t *pData)
    {
        if(dt7_state_ == LOST)
        {
            dt7_state_ = NORMAL;
        }
        LostTimeSetZero();
        RC_HandleData.rc.ch[0] = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;
        RC_HandleData.rc.ch[1] = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
        RC_HandleData.rc.ch[2] = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) | //!< Channel 2
                                  ((int16_t)pData[4] << 10)) &
                                 0x07FF;
        RC_HandleData.rc.ch[3] = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5] << 7)) & 0x07FF;

        RC_HandleData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2; //!< Switch left
        RC_HandleData.rc.s2 = ((pData[5] >> 4) & 0x0003);

        RC_HandleData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);   //!< Mouse X axis
        RC_HandleData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);   //!< Mouse Y axis
        RC_HandleData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8); //!< Mouse Z axis

        RC_HandleData.mouse.press_l = pData[12]; //!< Mouse Left Is Press ?
        RC_HandleData.mouse.press_r = pData[13]; //!< Mouse Right Is Press ?

        RC_HandleData.kb.key_code = pData[14] | (pData[15] << 8);                     //!< KeyBoard value
        RC_HandleData.rc.ch[4]    = ((int16_t)pData[16]) | ((int16_t)pData[17] << 8); // 左上角滑轮

        RC_HandleData.rc.ch[0] -= RC_CH_VALUE_OFFSET;
        RC_HandleData.rc.ch[1] -= RC_CH_VALUE_OFFSET;
        RC_HandleData.rc.ch[2] -= RC_CH_VALUE_OFFSET;
        RC_HandleData.rc.ch[3] -= RC_CH_VALUE_OFFSET;
        RC_HandleData.rc.ch[4] -= RC_CH_VALUE_OFFSET;
        
        rc_deadline_limit(&RC_HandleData.rc.ch[0]);
        rc_deadline_limit(&RC_HandleData.rc.ch[1]);
        rc_deadline_limit(&RC_HandleData.rc.ch[2]);
        rc_deadline_limit(&RC_HandleData.rc.ch[3]);
        rc_deadline_limit(&RC_HandleData.rc.ch[4]);        
    }
    
    /// @brief 获取数据结构体指针
    /// @return 返回数据结构体指针
    RC_ctrl_t *DR16_c::GetDataStructPtr()
    {
        return &RC_HandleData;
    }

    /// @brief 获取dr16状态
    /// @return 返回dr16状态
    DT7_State_e DR16_c::GetDR16State()
    {
        return dt7_state_;
    }

    /// @brief 获取dr16指针
    /// @return 返回dr16指针
    DR16_c *DR16_c::GetClassPtr()
    {
        return DR16_c::dr16_;
    }

    /// @brief 失联时间归0
    void DR16_c::LostTimeSetZero()
    {
        lost_time_ = 0;
    }

    /// @brief 获取失联时间指针
    /// @return 返回失联时间指针
    uint32_t *DR16_c::GetLostTimePtr()
    {
        return &lost_time_;
    }
    
    /// @brief 失联处理
    void LostHandle()
    {
            DR16_n::DR16_c *dr16 =  DR16_n::DR16_c::dr16_->GetClassPtr();
            dr16->RC_HandleData.rc.ch[0] = 0;
            dr16->RC_HandleData.rc.ch[1] = 0;
            dr16->RC_HandleData.rc.ch[2] = 0;
            dr16->RC_HandleData.rc.ch[3] = 0;
            dr16->RC_HandleData.rc.ch[4] = 0;
            dr16->dt7_state_ = LOST;
            dr16->RC_HandleData.rc.s1 = RC_SW_DOWN;
            dr16->RC_HandleData.rc.s2 = RC_SW_DOWN;
    }

    /// @brief 遥控器死区限制(防抖);
    /// @param input 
    /// @return 
    void DR16_c::rc_deadline_limit(int16_t *input)
    {
        if (*input < this->dead_line_&& *input > -this->dead_line_) {
            *input = 0;
        }
    }

   /// @brief 回调函数，处理遥控器数据
   /// @param pdata 数组指针
   /// @param psize 输入大小
   void callback(uint8_t *pdata, uint16_t psize)
   {
     if(psize == RC_FRAME_LENGTH)
        {
            DR16_Online();
            DR16_c *dr16 = DR16_c::dr16_->GetClassPtr();
            dr16->DataHandle(pdata);
        }
    }


    
}
void DR16_Online(void)
{
    DR16_n::dr16_lost_->Online();
}