/*************************** Dongguan-University of Technology -ACE**************************
 * @file   servo.cpp
 * @author  KazuHa12441
 * @version V1.4
 * @date    2024/10/24
 * @brief   舵机控制
 *
 * @todo: 补充
 *
 ********************************************************************************************/
#include "servo.hpp"

namespace Servo_n
{   
    /// @brief 舵机构造
    /// @param pwm pwm结构体
    /// @param type 舵机类型
    Servo_c::Servo_c(bsp_pwm_n::bsp_pwm_c *pwm,Servo_Type_e type):PWM_(pwm),servo_type_(type)
    {

        switch(type)
        {
            case ANGLE180:
            {
                anglemax = 180;
                break;
            }
            case ANGLE270:
            {
                anglemax = 270;
                break;
            }
            case ANGLE360:
            {
                anglemax = 360;
                break;
            }
            default:
            {
                while (1);
            }
        }
    }
    

    /// @brief 设置角度
    /// @param angle 舵机角度
    void Servo_c::SetAngle(uint16_t angle)
    {
        uint16_t persent;
        if(angle <= anglemax)
        {
        persent = angle/anglemax;
        now_angel_ = angle;
        PWM_->ECF_PWM_50HZ_Output(persent);
        }
    }
}