#ifndef __SERVO_HPP
#define __SERVO_HPP

#include "bsp_PWM.hpp"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

namespace Servo_n
{

   typedef enum
   {
    ANGLE180,
    ANGLE270,
    ANGLE360
   }Servo_Type_e;

   class Servo_c
   {
    private: 
    bsp_pwm_n::bsp_pwm_c *PWM_;
    Servo_Type_e servo_type_ = ANGLE180;
    float now_angel_;
    uint16_t anglemax;
    public:
    Servo_c(bsp_pwm_n::bsp_pwm_c *pwm,Servo_Type_e type);
    void SetAngle(uint16_t angle);
   };
}



#endif /*__SERVO_HPP*/