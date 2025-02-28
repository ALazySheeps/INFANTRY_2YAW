#ifndef __GIMBAL_TASK_HPP
#define __GIMBAL_TASK_HPP

extern "C"
{
    #include "FreeRTOS.h"
    #include "task.h"
    #include "Vision.h"
    void Gimbal_Task(void const * argument);
}

#include "gimbal_ctrl.hpp"
#include "bsp_dwt.hpp"
#include "dr16.hpp"
#include "BMI088driver.hpp"
#include "dji_motor.hpp"



#define BIGYAW_ID 0x224
#define SMALLYAW_ID 0x411
#define CHASSIS_ID 0x112



#endif