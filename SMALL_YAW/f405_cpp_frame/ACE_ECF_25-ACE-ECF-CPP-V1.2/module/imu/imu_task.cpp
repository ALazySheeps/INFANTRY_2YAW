#include "BMI088driver.hpp"
#include "imu_task.hpp"
#include "task.h"

void IMU_Task(void const * argument)
{
    
    while (1)
    {
        BMI088Instance_c::BMI_UpData();
        vTaskDelay(1);
    }

}
