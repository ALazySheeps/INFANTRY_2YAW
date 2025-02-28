#include "bigyaw_ctrl.hpp"
#include "gimbal_task.hpp"


void Gimbal_Task(void const *argument)
{

    for(;;)
    {
        big_yaw_ctrl_c::Get_big_yaw_instance()->Big_Yaw_ctrl_loop();
      
        vTaskDelay(1);
    }
}




