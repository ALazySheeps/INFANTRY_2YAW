#include "fire_task.hpp"





void Fire_Task(void const * argument)
{

    for(;;)
    {
        //taskENTER_CRITICAL();
        
        //taskEXIT_CRITICAL();
        vTaskDelay(1);
    }
}


