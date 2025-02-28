/*************************** Dongguan-University of Technology -ACE**************************
 * @file    safe_task.c
 * @author  zhengNannnn
 * @version V1.0
 * @date    2023/11/5
 * @brief
 ******************************************************************************
 * @verbatim
 *  安全任务函数，支持用户自定义名称、失联检测时间、失联回调函数
 *  使用方法：
 *      申请一个安全任务，丢入名字及自定义失联回调函数
 *      类似与喂狗机制，在收到消息时刷新在线状态
 *      当达到失联阈值时执行自定义任务
 *  demo：
 *       //创建安全任务
 *       osThreadDef(SAFE_TASK, Safe_Task, osPriorityHigh, 0, 128);
 *		 Chassis_TASKHandle = osThreadCreate(osThread(Chassis_task), NULL);
 *       
 *       void online(void){enable米狗电机}
 *       void disconnect(void){unable米狗电机}
 *       //申请安全任务
 *       Safe_task_c demo_safe_task("demo",10,disconnect,online);
 *       demo_safe_task.online();
 * @attention
 *       好用爱用
 * @version           time
 * v1.0   基础版本
 * v2.0   C++升级版本  2024-8-27
 ************************** Dongguan-University of Technology -ACE***************************/
#include "safe_task.hpp"
#include "dr16.hpp"
#include "real_main.hpp"
extern "C"{
    #include "safe_task.h"
    #include <string.h>
    #include <stdint.h>
    #include "bsp_dwt.hpp"
    #include "FreeRTOS.h"
    #include "task.h"
}




Safe_task_c* Head = nullptr;

/*任务间采用链表*/
static bool dwt_init = false;




Safe_task_c::Safe_task_c(const char* name, uint64_t Discon_ms, Callback DisconnetCallBack, Callback OnlineCallback)
{
    Safe_task_c* last_task_ptr = Head;
    //写入相关参数
    strcpy(this->name, name);
    this->Disconnection_threshold = Discon_ms;
    this->Disconnection_ms = 0;
    this->Disconnection_falg = false;
    this->First_Disconnect = true;
    this->OnlineCallBack = OnlineCallback;
    this->DisconnetCallBack = DisconnetCallBack;
    this->Last_online_time_ms = 0;
    this->next_task = nullptr;
    //链表尾插法
    if(Head == nullptr)
    {
        Head = this;
        return;
    }        
    while(last_task_ptr->next_task != nullptr)     last_task_ptr = last_task_ptr->next_task;
    last_task_ptr->next_task = this;
}
/**
 * @brief 刷新在线状态
 * @note  若有注册在线回调，则会运行
 */
void Safe_task_c::Online(void)
{
    BSP_DWT_n::BSP_DWT_c *dwt = BSP_DWT_n::BSP_DWT_c::ECF_Get_DwtInstance(); 
    if (this->OnlineCallBack != nullptr)    this->OnlineCallBack();
    this->Last_online_time_ms = dwt->ECF_DWT_GetTimeline_ms();
    this->Disconnection_ms = 0;
    this->Disconnection_falg = false;
    this->First_Disconnect = true;
}
/**
 * @brief 失联时间计算
 */
void Safe_task_c::Calc_Disconnection_time(void)
{
    BSP_DWT_n::BSP_DWT_c *dwt = BSP_DWT_n::BSP_DWT_c::ECF_Get_DwtInstance(); 
    this->Disconnection_ms = dwt->ECF_DWT_GetTimeline_ms()-this->Last_online_time_ms;
    if (this->Disconnection_ms < 0) this->Disconnection_ms = 0;
    if (this->Disconnection_ms > this->Disconnection_threshold) this->Disconnection_falg = true;
}
/**
 * @brief 若有掉线失联，执行失联函数
 * @note  失联函数仅执行一次
 */
void Safe_task_c::Doing_DisconnetCallBack(void)
{
    if (this->Disconnection_falg)   
    {
        if (this->First_Disconnect) this->DisconnetCallBack();
        this->First_Disconnect = false;
    }
}

/**
 * @brief 丢到freertos任务
 */
void Safe_Task(void const *argument)
{
	while(1)
	{
        //链表轮询
        Safe_task_c* temp_task_ptr = Head;
        while(temp_task_ptr != nullptr)
        {
            temp_task_ptr->Calc_Disconnection_time();
            temp_task_ptr->Doing_DisconnetCallBack();
            temp_task_ptr = temp_task_ptr->next_task;
        }
        vTaskDelay(1);
	}
}

