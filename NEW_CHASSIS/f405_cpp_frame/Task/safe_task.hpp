#ifndef __TASK_SAFE_HPP  //如果未定义
#define __TASK_SAFE_HPP  //那么定义
#include <stdlib.h>
#include <stdint.h>
#include <functional>
// typedef void (*Callback)(void);
using Callback = std::function<void(void)>;
class Safe_task_c
{
    public:
    char   name[50];                 //任务名称
    uint64_t  Disconnection_ms;      //失联计数器
    uint64_t  Disconnection_threshold;  //失联阈值
    bool   Disconnection_falg;       //失联标志位
    bool   First_Disconnect;         //首次失联标志位，防止多次进入失联回调函数
    Safe_task_c(const char* name, uint64_t Discon_ms, Callback DisconnetCallBack);
    Safe_task_c(const char* name, uint64_t Discon_ms, Callback DisconnetCallBack, Callback OnlineCallback);
    void Online(void);
    void Calc_Disconnection_time(void);
    void Doing_DisconnetCallBack(void);
    Safe_task_c *next_task = nullptr;
    private:
    uint64_t Last_online_time_ms = 0;
    Callback DisconnetCallBack;    //用户自定义失联回调函数
    Callback OnlineCallBack;
    // void (*DisconnetCallBack)(void);    //用户自定义失联回调函数
    // void (*OnlineCallBack)(void); 

};


#endif