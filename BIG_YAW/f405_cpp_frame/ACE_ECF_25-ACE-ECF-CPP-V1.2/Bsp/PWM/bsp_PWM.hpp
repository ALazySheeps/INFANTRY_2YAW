#ifndef __BSP_PWM_H
#define __BSP_PWM_H


#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "tim.h"

#ifdef __cplusplus
}

namespace bsp_pwm_n{

// 回调
    using pwm_overflow_callback = void (*)();                    // 计数器溢出中断回调函数
    using pwm_compare_callback = void (*)();                     // 达到比较值触发回调函数

    typedef enum{
        PWM_NORMAL = 0x00,         
        PWM_OVERFLOW_IT = 0x01,   //定时器溢出中断
        PWM_COMPARE_IT = 0x02,    //比较中断
        PWM_DMA_SEND = 0x04,      //DMA发送
    }pwm_mode_e;

    typedef struct{
        TIM_HandleTypeDef *htim;
        uint32_t channel;
        uint8_t pwm_mode;                                        //pwm模式(可选多种)
        pwm_overflow_callback pwm_overflow_callback_ptr;         //溢出中断（没有给null）
        pwm_compare_callback pwm_compare_callback_ptr;           //比较中断（没有给null）
        uint8_t is_HobbyWing;                                    //是否用于好盈电调控制
        uint16_t first_value = 0; //初始值设置
    }pwm_init_t;
        


    class bsp_pwm_c{
    public:
        bsp_pwm_c(pwm_init_t pwm_init);
        bsp_pwm_c();
        void ECF_PWM_Init(pwm_init_t pwm_init);
        void ECF_PWM_50HZ_Output(uint16_t Output_Percent);
        void ECF_PWM_Set_PSC(uint16_t psc);
        void ECF_PWM_Set_Arr(uint16_t arr);
        void ECF_PWM_Set_CCR(uint16_t ccr);
        void ECF_PWM_DMA_SendArray(uint8_t *buf, uint16_t len);
        pwm_overflow_callback pwm_overflow_callback_ptr_ = nullptr;         //溢出中断（没有给null）
        pwm_compare_callback pwm_compare_callback_ptr_ = nullptr;           //比较中断（没有给null）
        //好盈电机
        void ECF_HobbyWing_ESC_Control(uint16_t pwm);
        bsp_pwm_c* instance_next_;
    //private:
        TIM_HandleTypeDef *htim_;
        uint32_t channel_;
        uint8_t is_HobbyWing_;
        uint8_t pwm_mode_;
    };



};

#endif

#endif
