/**
 *****************************东莞理工学院ACE实验室 *****************************
 * @file bsp_PWM.cpp/hpp
 * @brief 对舵机、好盈电调、WS2812B有专门的控制函数，方便调用
 * @author 胡炜
 * @note  
 * @version v1.0 2024/9/7 将启动程序合并到构造函数中。
 *          v2.0 2024/10/8 添加了比较中断，定时器溢出中断，DMA通道传输
 *
 * @example
 * 1、初始化：这里提供两种初始化方式，一种是通过构造函数，第二种则是主动调用，便于pwm类定义在其他类中
 * 2、CubeMX如果未配置好psc，arr可以调用函数进行配置
 * 3、如果用于好盈电调，初始化会自动调用其启动程序
 * void compare_callback(){}

    pwm_init_t init{
        .htim = &htim2,
        .channel = TIM_CHANNEL_1,
        .pwm_mode = PWM_COMPARE_IT | PWM_DMA_SEND,
        .pwm_overflow_callback_ptr = nullptr,
        .pwm_compare_callback_ptr = compare_callback,
        .is_HobbyWing = false
    };

    void k()
    {
        bsp_pwm_c example(init);
        example.ECF_PWM_Set_Arr(2000);
    }
 * 
 @endverbatim
 *****************************东莞理工学院ACE实验室 *****************************
 */



#include "bsp_PWM.hpp"
#include "cmsis_os.h"

using namespace bsp_pwm_n;

static bsp_pwm_c* pwm_instance_head = nullptr;  //指向类的链表



/**
 * @brief 构造函数，开启PWM通道
 * @param pwm_init 初始化结构体
 */
bsp_pwm_c::bsp_pwm_c(pwm_init_t pwm_init)
{
    //合法性检查,需要添加USE_FULL_ASSERT宏定义并在main中实现assert_failed();
    assert_param(IS_TIM_INSTANCE(pwm_init.htim->Instance));
    assert_param(IS_TIM_CCX_INSTANCE(pwm_init.htim, pwm_init.channel));

    this->htim_ = pwm_init.htim;
    this->channel_ = pwm_init.channel;
    ECF_PWM_Set_CCR(pwm_init.first_value);
    HAL_TIM_PWM_Start(htim_,channel_);

    this->pwm_mode_ = pwm_init.pwm_mode;
    if(this->pwm_mode_ & PWM_COMPARE_IT)
        this->pwm_compare_callback_ptr_ = pwm_init.pwm_compare_callback_ptr;
    if(this->pwm_mode_ & PWM_OVERFLOW_IT)
        this->pwm_overflow_callback_ptr_ = pwm_init.pwm_overflow_callback_ptr;
    //开启中断
    if(this->pwm_mode_ & PWM_OVERFLOW_IT || this->pwm_mode_ & PWM_COMPARE_IT)
        HAL_TIM_Base_Start_IT(this->htim_);  
    if(this->pwm_mode_ & PWM_DMA_SEND)
        HAL_TIM_PWM_Stop_DMA(this->htim_,this->channel_);

    this->is_HobbyWing_ = pwm_init.is_HobbyWing;

    if(pwm_instance_head == nullptr)
    {
        pwm_instance_head = this;
    }
    else
    {
        bsp_pwm_c* last_ptr = pwm_instance_head;
        while(last_ptr->instance_next_ != nullptr)
        {
            last_ptr = last_ptr->instance_next_;
        }
        last_ptr->instance_next_ = this;
    }

    //好盈电调启动程序
    //原理：油门拉最高然后拉最低
    if(is_HobbyWing_)
    {
        vTaskDelay(4000);
        __HAL_TIM_SetCompare(this->htim_, this->channel_, 2000);
        vTaskDelay(4000);
        __HAL_TIM_SetCompare(this->htim_, this->channel_, 1000);
        vTaskDelay(4000);
    }


}

bsp_pwm_c::bsp_pwm_c(){}

/**
 * @brief pwm初始化函数（如果调用了带参构造就不用这个了）
 * @param pwm_init 初始化结构体
 * @return None
 */
void bsp_pwm_c::ECF_PWM_Init(pwm_init_t pwm_init)
{
    //合法性检查,需要添加USE_FULL_ASSERT宏定义并在main中实现assert_failed();
    assert_param(IS_TIM_INSTANCE(pwm_init.htim->Instance));
    assert_param(IS_TIM_CCX_INSTANCE(pwm_init.htim, pwm_init.channel));

    this->htim_ = pwm_init.htim;
    this->channel_ = pwm_init.channel;
    ECF_PWM_Set_CCR(pwm_init.first_value);
    HAL_TIM_PWM_Start(htim_,channel_);

    this->pwm_mode_ = pwm_init.pwm_mode;
    if(this->pwm_mode_ & PWM_COMPARE_IT)
        this->pwm_compare_callback_ptr_ = pwm_init.pwm_compare_callback_ptr;
    if(this->pwm_mode_ & PWM_OVERFLOW_IT)
        this->pwm_overflow_callback_ptr_ = pwm_init.pwm_overflow_callback_ptr;
    //开启中断
    if(this->pwm_mode_ & PWM_OVERFLOW_IT || this->pwm_mode_ & PWM_COMPARE_IT)
        HAL_TIM_Base_Start_IT(this->htim_);  
    if(this->pwm_mode_ & PWM_DMA_SEND)
        HAL_TIM_PWM_Stop_DMA(this->htim_,this->channel_);

    this->is_HobbyWing_ = pwm_init.is_HobbyWing;

    if(pwm_instance_head == nullptr)
    {
        pwm_instance_head = this;
    }
    else
    {
        bsp_pwm_c* last_ptr = pwm_instance_head;
        while(last_ptr->instance_next_ != nullptr)
        {
            last_ptr = last_ptr->instance_next_;
        }
        last_ptr->instance_next_ = this;
    }

    //好盈电调启动程序
    //原理：油门拉最高然后拉最低
    if(is_HobbyWing_)
    {
        vTaskDelay(4000);
        __HAL_TIM_SetCompare(this->htim_, this->channel_, 2000);
        vTaskDelay(4000);
        __HAL_TIM_SetCompare(this->htim_, this->channel_, 1000);
        vTaskDelay(4000);
    }
}

/**
 * @brief DMA数据发送
 * @return None
 */
void bsp_pwm_c::ECF_PWM_DMA_SendArray(uint8_t *buf, uint16_t len)
{
    if(this->pwm_mode_ & PWM_DMA_SEND)
    {
        HAL_TIM_PWM_Start_DMA(this->htim_, this->channel_, (uint32_t*)buf, len);
    }
}


/**
 * @brief 50HZ输出频率百分比调整函数(普通舵机)
 * @param[in] Output_Percent（0~100）
 * @return None
 */
void bsp_pwm_c::ECF_PWM_50HZ_Output(uint16_t Output_Percent)
{
    if (Output_Percent > 100) {
        return;
    }
    int ccr = Output_Percent*2000/100+500;
    __HAL_TIM_SetCompare(this->htim_, this->channel_, ccr);
}




/**
 * 好盈电调控制函数
 * 注意pwm控制范围在 1200到1400（我怕爆了）
 * @param pwm
 */
void bsp_pwm_c::ECF_HobbyWing_ESC_Control(uint16_t pwm) 
{
    if(this->is_HobbyWing_)
    {
        //别超油门了
        if (pwm >= 1400) pwm = 1300;
        else if(pwm <= 1190)  pwm=1140;
        __HAL_TIM_SetCompare(htim_, channel_, pwm);
    }
}

/**
 * @brief 设置定时器的预分频值
 * @param psc:预分频值
 */
void bsp_pwm_c::ECF_PWM_Set_PSC(uint16_t psc) { __HAL_TIM_PRESCALER(this->htim_,psc); }

/**
 * @brief 设置定时器的最大计数值
 * @param psc:预分频值
 */
void bsp_pwm_c::ECF_PWM_Set_Arr(uint16_t arr) { __HAL_TIM_SetAutoreload(this->htim_,arr); }

/**
 * @brief 设置定时器的比较值
 * @param ccr:比较值
 */
void bsp_pwm_c::ECF_PWM_Set_CCR(uint16_t ccr){ __HAL_TIM_SetCompare(this->htim_, this->channel_, ccr); }

//比较中断(DMA传输完成中断)
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    bsp_pwm_c* temp_ptr = pwm_instance_head;
    while(temp_ptr != nullptr)
    {
        if(temp_ptr->htim_ == htim && temp_ptr->pwm_mode_ & PWM_COMPARE_IT)
        {
            temp_ptr->pwm_compare_callback_ptr_();
            break;
        }
        if(temp_ptr->pwm_mode_ & PWM_DMA_SEND)
        {
            HAL_TIM_PWM_Stop_DMA(temp_ptr->htim_,temp_ptr->channel_);
        }
        temp_ptr = temp_ptr->instance_next_;
    }
}

//溢出中断
void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *htim)
{
    bsp_pwm_c* temp_ptr = pwm_instance_head;
    while(temp_ptr != nullptr)
    {
        if(temp_ptr->htim_ == htim && temp_ptr->pwm_mode_ & PWM_OVERFLOW_IT)
        {
            temp_ptr->pwm_overflow_callback_ptr_();
            break;
        }
        temp_ptr = temp_ptr->instance_next_;
    }
}

