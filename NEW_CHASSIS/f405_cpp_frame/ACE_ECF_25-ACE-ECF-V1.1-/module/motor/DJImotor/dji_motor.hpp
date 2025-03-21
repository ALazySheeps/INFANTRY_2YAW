#ifndef __DJI_MOTOR_HPP
#define __DJI_MOTOR_HPP

#ifdef __cplusplus
extern "C"
{
#endif
#include "stdint.h"
#include "string.h"

#ifdef __cplusplus
}

#define DJI_MOTOR_CNT 12

#include "real_main.hpp"
#include "bsp_can.hpp"
#include "Alg_PID.hpp"
#include "Motor_General_def.hpp"
#include "safe_task.hpp"

#ifdef STM32H723xx
#include "bsp_fdcan.hpp"
#else
#include "bsp_can.hpp"
#endif // USE_H7_if_or_not

#define DJI_MOTOR_CNT 12

/* 滤波系数设置为1的时候即关闭滤波 */
#define SPEED_SMOOTH_COEF 0.85f      // 最好大于0.85
#define CURRENT_SMOOTH_COEF 0.9f     // 必须大于0.9
#define ECD_ANGLE_COEF_DJI 0.043945f // (360/8192),将编码器值转化为角度制
namespace DJI_Motor_n
{
    /* DJI电机CAN反馈信息*/
    typedef struct
    {
        uint16_t feedback_ecd; // 反馈数据, 电机末端单圈编码, 0-8191,刻度总共有8192格(绝对编码值)
        float feedback_speed;  // 反馈数据, 电机末端实时转速, 单位rpm
        // 这里把实际电流值改成了浮点数 int32_t->float
        int16_t feedback_real_current; // 反馈数据, 实际电流
        float feedback_real_current_to_pid;

        uint8_t feedback_temperature; // 反馈数据, 温度 Celsius
        bool init_flag;               // 判断是否初始化
        uint16_t init_ecd;            // 首次接收到的编码值
        uint16_t last_ecd;            // 上一次读取的编码器值
        uint16_t last_record_ecd;     // 上一次总累计编码值
        float angle_single_round;     // 单圈角度
        float total_angle;            // 总绝对角度,注意方向
        float total_round;            // 总圈数（绝对）,注意方向

        float relative_angle;         // 单圈相对角度（考虑了零偏）,注意方向
        // 这里把累计编码值改成了浮点数 int32_t->float
        float record_ecd; // 总累计编码值, 注意方向
        float record_ecd_to_pid;

        float record_length; // 输出轴总累计长度, 注意方向, 单位为: 毫米
        float speed_aps;     // 根据反馈值计算的角速度,单位为: 度/秒
        float angular_speed; // 输出轴角速度,单位为: 度/秒
        float linear_speed;  // 输出轴线速度,单位为: 米/秒

        float zero_offset; // 电机零偏(单位：角度)
    } DJIMotorMeasure_t;

    /**
     * @brief DJI电机返回信息处理类
     */
    class DJIMotorMeasure_c
    {
    private:
    public:
        DJIMotorMeasure_t measure;
        float ecd2length;    // 编码值转长度, 单位为: 编码/毫米
        float radius;        // 电机输出轴连接轮子半径, 单位: 毫米;
        int16_t gear_Ratio;  // 减速比
        int16_t lap_encoder; // 编码器单圈码盘值（8192=12bit）
        void MeasureClear();
    };

    /**
     * @brief DJI电机发送信息类
     */
    class DJI_Motor_Instance
    {
    public: //private:
        float set_length; // 设定的长度值
        float set_speed;  // 设定的速度值
        float dt;         // 反馈时间间隔, 单位秒

        bool is_deline = false;  //电机掉线标志位
    public:
        Safe_task_c *lost_detection_dji;
        // 大疆电机自定义限幅
        int16_t Current_limit_H_3508_6020 = 16384;
        int16_t Current_limit_L_3508_6020 = -16384;
        int16_t Current_limit_H_2006 = 10000;
        int16_t Current_limit_L_2006 = -10000;
        // 分组发送设置
        uint8_t sender_group;
        uint8_t message_num;
        uint32_t feed_cnt;
        int16_t set;                             // 电机控制CAN发送设定值
        int16_t give_current;                    // 目前这东西发送给电调的电流值(该变量只用在电流环)
        float pid_out;                           // pid总的输出值，float形式
        float *other_speed_feedback_ptr_ = NULL; // 其他反馈来源的反馈数据指针
        float *other_angle_feedback_ptr_ = NULL; // 其他反馈来源的反馈数据指针
        DJI_Motor_n::DJIMotorMeasure_c MotorMeasure;
        Motor_General_Def_n::Motor_Controller_c motor_controller;
        BSP_CAN_Part_n::CANInstance_c motor_can_instance; // 电机CAN实例
        Motor_General_Def_n::Motor_Control_Setting_s motor_settings;
        Motor_General_Def_n::Motor_Working_Type_e stop_flag; // 启停标志
        Motor_General_Def_n::Motor_Type_e motor_type;        // 电机类型

        // 静态变量
        static uint8_t dji_motor_idx; // 全局电机索引,一共注册了多少个电机
        static DJI_Motor_Instance *dji_motor_instance_p[DJI_MOTOR_CNT];
        static uint8_t sender_enable_flag[6];

        DJI_Motor_Instance(Motor_General_Def_n::Motor_Init_Config_s config);
        void DJI_Disconnect();
        bool Get_Lost_Status();
        void DJI_Online();
        void DJIMotorSetRef(float ref);
        void DJIMotorStop();
        void DJIMotorEnable();
        void DJIMotorSelfLock();
        void DJIMotorClearSelfLock(void);
        void DJIMotorOnlyRecevie(void);
        void DJIMotorOuterLoop(Motor_General_Def_n::Closeloop_Type_e outer_loop);
        void MotorSenderGrouping();
        float Get_DJIMotor_dt(void);
        void Update_DJIMotor_dt(float new_dt);
        // get set 方法
        // 设置大疆电机的自定义限幅
        void Set_Current_limit_H_3508_6020(int16_t high);
        void Set_Current_limit_L_3508_6020(int16_t low);
        void Set_Current_limit_H_2006(int16_t high);
        void Set_Current_limit_L_2006(int16_t low);
        // 设置其他反馈
        void Set_ANGLE_PID_Motor_feedback(float *ptr);
        void Set_ANGLE_PID_other_feedback(float *other_ptr);
        void Set_SPEED_PID_other_feedback(float *other_ptr);
        // 获取电机的实际返回值
        DJIMotorMeasure_t Get_Motor_measure();
        // 获取电机最终的PID结果
        float Get_Motor_Final_PidOut();
        int16_t Get_Motor_Give_current();
        // 电机输出只用我自己设置的，不要PID计算结果
        void SetDJIMotorOutputFix(int16_t output);
    };
    void DJIMotorControl();
}

#endif


#endif // !__DJI_MOTOR_HPP

/* ------------------------------------------------------------------------------------------这是一条分割线------------------------------------------------------------------------------------------------------- */
