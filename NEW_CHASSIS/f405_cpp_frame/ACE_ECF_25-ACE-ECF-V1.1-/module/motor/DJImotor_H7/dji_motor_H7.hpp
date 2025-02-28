#ifndef __DJI_MOTOR_HPP
#define __DJI_MOTOR_HPP


#ifdef __cplusplus
extern "C" {
#endif
#include "stdint.h"
#include "string.h"

#ifdef __cplusplus
}

#define DJI_MOTOR_CNT 12

#include "Alg_PID.hpp"
#include "Motor_General_def.hpp"
#include "bsp_fdcan.hpp"


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
        uint16_t feedback_ecd;               // 反馈数据, 电机末端单圈编码, 0-8191,刻度总共有8192格
        float feedback_speed;                // 反馈数据, 电机末端实时转速, 单位rpm
        int16_t feedback_real_current;       // 反馈数据, 实际电流
        uint8_t feedback_temperature;        // 反馈数据, 温度 Celsius

        bool init_flag;             // 判断是否为首次接收到数据

        uint16_t init_ecd;          // 首次接收到的编码值
        uint16_t last_ecd;          // 上一次读取的编码器值
        uint16_t last_record_ecd;   // 上一次总累计编码值
        float angle_single_round;   // 单圈角度
        float total_angle;          // 总角度,注意方向
        float total_round;          // 总圈数,注意方向
        int32_t record_ecd;         // 总累计编码值, 注意方向
        float record_length;        // 输出轴总累计长度, 注意方向, 单位为: 毫米

        float speed_aps;            // 根据反馈值计算的角速度,单位为: 度/秒
        float angular_speed;        // 输出轴角速度,单位为: 度/秒
        float linear_speed;         // 输出轴线速度,单位为: 米/秒
    }DJIMotorMeasure_t;
    
    // dji电机堵转初始化结构体
    typedef struct
    {
        uint16_t block_times;
        uint8_t block_init_if_finish;
        float last_position;
        int16_t init_current;       // 朝某个方向初始化的电流值
    }DJIMotorBlockInit_t;

    /**
     * @brief DJI电机返回信息处理类
    */
    class DJIMotorMeasure_c
    {
        private:
        public:
            DJIMotorMeasure_t measure;
            float ecd2length;           // 编码值转长度, 单位为: 编码/毫米
            float radius;               // 电机输出轴连接轮子半径, 单位: 毫米;
            int16_t gear_Ratio;         //减速比
            int16_t lap_encoder;        //编码器单圈码盘值（8192=12bit）
            
            void MeasureClear();

            // DJIMotorMeasure_c(/* args */);
            // ~DJIMotorMeasure_c();

            // int32_t GetRecordEcdVal(void);
            // float GetRecordLengthVal(void);

            // float GetAngSpeedVal(void);
            // float GetlinSpeedVal(void);
            // float GetSpeedVal(void);

            // uint16_t GetFdSpeedVal(void);
            // uint16_t GetFdEcdVal(void);
            // int16_t GetFdCurrentVal(void);
            // uint8_t GetFdTempVal(void);
    };

    /**
     * @brief DJI电机发送信息类
    */
    class DJI_Motor_Instance
    {
    private:
        float set_length;           // 设定的长度值
        float set_speed;            // 设定的速度值  

        float dt; // 反馈时间间隔, 单位秒
    public:
        // 分组发送设置
        uint8_t  sender_group;
        uint8_t  message_num;
        uint32_t feed_cnt;
        int16_t  give_current;       // 发送给电调的电流值

        float *other_speed_feedback_ptr_ = NULL;
        float *other_angle_feedback_ptr_ = NULL; // 其他反馈来源的反馈数据指针
        float *speed_feedforward_ptr_ = NULL;   // 速度前馈数据指针,可以通过此指针设置速度前馈值,或LQR等时作为速度状态变量的输入
        float *current_feedforward_ptr_ = NULL; // 电流前馈指针

        DJIMotorBlockInit_t block_val;
        DJI_Motor_n::DJIMotorMeasure_c MotorMeasure;
        Motor_General_Def_n::Motor_Controller_c motor_controller;
        BSP_CAN_Part_n::FDCANInstance_c motor_fdcan_instance;   // 电机CAN实例
        Motor_General_Def_n::Motor_Control_Setting_s motor_settings;
        Motor_General_Def_n::Motor_Working_Type_e stop_flag; // 启停标志
        Motor_General_Def_n::Motor_Type_e motor_type;        // 电机类型
        DJI_Motor_Instance(Motor_General_Def_n::Motor_Init_Config_s config);
        // ~DJI_Motor_Instance();
        // 静态变量
        static uint8_t dji_motor_idx; // 全局电机索引,一共注册了多少个电机
        static DJI_Motor_Instance *dji_motor_instance_p[DJI_MOTOR_CNT];
        static uint8_t sender_enable_flag[9];
        
        void DJIMotorSetRef(float ref);
        void DJIMotorBlockInitAchieve(void);
        void DJIMotorStop();
        void DJIMotorEnable();
        void DJIMotorLock();
        // void DJIMotorOuterLoop(Motor_General_Def_n::Closeloop_Type_e outer_loop);
        // void DJIMotorInit(Motor_General_Def_n::Motor_Init_Config_s config);
        uint8_t DJIMotorBlockInit(int16_t init_current);
        void MotorSenderGrouping();
        float Get_DJIMotor_dt(void);
        void Update_DJIMotor_dt(float new_dt);
    };
    void DJIMotorControl();
}

#endif


#endif // !__DJI_MOTOR_HPP
