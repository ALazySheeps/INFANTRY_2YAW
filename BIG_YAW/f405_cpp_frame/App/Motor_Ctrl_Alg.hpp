#ifndef MOTOR_CALC_HPP
#define MOTOR_CALC_HPP

#include "lqr_alg.hpp"
#include "Alg_PID.hpp"
#include "filter.hpp"


//lqr计算类，加入积分补偿
class MotorCalc_c
{
public:
    bool using_only_i = false;
    float K0,K1;
    float OutLimit;
    lqr_alg_n::lqr_alg_c lqr;   // LQR控制器
    pid_alg_n::pid_alg_c OnlyIpid;  // 积分项PID

    const float *PosClose = nullptr;      // 设定位置 与 实际位置 差
    const float *SetPosVal;     // 设定位置
    const float *ActuallPosVal; // 目前位置
    const float *ActuallSpeed;  // 目前速度
    
    float LqrOut = 0;
    float PidOut = 0;
    filter_alg_n::first_order_filter_c filter;
public:
    MotorCalc_c(float Out_Limit);
    MotorCalc_c(float K0, float K1, float Out_Limit);//LQR计算器
    MotorCalc_c(float K0, float K1, float Out_Limit, pid_alg_n::PID_Init_Config_t OnlyI_PidInit);//LQR计算器加i积分
    void  LQR_Set_K(float K0, float K1);
    float LQR_Calc();
    void GetValPoint(const float *SetPosVal, const float *ActuallPosVal, const float *ActuallSpeed);//传入设定值与实际值，LQR计算会自动计算差值
    void GetValPoint(const float *PosClose, const float *ActuallSpeed);//传入差值
};

//IMU根据电机反馈做出限位
void IMU_AngleLimit(float motor_coord_min,float motor_coord_max,
    float real_motor_angle,float real_imu_angle,float* imu_min_limit,float* imu_max_limit);
//6020参数转换,力矩转换成电压
float torque_to_voltage_6020(float torque);

#endif
