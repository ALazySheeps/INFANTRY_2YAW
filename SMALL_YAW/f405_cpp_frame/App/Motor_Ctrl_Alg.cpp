#include "Motor_Ctrl_Alg.hpp"

//双环pid计算器
MotorCalc_c::MotorCalc_c(float OutLimit)
:
filter(1),
K0(K0),
K1(K1),
lqr(2, 1, new float[2]{0.5f, 1.0f}),
OnlyIpid(),
OutLimit(abs(OutLimit))
{
    
}

//LQR计算器
MotorCalc_c::MotorCalc_c(float K0, float K1, float _Out_Limit)
:
filter(1),
K0(K0),
K1(K1),
lqr(2, 1, new float[2]{K0, K1}),
OnlyIpid(),
OutLimit(abs(_Out_Limit))
{

}

//LQR计算器加i积分
MotorCalc_c::MotorCalc_c(float K0, float K1, float _Out_Limit, pid_alg_n::PID_Init_Config_t OnlyI_PidInit)
:
filter(1),
K0(K0),
K1(K1),
lqr(2, 1, new float[2]{K0, K1}),
OnlyIpid(OnlyI_PidInit),
OutLimit(abs(_Out_Limit))
{
    using_only_i = true;
}

//设置LQR参数矩阵
void MotorCalc_c::LQR_Set_K(float K0, float K1)
{
    this->K0 = K0;
    this->K1 = K1;
    lqr.lqr_data_.k[0] = K0;
    lqr.lqr_data_.k[1] = K1;
}

/**
 * @brief 传入计算器相关数据指针
 * @param SetPosVal 设定位置
 * @param ActuallPosVal 实际位置
 * @param ActuallSpeed  实际速度
 * @note  LQR计算器自动计算差值
 */
void MotorCalc_c::GetValPoint(const float *SetPosVal, const float *ActuallPosVal, const float *ActuallSpeed)
{
    this->SetPosVal = SetPosVal;
    this->ActuallPosVal = ActuallPosVal;
    this->ActuallSpeed = ActuallSpeed;
}

/**
 * @brief 传入计算器相关数据指针
 * @param PosClose 设定位置差值
 * @param ActuallSpeed  实际速度
 * @note  LQR将读取差值计算
 */
void MotorCalc_c::GetValPoint(const float *PosClose, const float *ActuallSpeed)
{
    this->PosClose = PosClose;
    this->ActuallSpeed = ActuallSpeed;
}


/**
 * @brief 调用LQR计算器
 */
float MotorCalc_c::LQR_Calc()
{
    lqr.lqr_data_.k[0] = K0;
    lqr.lqr_data_.k[1] = K1;

    float Distance;
    if (PosClose != nullptr)
    {
        Distance = *PosClose;
    }
    else
    {
        Distance = *SetPosVal - *ActuallPosVal;
    }

    //位置差 速度
    //k0     k1 阻尼
    float system_state_2[2] = {Distance, *ActuallSpeed};
    lqr.ECF_LQR_Data_Update(system_state_2);
    LqrOut = lqr.ECF_LQR_Calculate();
    
    if (using_only_i)
        PidOut = OnlyIpid.ECF_PID_Calculate(Distance, 0);
    float Out = LqrOut + PidOut;
    if (Out > OutLimit)         Out = OutLimit;
    else if (Out < -OutLimit)   Out = -OutLimit;

    return filter.first_order_filter(Out);
}

user_maths_c motor_maths;

//IMU根据电机反馈做出限位
void IMU_AngleLimit(float motor_coord_min,float motor_coord_max,
    float real_motor_angle,float real_imu_angle,float* imu_min_limit,float* imu_max_limit)
{
    float temp;
    temp=real_imu_angle-real_motor_angle;
    *imu_min_limit=temp+motor_coord_min;
    *imu_max_limit=temp+motor_coord_max;
}

//6020参数转换,力矩转换成电压
float torque_to_voltage_6020(float torque)
{
	float voltage = 0.0f;
		
	float current = torque / 0.741f * 10000;
	voltage = (current - 128.3507f) / 0.7778f;
	voltage = motor_maths.user_val_limit(voltage,-25000,25000);
	
	return voltage;
		
}
