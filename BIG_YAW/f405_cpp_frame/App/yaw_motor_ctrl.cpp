#include "bigyaw_ctrl.hpp"

yaw_motor_ctrl_c::yaw_motor_ctrl_c(uint8_t big_yaw_motor_id,uint8_t small_yaw_motor_id,float big_yaw_offset_angle,float small_yaw_offset_angle):
big_yaw_id_(big_yaw_motor_id),small_yaw_id_(small_yaw_motor_id),
bigyaw_motor(nullptr),smallyaw_motor(nullptr),
bigyaw_lqr_alg(nullptr),big_yaw_offset_angle_(big_yaw_offset_angle),small_yaw_offset_angle_(small_yaw_offset_angle)
{
    target_angle_ = 0;
}

/**
 * @brief 大yaw初始化
 */
void yaw_motor_ctrl_c::Yaw_init()
{
    Motor_General_Def_n::Motor_Init_Config_s big_yaw_motor_confit = {
        .controller_setting_init_config = { 
          .outer_loop_type       =  Motor_General_Def_n::OPEN_LOOP,
          .close_loop_type       =  Motor_General_Def_n::OPEN_LOOP,
          .motor_reverse_flag    =  Motor_General_Def_n::MOTOR_DIRECTION_NORMAL,
          .feedback_reverse_flag =  Motor_General_Def_n::FEEDBACK_DIRECTION_NORMAL,
          // 反馈来源可设置为 MOTOR_FEED OTHER_FEED SPEED_APS ANGULAR_SPEED LINEAR_SPEED
          .speed_feedback_source =  Motor_General_Def_n::MOTOR_FEED,
        },
        .motor_type = Motor_General_Def_n::GM6020,
        .can_init_config = { 
          .can_handle = &hcan2,
          .tx_id = big_yaw_id_,// 看电调闪几下就填几
        },
        .zero_offset = big_yaw_offset_angle_
    };

    Motor_General_Def_n::Motor_Init_Config_s small_yaw_motor_confit = {
        .motor_type = Motor_General_Def_n::GM6020,
        .can_init_config = { 
          .can_handle = &hcan2,
          .tx_id = small_yaw_id_,// 看电调闪几下就填几
        },
        .zero_offset = small_yaw_offset_angle_
    };

    bigyaw_motor = new DJI_Motor_n::DJI_Motor_Instance(big_yaw_motor_confit);
    smallyaw_motor = new DJI_Motor_n::DJI_Motor_Instance(small_yaw_motor_confit);

    bigyaw_lqr_alg = new MotorCalc_c(0,0,25000);



    bigyaw_motor->DJIMotorEnable();
    smallyaw_motor->DJIMotorOnlyRecevie();
}

/**
 * @brief 大yaw控制模式设置
 * @param gimbal_mode  云台模式
 * 
 */
void yaw_motor_ctrl_c::Yaw_set_mode(gimbal_behaviour_e gimbal_mode)
{
    gimbal_mode_ = gimbal_mode;
    static gimbal_behaviour_e last_gimbal_mode = gimbal_behaviour_e::GIMBAL_ZERO_FORCE;

    //设置无力状态
    if(gimbal_mode_ == gimbal_behaviour_e::GIMBAL_ZERO_FORCE)
    {
        big_yaw_motor_mode_ = big_yaw_motor_behaviour_e::BIG_YAW_ZERO_FORCE;
    }
    //切入自瞄或者取消自瞄，大Yaw复位
    else if(last_gimbal_mode != gimbal_mode_)
    {
        big_yaw_motor_mode_ = big_yaw_motor_behaviour_e::BIG_YAW_RESET;
        is_reset = false;
    }
    //自瞄模式下大Yaw锁定
    else if(gimbal_mode_ == gimbal_behaviour_e::GIMBAL_AUTOATTACK && is_reset)
    {
        big_yaw_motor_mode_ = big_yaw_motor_behaviour_e::BIG_YAW_LOCK;
    }
    //手描操控
    else if(gimbal_mode_ == gimbal_behaviour_e::GIMBAL_MANUAL && is_reset)
    {
        big_yaw_motor_mode_ = big_yaw_motor_behaviour_e::BIG_YAW_MANUAL;
    }

    last_gimbal_mode = gimbal_mode_;
}


/**
 * @brief 数据更新与设定
 * @param dr16_data  遥控器数据
 */
void yaw_motor_ctrl_c::Yaw_data_update(DR16_n::RC_ctrl_t &dr16_data)
{
    big_yaw_angle = user_maths_c().loop_fp32_constrain(bigyaw_motor->MotorMeasure.measure.relative_angle,-180.0f,180.0f);
    small_yaw_angle = user_maths_c().loop_fp32_constrain(smallyaw_motor->MotorMeasure.measure.relative_angle,-180.0f,180.0f);
    big_yaw_speed = bigyaw_motor->MotorMeasure.measure.feedback_speed;

    switch(big_yaw_motor_mode_)
    {
        case big_yaw_motor_behaviour_e::BIG_YAW_LOCK:
            target_angle_ = big_yaw_angle;
            break;
        case big_yaw_motor_behaviour_e::BIG_YAW_MANUAL:
            target_angle_-=(float)(dr16_data.rc.ch[0]*YAW_DPI);
            target_angle_-=(float)(dr16_data.mouse.x*YAW_DPI_MOUSE);
            break;
        case big_yaw_motor_behaviour_e::BIG_YAW_RESET:
            target_angle_ = small_yaw_angle;
            break;
        default: break;
    }
    target_angle_ = user_maths_c().loop_fp32_constrain(target_angle_,-180.0f,180.0f);

    //判断是否转到位
    if(abs(big_yaw_angle - small_yaw_angle) < 5 && big_yaw_speed < 300)
    {
        is_reset = true;
    }
}

/**
 * @brief Yaw轴无力设定
 */
void yaw_motor_ctrl_c::Yaw_set_zero_output()
{
    output = 0.0f;
}


/**
 * @brief lqr计算
 */
void yaw_motor_ctrl_c::Yaw_ctrl_data_caculate()
{
    float Yaw_motortarget,Yaw_distance;
    
    Yaw_motortarget = user_maths_c().float_min_distance(target_angle_, big_yaw_angle, -180.0f, 180.0f);
    Yaw_distance = (0.0f - Yaw_motortarget) / 57.295779513f;
    bigyaw_lqr_alg->GetValPoint(&Yaw_distance,&big_yaw_speed);
    output = torque_to_voltage_6020(bigyaw_lqr_alg->LQR_Calc());
}

/**
 * @brief 大yaw电机控制循环
 */
void yaw_motor_ctrl_c::Yaw_ctrl_loop(gimbal_behaviour_e gimbal_mode,DR16_n::RC_ctrl_t &dr16_data)
{
    Yaw_set_mode(gimbal_mode);
    Yaw_data_update(dr16_data);
    if(big_yaw_motor_mode_ == big_yaw_motor_behaviour_e::BIG_YAW_ZERO_FORCE)
    {
        Yaw_set_zero_output();
    }
    else
    {
        Yaw_ctrl_data_caculate();
    }

    bigyaw_motor->DJIMotorSetRef(output);
}

/**
 * @brief 获取输出值
 */
float yaw_motor_ctrl_c::Yaw_get_output() const
{
    return output;
}
