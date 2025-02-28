#ifndef __FIRE_CTRL_HPP
#define __FIRE_CTRL_HPP

extern "C"
{
    #include "Vision.h"
}

#include "fire_confit.hpp"
#include "dr16.hpp"
#include "dji_motor.hpp"
#include "Alg_PID.hpp"
#include "gimbal_ctrl.hpp"


class fire_ctrl_c{
    public:
    fire_ctrl_c();
    Visual_Tx_t     virtual_tx_data;  //发送视觉数据

    private:
    Visual_Rx_t*    virtual_rx_data;  //接收视觉数据
    DR16_n::RC_ctrl_t* dt7;//dt7遥控器数据

    DJI_Motor_n::DJI_Motor_Instance* motor_left_fire;
    DJI_Motor_n::DJI_Motor_Instance* motor_right_fire;
    DJI_Motor_n::DJI_Motor_Instance* motor_pluck_fire;

    //拨弹盘控制算法
    pid_alg_n::cascade_pid_c* pid_pluck;   //控制单发
    pid_alg_n::pid_alg_c* pid_fire;        //控制连发

    float* gimbal_pitch_dif;//获取pitch设定角度与实际角度差值
    float* gimbal_yaw_dif;//获取yaw设定角度与实际角度差值
    

    //真实物理量
    struct{
        float shoot_speed;  //  弹速
        float shoot_rate;   //  射频
        float shoot_hot;    //（预留）热量
    }Real_physical_Val;

    //设定物理量
    struct{
        float shoot_speed_set;//弹速
        float shoot_rate_set; //射频
    }Set_physical_Val;

    //电机给定值
    struct{
        float left_motor_speed_set;     //左摩擦轮速度设定值
        float right_motor_speed_set;    //右摩擦轮速度设定值
        float pluck_motor_auto_set;     //拨弹盘连发设定值
        float pluck_motor_semi_set;     //拨弹盘单发设定值
    }motor_set_value;
    
/**************************************发射状态指示*************************************************/
    struct{
        bool isFricReady;//是否开启了摩擦轮
        bool isStuck;    //是否卡弹
        bool isSemi;     //是否单发
        bool singlefire_state=0;  //子弹是否发送出去 0未发送 1发送
        fire_set_mode_e mode; //发射模式
        uint8_t fric_state=0;  //摩擦轮状态 遥控器：初始为0，上波一次为1，归中为2，再次上波为3，归中为0  键盘：初始为0，按下为1，松开为2，再次按下为3，松开为0
    }static_flag;
    
    filter_alg_n::recursive_ave_filter_c* predict_shoot_speed;  //递推平均滤波


/****************************************控制函数*************************************************/
    public:
    void Fire_Control_loop();                               //发射控制循环
    void Fire_Init();
    static fire_ctrl_c* Get_Fire_Instance();

    private:
    static fire_ctrl_c* Fire_Instance;
    void set_fric_zero();  //摩擦轮无力
    void set_fire_zero();  //拨弹盘无力
    void fric_set();       //遥控器控制摩擦轮开关
    void fire_mode_set();  //发射模式设置函数
    void shoot_single_control(fire_set_mode_e last_shoot_mode); //单发控制函数
    void shoot_auto_control();                                  //连发控制函数
    void Shoot_ZeroForce_Set();                                 //发射机构无力设置
    void Shoot_V_Get();
    void Shoot_Control_Get();
    void Shoot_Set_Val();                                   //发射机构位置速度环计算

/**************************************物理量转换*************************************************/
    //射速转换(rpm->m/s)
    static float convert_shootspd(int16_t rpm){return (float)(rpm*MOTORRPM_TO_FIRESPEED);}
    //射频转换(rpm->Hz)
    static float convert_shootrat(int16_t rpm){return (float)(rpm*MOTORRPM_TO_FIRERATE);}

};





#endif
