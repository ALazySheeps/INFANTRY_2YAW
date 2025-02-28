#ifndef __CHASSIS_CTRL_HPP
#define __CHASSIS_CTRL_HPP


#include "bsp_can.hpp"
#include "Alg_PID.hpp"
#include "filter.hpp"
#include "dr16.hpp"
#include "dji_motor.hpp"
#include "user_maths.hpp"
#include "lqr_alg.hpp"
#include "chassis_confit.hpp"
#include "bsp_referee.hpp"

#include <set>


using namespace Motor_General_Def_n;
using namespace pid_alg_n;

namespace chassis_ctrl_n{

    typedef struct
    {
        float x_speed;
        float y_speed;
        float z_speed;
        uint8_t start_rotate = 0;
    }navigation_control_data;  //导航控制数据

    typedef enum{
        MANNUL=0,//底盘正常移动，不跟随云台
        FOLLOW,  //底盘跟随云台
        ROTATION,//底盘小陀螺
        CRAZY,
        NAVIGATION  //导航模式
    }Chassis_Mode_e;

    typedef enum{
        ZERO_FORCE=10,//底盘无力状态
        SPEED,
        LOCK    //底盘位置锁死状态
    }Chassis_State_e;

    //底盘轮组编号
    typedef enum{
        RF = 0,
        RB = 1,
        LB = 2,
        LF = 3,
        WHEEL_NUM = 4   
    }Wheel_num_e; 

    typedef struct{
        float x = 0.0f;
        float y = 0.0f;
        float z_spd = 0.0f;
        float z_pos = 0.0f;
    }V_t;

    typedef struct{
        float motor_set_ref;          //设定值
        float motor_give_current;     //经计算后电流给定值
    }Motor_data_t;

    typedef struct{
        float Chassis_Power_All = 0;//底盘总功率
        int16_t speed[4];//转子转速
        int16_t I_out[4];//力矩电流
        float P_out[4];//电机输出功率
        float P_in[4];//电机输入功率
        int16_t F_out[4];//输出力矩
        int16_t I_max[4];//力矩电流
        int16_t give_current[4];//PID计算电流值
        int8_t rever[4] = {1,1,1,1}; //取绝对值系数
    }Power_ctrl_t;  //计算功率相关
        
    class chassis_ctrl_c{
        public:
            chassis_ctrl_c();
            static chassis_ctrl_c* Chassis_Instance;   //唯一实例
        /***************************************底盘控制相关指针********************************************/
            DR16_n::RC_ctrl_t* dt7;    //遥控器指针
            REFEREE_t* referee;        //裁判系统指针

            navigation_control_data* navigation_p;
            
        /***************************************************************************************************/

        /******************************************底盘控制数据********************************************/
            chassis_ctrl_n::V_t Vset_gimbal; //遥控器在云台坐标系下的设定值
            chassis_ctrl_n::V_t Vset_chassis; //云台坐标分解到底盘
            struct {
                float x = 0.0f;  // X轴分量
                float y = 0.0f;  // Y轴分量
                float z = 0.0f;  // Z轴分量
            } ChassisVelocity;  //底盘设定速度
            
            Chassis_Mode_e mode;    //底盘模式
            Chassis_State_e state;  //底盘运动状态
        /***************************************************************************************************/

        /***************************************底盘电机控制数据********************************************/
            //云台底盘差角
            float chassis_yaw_diff_angle = 0.0f;

            DJI_Motor_n::DJI_Motor_Instance* wheel_motor[WHEEL_NUM];
            DJI_Motor_n::DJI_Motor_Instance* yaw_motor; //yaw轴电机

            pid_alg_n::pid_alg_c* motor_speed_pid[WHEEL_NUM];   //电机速度环PID
            pid_alg_n::pid_alg_c* motor_lock_pid[WHEEL_NUM];    //电机自锁PID

            Motor_data_t wheel_data[WHEEL_NUM];

            uint8_t have_deline = 0;         //失联电机数量
            uint8_t deline_motor[4] = {0,0,0,0};  //失联电机集合
        /***************************************************************************************************/

        /***********************************底盘设定速度相关pid********************************************/
            //底盘遥控器通道xy设定速度放大 //这实际上是个斜坡
            pid_alg_n::pid_alg_c* pid_chassis_x;
            pid_alg_n::pid_alg_c* pid_chassis_y;
            
            //底盘XY轴速度设定滤波
            filter_alg_n::first_order_filter_c* speedX_filter;
            filter_alg_n::first_order_filter_c* speedY_filter;
            
            //底盘小陀螺转速设定速度放大 //PID计算只设置目标值，实际值来源未传入指针，值为0 //这实际上是个斜坡
            pid_alg_n::pid_alg_c* pid_chassis_z; 
            
            //底盘跟随模式跟随设定角度与速度pid计算
            #if FOLLOW_CONTRAL_MODE==PID
            //PID计算只设置目标值，实际值来源未传入指针，值为0
            Motor_General_Def_n::Motor_Controller_c* pid_follow;
            #else
            //LQR算法
            lqr_alg_n::lqr_alg_c* lqr_follow;
            #endif
        /***************************************************************************************************/

        /****************************************功率控制****************************************************/
            Power_ctrl_t power_ctrl;
        /***************************************************************************************************/

        /**************************************底盘控制函数**************************************************/
            void Chassis_Init();
            void Chassis_Ctrl_Loop();
            void Set_Chassis_navigation(navigation_control_data &navigation_data);
            static chassis_ctrl_c* Get_Chassis_Instance();
        private:
            void chassis_mode_set();
            void Chassis_SetBehavior();
            void Chassis_Act();
            void gimbalToChassisCoord();
            void chassis_zeroForce_set();
            void chassis_speed_pid_calculate();
            void chassis_motion_decomposition();
            void motor_speed_pid_calculate();

            float chassis_motion_speed_distortion(float x,float y,float z);

            void Chassis_power_limit(uint8_t Power_Limit_Val);
    };




}



#endif
