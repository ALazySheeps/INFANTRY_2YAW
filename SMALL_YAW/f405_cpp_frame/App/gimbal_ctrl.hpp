#ifndef __GIMBAL_CTRL_H__
#define __GIMBAL_CTRL_H__

extern "C"
{
    #include "Vision.h"
}

#include "real_main.hpp"
#include "gimbal_confit.hpp"
#include "imu_task.hpp"
#include "Alg_PID.hpp"
#include "lqr_alg.hpp"
#include "dm_mit_mode.hpp"
#include "dji_motor.hpp"
#include "dr16.hpp"
#include "BMI088driver.hpp"
#include "Motor_Ctrl_Alg.hpp"
#include <functional>
#include "fire_confit.hpp"


typedef enum
{
	GIMBAL_MANUAL,		         // 手动状态
	GIMBAL_AUTOATTACK,	         // 自瞄状态
	GIMBAL_SMALL_AUTOBUFF,	         // 小符状态
	GIMBAL_BIG_AUTOBUFF,    	 // 大符状态
	GIMBAL_REPLENISHMEN,             // 补给状态
        GIMBAL_ZERO_FORCE,               // 无力状态
        GIMBAL_RESET                     // 复位状态
} gimbal_behaviour_e;

typedef struct
{
        float actual_imu_pos;
        float actual_ecd_pos;
        float actual_speed;
        float actual_current;
        float actual_gyro;
}Actual_Data_t;

typedef struct
{
        Actual_Data_t actual_data;   //实际值
        float mannal_set;   //手瞄设定值
        float virtual_set;  //视觉设定值
        float final_set;    //最终决定设定值
        float output;       //算法计算输入值
        float reset_angle;   //记录复位时的角度
}Motot_Data_t;

typedef struct
{       //PITCH轴相对角度限位（电机）
        const float PITCH_MOTOR_MIN=-0.5f;
        const float PITCH_MOTOR_MAX=-50.0f;
        //PITCH轴绝对角度限位(IMU)
        const float Pitch_IMU_MAX =  30.0f;
        const float Pitch_IMU_MIN = -19.5f;
        //PITCH轴相对角度限位（IMU）
        float PITCH_IMU_MIN_p =  30.0f;
        float PITCH_IMU_MAX_p = -19.5f;
}Pitch_Angle_Limit;  //pitch轴imu限位

typedef struct
{

}gimbal_control_alg_t;


class gimbal_ctrl_c{
    public:
        gimbal_ctrl_c();
        ~gimbal_ctrl_c() = default;
        const DR16_n::RC_ctrl_t* dt7;         //遥控器指针
        const INS_t* imu;                     //陀螺仪指针
        Visual_Tx_t     virtual_tx_data;  //发送视觉数据
        const Visual_Rx_t* virtual_data;      //视觉数据指针

        DM_Motor_n::DM_Mit_Mode_c *Pitch_motor;        //pitch达妙电机
        DJI_Motor_n::DJI_Motor_Instance* yaw_motor;     //yaw6020电机

        DJI_Motor_n::DJI_Motor_Instance* big_yaw_motor;     //大yaw6020电机

        DJI_Motor_n::DJI_Motor_Instance* motor_left_fire;
        DJI_Motor_n::DJI_Motor_Instance* motor_right_fire;
        
        gimbal_behaviour_e gimbal_mode;   //云台行为模式

        bool allow_fire_;       //允许发弹标志位
        bool isFricReady;       //是否开启了摩擦轮
        bool is_reset = false;    //是否复位
        bool is_first_reset = false; //是否首次进入复位
        uint8_t fric_state=0;  //摩擦轮状态 遥控器：初始为0，上波一次为1，归中为2，再次上波为3，归中为0  键盘：初始为0，按下为1，松开为2，再次按下为3，松开为0

//************************************电机设定值******************************************************** */
        Motot_Data_t pitch_motor_data;
        Motot_Data_t yaw_motor_data;

        float left_motor_speed_set;     //左摩擦轮速度设定值
        float right_motor_speed_set;    //右摩擦轮速度设定值

//************************************电机控制算法******************************************************** */
        //lqr手瞄pitch
        MotorCalc_c* lqr_pitch;
        //yaw复位lqr
        MotorCalc_c* lqr_reset_yaw;
        pid_alg_n::pid_alg_c* pid_yaw_lock;  //yaw自锁pid
            
        //自瞄pitch使用PID控制 TODO:LQR
        MotorCalc_c* lqr_virtual_pitch;

        //自瞄yaw使用LQR控制,添加pid消除稳态误差
        MotorCalc_c* lqr_virtual_yaw;
            

        filter_alg_n::first_order_filter_c* virtual_pitch_filter;  //pitch自瞄低通滤波
        filter_alg_n::sliding_mean_filter_c* virtual_yaw_filter;    //yaw自瞄滑动窗口滤波
        
        float pitch_dif; //自瞄偏差
        float yaw_dif;

        Pitch_Angle_Limit angle_limit;  //pitch角度限位
        filter_alg_n::recursive_ave_filter_c* predict_shoot_speed;  //递推平均滤波

        //真实物理量
        struct{
                float shoot_speed;  //  弹速
                float shoot_rate;   //  射频
                float shoot_hot;    //（预留）热量
        }Real_physical_Val;

//*****************************************函数*********************************************************** */
        void fire_mode_set();
        void Gimbal_Mode_Set();
        void Shoot_V_Get(const shoot_msg_t& temp_shoot_info);
        gimbal_behaviour_e gimbal_set_virtual_Mode();
        void Gimbal_CtrlLoop(const shoot_msg_t& temp_shoot_info);
        void gimbal_feedbackset();//更新反馈值
        void Gimbal_Init();
        void set_fric_zero();  //摩擦轮无力
        void Fric_Set();       //遥控器控制摩擦轮开关

        static gimbal_ctrl_c* Get_Gimbal_Instance();
        private:
        static gimbal_ctrl_c* Gimbal_Instance;  //唯一实例

        static void pitch_rx_call_back(BSP_CAN_Part_n::CANInstance_c *register_instance);
};




#endif
