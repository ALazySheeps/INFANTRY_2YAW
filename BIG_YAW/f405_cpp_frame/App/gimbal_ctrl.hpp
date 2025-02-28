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
#include "Cybergear_motor.hpp"
#include "dji_motor.hpp"
#include "dr16.hpp"
#include "BMI088driver.hpp"
#include "Motor_Ctrl_Alg.hpp"


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
}Motot_Data_t;




class gimbal_ctrl_c{
    public:
        gimbal_ctrl_c();
        const DR16_n::RC_ctrl_t* dt7;         //遥控器指针

        DJI_Motor_n::DJI_Motor_Instance* yaw_motor;     //yaw6020电机
        
        gimbal_behaviour_e gimbal_mode;   //云台行为模式

//************************************电机设定值******************************************************** */
        Motot_Data_t pitch_motor_data;
        Motot_Data_t yaw_motor_data;

//************************************电机控制算法******************************************************** */
        //lqr手瞄pitch yaw
        MotorCalc_c* lqr_yaw;

//*****************************************函数*********************************************************** */
        void Gimbal_Mode_Set();
        gimbal_behaviour_e gimbal_set_virtual_Mode();
        void Gimbal_CtrlLoop();
        void gimbal_feedbackset();//更新反馈值
        gimbal_behaviour_e* Get_Gimbal_Mode(); //获取云台模式指针
        void Gimbal_Init();

        static gimbal_ctrl_c* Get_Gimbal_Instance();
        private:
        static gimbal_ctrl_c* Gimbal_Instance;  //唯一实例
};




#endif
