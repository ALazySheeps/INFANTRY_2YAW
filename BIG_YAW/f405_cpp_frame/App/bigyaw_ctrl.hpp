#ifndef __BIGYAW_CTRL_HPP__
#define __BIGYAW_CTRL_HPP__

#include "real_main.hpp"
#include "Motor_Ctrl_Alg.hpp"
#include "dji_motor.hpp"
#include "Alg_PID.hpp"
#include "dr16.hpp"
#include "fire_confit.hpp"
#include "gimbal_confit.hpp"
extern "C" 
{
    #include "visual_config.h"
}

#define SMALLYAW_ID 0x411
#define BIGYAW_ID 0x224
#define CHASSIS_ID 0x112

enum class gimbal_behaviour_e
{
	GIMBAL_MANUAL,		         // 手动状态
	GIMBAL_AUTOATTACK,	         // 自瞄状态
	GIMBAL_SMALL_AUTOBUFF,	     // 小符状态
	GIMBAL_BIG_AUTOBUFF,    	 // 大符状态
	GIMBAL_REPLENISHMEN,         // 补给状态
    GIMBAL_ZERO_FORCE            // 无力状态
} ;

//大yaw控制模式
enum class big_yaw_motor_behaviour_e
{
    BIG_YAW_ZERO_FORCE,         //无力
    BIG_YAW_MANUAL,             //手描
    BIG_YAW_RESET,              //归中
    BIG_YAW_LOCK                //锁定
} ;    

class yaw_motor_ctrl_c
{
    public:
    yaw_motor_ctrl_c(uint8_t big_yaw_motor_id,uint8_t small_yaw_motor_id,float big_yaw_offset_angle,float small_yaw_offset_angle);
    ~yaw_motor_ctrl_c() = default;

    private:

    uint8_t big_yaw_id_;
    uint8_t small_yaw_id_;
    DJI_Motor_n::DJI_Motor_Instance* bigyaw_motor;
    DJI_Motor_n::DJI_Motor_Instance* smallyaw_motor;

    /**电机控制算法 */
    MotorCalc_c* bigyaw_lqr_alg;

    const float big_yaw_offset_angle_;  //0°偏角
    const float small_yaw_offset_angle_;  //0°偏角
    float target_angle_ = 0;   //目标角度
    float big_yaw_angle = 0;   //大yaw当前角度
    float small_yaw_angle = 0; //小yaw当前角度
    float big_yaw_speed = 0;   //yaw电机转速
    bool is_reset = false;         //归中标志
    float output = 0;

    gimbal_behaviour_e gimbal_mode_ = gimbal_behaviour_e::GIMBAL_ZERO_FORCE;  //云台行为模式
    big_yaw_motor_behaviour_e big_yaw_motor_mode_ = big_yaw_motor_behaviour_e::BIG_YAW_ZERO_FORCE;
    
    void Yaw_set_mode(gimbal_behaviour_e gimbal_mode);    //设置yaw控制模式
    void Yaw_data_update(DR16_n::RC_ctrl_t &dr16_data);
    void Yaw_set_zero_output();
    void Yaw_ctrl_data_caculate();
    public:
    void Yaw_init();
    float Yaw_get_output() const;
    void Yaw_ctrl_loop(gimbal_behaviour_e gimbal_mode,DR16_n::RC_ctrl_t &dr16_data);

};

//发射输入方式设定
typedef enum{
    RST=0,      //拨杆归中
    SEMI,       //拨杆下拨
    AUTO,       //拨杆拉满
}fire_set_mode_e;

typedef enum{
    SEMI_FIRE=3,    //单发
    AUTO_FIRE,      //连发
    NO_FIRE,        //不发射（锁定状态）
    ZERO_FIRE       //发射机构无力
}fire_mode_t;

class pluck_ctrl_c
{
    public:
    pluck_ctrl_c(uint8_t id,CAN_HandleTypeDef* can_handle);
    ~pluck_ctrl_c() = default;

    private:
    uint8_t id_;
    CAN_HandleTypeDef* can_handle_;
    DJI_Motor_n::DJI_Motor_Instance* pluck_motor;     //拨弹盘2006电机
     
    pid_alg_n::cascade_pid_c* pluck_semi_pid; //拨弹盘单发控制pid
    pid_alg_n::pid_alg_c* pluck_auto_pid;  //拨弹盘连发控制pid

    float output = 0;       //给定输出值

    fire_set_mode_e fire_set_mode_;     //拨杆状态
    fire_mode_t fire_mode_;             //发射模式

    float pluck_motor_auto_set;     //拨弹盘连发设定值
    float pluck_motor_semi_set;     //拨弹盘单发设定值

    uint8_t robot_level_; //机器人等级
    float fire_rate_;    //射频

    bool singlefire_state=0;  //子弹是否发送出去 0未发送 1发送
    bool allow_fire_;  //允许开火标志位
    bool isSemi;       //是否单发（键鼠操作）
    bool isStuck;      //是否卡弹
    // struct{
    //     
    //     
        
    // }static_flag;

    void Pluck_set_ctrl_data(const DR16_n::RC_ctrl_t &dr16_data);
    inline void Pluck_hurt_ctrl(const DR16_n::RC_ctrl_t &dr16_data,const shoot_msg_t& temp_shoot_info);
    void Pluck_lock();
    void Pluck_zero();
    void Pluck_set_mode();
    void Pluck_set_target_data();
    void Pluck_ctrl_cauculate();
    void Pluck_semi_ctrl();
    void Pluck_auto_ctrl(uint8_t robot_level);

    public:
    void Pluck_init();
    void Pluck_ctrl_loop(const DR16_n::RC_ctrl_t &dr16_data,const shoot_msg_t& temp_shoot_info);
    void Pluck_allow_fire();
    void Pluck_stop_fire();
};


class big_yaw_ctrl_c: public yaw_motor_ctrl_c,pluck_ctrl_c
{
    public:
    big_yaw_ctrl_c();
    ~big_yaw_ctrl_c() = default;

    BSP_CAN_Part_n::CANInstance_c bigyaw_to_chassis;
    BSP_CAN_Part_n::CANInstance_c big_yaw_to_smallyaw;

    static big_yaw_ctrl_c* big_yaw_instance_;
    static big_yaw_ctrl_c* Get_big_yaw_instance();

    shoot_msg_t temp_shoot_info;  //底盘发来的发射机构信息
    DR16_n::RC_ctrl_t dr16_data;  //遥控器数据
    bool allow_fire_ = false;     //是否允许发射
    gimbal_behaviour_e gimbal_behaviour_ = gimbal_behaviour_e::GIMBAL_ZERO_FORCE;

    void Big_Yaw_ctrl_loop();

    static void DecodeChassisData(BSP_CAN_Part_n::CANInstance_c* register_instance);
    static void DecodeSmallYawData(BSP_CAN_Part_n::CANInstance_c* register_instance);
};




#endif
