#include "gimbal_ctrl.hpp"
#include "user_maths.hpp"
#include "user_lib.h"
#include "gimbal_task.hpp"
#include "Motor_Ctrl_Alg.hpp"

user_maths_c gimbal_maths;

//extern uint8_t gimbal_power;
uint8_t gimbal_power = 1;

//**********************************lqr参数设定****************************************************/
//手瞄lqr参数设定
float k_pitch_lqr[2] = {PITCH_K_0, PITCH_K_1};
//自瞄lqr参数设定
float k_virtual_pitch_lqr[2] = {PITCH_VIRTUAL_K_0,PITCH_VIRTUAL_K_1};
float k_virtual_yaw_lqr[2] = {YAW_VIRTUAL_K_0,YAW_VIRTUAL_K_1};


gimbal_ctrl_c::gimbal_ctrl_c():
dt7(nullptr),imu(nullptr),virtual_data(nullptr),Pitch_motor(nullptr),yaw_motor(nullptr),
big_yaw_motor(nullptr),motor_right_fire(nullptr),motor_left_fire(nullptr),
lqr_pitch(nullptr),lqr_reset_yaw(nullptr),pid_yaw_lock(nullptr),lqr_virtual_pitch(nullptr),
lqr_virtual_yaw(nullptr),virtual_pitch_filter(nullptr),virtual_yaw_filter(nullptr)
{
    
}

void gimbal_ctrl_c::Gimbal_Init()
{
    Motor_General_Def_n::Motor_Init_Config_s yaw_motor_confit = {
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
      .tx_id = 2,// 看电调闪几下就填几
    },
    .zero_offset = 0,
    };

    Motor_General_Def_n::Motor_Init_Config_s big_yaw_motor_confit = {
        .motor_type = Motor_General_Def_n::GM6020,
        .can_init_config = { 
          .can_handle = &hcan2,
          .tx_id = 5,// 看电调闪几下就填几
        },
        .zero_offset = 0,
        };

    Motor_General_Def_n::Motor_Init_Config_s right_fric_motor_confit = {
        .controller_param_init_config = {
          .speed_PID = {
            .Kp = FIRE_LEFT_KP,
            .Ki = FIRE_LEFT_KI,
            .Kd = FIRE_LEFT_KD,
            .mode = Output_Limit,
            .max_out = FIRE_MAX,
          },
        },
        .controller_setting_init_config = { 
          .outer_loop_type       =  Motor_General_Def_n::SPEED_LOOP,
          .close_loop_type       =  Motor_General_Def_n::SPEED_LOOP,
          .motor_reverse_flag    =  Motor_General_Def_n::MOTOR_DIRECTION_NORMAL,
          .feedback_reverse_flag =  Motor_General_Def_n::FEEDBACK_DIRECTION_NORMAL,
          // 反馈来源可设置为 MOTOR_FEED OTHER_FEED SPEED_APS ANGULAR_SPEED LINEAR_SPEED
          .speed_feedback_source =  Motor_General_Def_n::MOTOR_FEED,
        },
        .motor_type = Motor_General_Def_n::M3508,
        .can_init_config = { 
          .can_handle = &hcan1,
          .tx_id = 2,// 看电调闪几下就填几
        }
        };
    
        Motor_General_Def_n::Motor_Init_Config_s left_fric_motor_confit = {
        .controller_param_init_config = {
          .speed_PID = {
            .Kp = FIRE_LEFT_KP,
            .Ki = FIRE_LEFT_KI,
            .Kd = FIRE_LEFT_KD,
            .mode = Output_Limit,
            .max_out = FIRE_MAX,
          },
        },
        .controller_setting_init_config = { 
          .outer_loop_type       =  Motor_General_Def_n::SPEED_LOOP,
          .close_loop_type       =  Motor_General_Def_n::SPEED_LOOP,
          .motor_reverse_flag    =  Motor_General_Def_n::MOTOR_DIRECTION_REVERSE,  //设置反转
          .feedback_reverse_flag =  Motor_General_Def_n::FEEDBACK_DIRECTION_NORMAL,
          // 反馈来源可设置为 MOTOR_FEED OTHER_FEED SPEED_APS ANGULAR_SPEED LINEAR_SPEED
          .speed_feedback_source =  Motor_General_Def_n::MOTOR_FEED,
        },
        .motor_type = Motor_General_Def_n::M3508,
        .can_init_config = { 
          .can_handle = &hcan1,
          .tx_id = 1,// 看电调闪几下就填几
        }
        };
    

    //pitch位置环
    pid_alg_n::PID_Init_Config_t pitch_pid_P_confit = {
    .Kp = PITCH_VIRTUAL_P_KP,
    .Ki = PITCH_VIRTUAL_P_KI,
    .Kd = PITCH_VIRTUAL_P_KD,
    .ActualValueSource = nullptr,
    .mode = Output_Limit | Integral_Limit,
    .max_out = PITCH_P_MAX,
    .max_Ierror = 320,
    };

    //pitch速度环
    pid_alg_n::PID_Init_Config_t pitch_pid_S_confit = {
    .Kp = PITCH_VIRTUAL_S_KP,
    .Ki = PITCH_VIRTUAL_S_KI,
    .Kd = PITCH_VIRTUAL_S_KD,
    .ActualValueSource = nullptr,
    .mode = Output_Limit,
    .max_out = PITCH_S_MAX,
    };

    //yaw自锁pid
    pid_alg_n::PID_Init_Config_t yaw_pid_lock_confit = {
        .Kp = PITCH_VIRTUAL_S_KP,
        .Ki = PITCH_VIRTUAL_S_KI,
        .Kd = PITCH_VIRTUAL_S_KD,
        .ActualValueSource = nullptr,
        .mode = Output_Limit,
        .max_out = PITCH_S_MAX,
        };

    //yaw轴PID辅助参数
    pid_alg_n::PID_Init_Config_t yaw_pid_confit = {
    .Kp = 0,
    .Ki = 450,
    .Kd = 0,
    .Kfa = 60000,
    .ActualValueSource = nullptr,
    .mode = Feedforward | Integral_Limit | ChangingIntegrationRate,
    .max_Ierror = 350,
    .errorabsmax = 4,
    .errorabsmin = 0,
    };

    DM_Motor_n::DM_ModePrame_s congfig_ = {
        // 速度kp-0.00372 0.01572
        .kp_max = 500,
        .kp_min = 0,
        .kd_min = 0,
        .kd_max = 5,
        .v_min = -30,
        .v_max = 30,
        .p_min = -2,
        .p_max = 2,
        .t_min = -10,
        .t_max = 10,
        // 位置速度
        .postion_bits = 16,
        .velocity_bits = 12,
        .toeque_bits = 12,
        .kp_bits = 12,
        .kd_bits = 12,
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 0xD4,  //Slave_ID
            .rx_id = 0xC4,  //Master_ID
            .SAND_IDE = CAN_ID_STD,
        },
        .output = {.p_des = 0, .v_des = 0, .Kp = 0, .Kd = 0, .t_des = 0},
    };
    DM_Motor_n::DM_ModePrame_s *pconfig = new DM_Motor_n::DM_ModePrame_s(congfig_);

    Pitch_motor = new DM_Motor_n::DM_Mit_Mode_c(pconfig);
    Pitch_motor->ECF_SetRxCallBack(pitch_rx_call_back);
    yaw_motor = new DJI_Motor_n::DJI_Motor_Instance(yaw_motor_confit);
    big_yaw_motor = new DJI_Motor_n::DJI_Motor_Instance(big_yaw_motor_confit);
     //初始化电机
    motor_right_fire = new DJI_Motor_n::DJI_Motor_Instance(right_fric_motor_confit);
    motor_left_fire  = new DJI_Motor_n::DJI_Motor_Instance(left_fric_motor_confit);

    DR16_n::DR16_c *gimbal_dr16 = DR16_n::DR16_c::dr16_->GetClassPtr();
    dt7 = gimbal_dr16->GetDataStructPtr();
    
    imu = BMI088Instance_c::BMI_List[0]->Get_INS_Data_Point();
    virtual_data = Get_virtual_recive_ptr();

    //控制算法参数设定
    lqr_pitch = new MotorCalc_c(PITCH_K_0,PITCH_K_1,6);
    lqr_reset_yaw = new MotorCalc_c(YAW_VIRTUAL_K_0,YAW_VIRTUAL_K_1,6);
    pid_yaw_lock = new pid_alg_n::pid_alg_c(yaw_pid_lock_confit);
    //pid_virtual_pitch = new pid_alg_n::cascade_pid_c(pitch_pid_P_confit,pitch_pid_S_confit,6);
    lqr_virtual_pitch = new MotorCalc_c(PITCH_VIRTUAL_K_0,PITCH_VIRTUAL_K_1,6);
    lqr_virtual_yaw = new MotorCalc_c(YAW_VIRTUAL_K_0,YAW_VIRTUAL_K_1,25000,yaw_pid_confit);

    virtual_yaw_filter = new filter_alg_n::sliding_mean_filter_c();
    virtual_pitch_filter = new filter_alg_n::first_order_filter_c(1);

    //反馈弹速递推平均滤波
    predict_shoot_speed = new filter_alg_n::recursive_ave_filter_c();  

    gimbal_mode = GIMBAL_MANUAL;
    pitch_dif = yaw_dif = 0;
    memset(&pitch_motor_data,0,sizeof(Motot_Data_t));
    memset(&yaw_motor_data,0,sizeof(Motot_Data_t));

    motor_right_fire->motor_controller.speed_PID.ECF_PID_ChangeActValSource(&motor_right_fire->MotorMeasure.measure.feedback_speed);
    motor_left_fire->motor_controller.speed_PID.ECF_PID_ChangeActValSource(&motor_left_fire->MotorMeasure.measure.feedback_speed);
    
    //初始化电机
    pitch_motor_data.mannal_set = -imu->Roll;
    pitch_motor_data.final_set = -imu->Roll;
    yaw_motor_data.final_set = imu->Yaw;
    //电机使能
    //big_yaw_motor->DJIMotorOnlyRecevie();
    yaw_motor->DJIMotorEnable();
    Pitch_motor->StateSet(DM_Motor_n::DM_ENABLE);
    motor_right_fire->DJIMotorEnable();
    motor_left_fire->DJIMotorEnable();
}


//摩擦轮无力
void gimbal_ctrl_c::set_fric_zero()
{
    left_motor_speed_set = 0;
    right_motor_speed_set = 0;
}

/**
 * @brief          将电机转子rpm值转换到实际射速与射频
 * @retval         none
 */
void gimbal_ctrl_c::Shoot_V_Get(const shoot_msg_t& temp_shoot_info)
{
    //获取当前弹速
    static float last_shoot_speed=25.1f;
    //加入设定样本
    if((temp_shoot_info.shoot_bullet_speed>20&&temp_shoot_info.shoot_bullet_speed<30)&&last_shoot_speed!=temp_shoot_info.shoot_bullet_speed)
    {
        last_shoot_speed=temp_shoot_info.shoot_bullet_speed;
        //对弹速滤波
        Real_physical_Val.shoot_speed=predict_shoot_speed->Recursive_ave_filter(temp_shoot_info.shoot_bullet_speed,10);
    }
    
    //防发电
    if(Real_physical_Val.shoot_speed<20||Real_physical_Val.shoot_speed>31)
    {
        Real_physical_Val.shoot_speed=25;
    }
    
    //获取当前射频
    //Real_physical_Val.shoot_rate = convert_shootrat(motor_pluck_fire->MotorMeasure.measure.feedback_speed);

    Real_physical_Val.shoot_hot=temp_shoot_info.shoot_barrel_heat_current;//获取当前热量

    virtual_tx_data.bullet_speed = Real_physical_Val.shoot_speed;
}

/**
 * @brief          根据遥控器控制摩擦轮开/关
 */
void gimbal_ctrl_c::Fric_Set()
{
    static uint8_t last_key_f;
     if((dt7->rc.ch[4] > -300 && dt7->rc.ch[4] < -50) && isFricReady==false)
     {
         #if NOW_STATE==TEST
            left_motor_speed_set = FIRE_SPEED;
            right_motor_speed_set = FIRE_SPEED;
         #else
         //TODO:加入裁判系统更新弹速设定
         //根据裁判系统设定弹速//联盟赛射速上限30，可不用设定
            left_motor_speed_set = FIRE_SPEED;
            right_motor_speed_set = FIRE_SPEED;
         #endif
            isFricReady = true;
            fric_state  = 1;
     }
     else if((dt7->rc.ch[4] > -300 && dt7->rc.ch[4] < -50) && fric_state==2)
     {
         set_fric_zero();
         isFricReady = false;//摩擦轮关闭，指示不能开火
         fric_state  = 3;
     }
     else if((dt7->rc.ch[4] < 5 && dt7->rc.ch[4] > -5) && (fric_state==1 || fric_state==3))//获取归中状态
     {
         fric_state=abs(fric_state-3);
     }    

     if(((last_key_f==0)&&(dt7->kb.bit.F==1))&&fric_state==0)
     {
         #if NOW_STATE==TEST
            left_motor_speed_set = FIRE_SPEED;
            right_motor_speed_set = FIRE_SPEED;
         #else
         //TODO:加入裁判系统更新弹速设定
         //根据裁判系统设定弹速//联盟赛射速上限30，可不用设定
            left_motor_speed_set = FIRE_SPEED;
            right_motor_speed_set = FIRE_SPEED;
         #endif
         isFricReady=true;
         fric_state=1;
     }
     else if(((last_key_f==0) && (dt7->kb.bit.F==1)) && fric_state==2)
     {
         set_fric_zero();
         isFricReady=false;//摩擦轮关闭，指示不能开火
         fric_state=3;
     }
     else if(((last_key_f==1)&&(dt7->kb.bit.F==0))&&(fric_state==1||fric_state==3))//获取归中状态
     {
         fric_state=abs(fric_state-3);
     }    
     
     last_key_f=dt7->kb.bit.F;

}

/**
 * @brief          自瞄模式切换
 * @param[in]      控制云台结构体指针
 * @retval         none
 */
gimbal_behaviour_e gimbal_ctrl_c::gimbal_set_virtual_Mode()//设定自瞄模式
{
    if(virtual_data->distance <= 0)  //视觉看不到目标时，切换为手瞄
    {
        return GIMBAL_MANUAL;
    }
    else
    {
        return GIMBAL_AUTOATTACK;//看到目标就改为自瞄
    }
}

/**
 * @brief          设定发射模式
 * @param[in]      控制发射模式相关结构体
 * @retval         当前发射模式
 */
void gimbal_ctrl_c::fire_mode_set()
{
    static uint8_t last_key_x;
    static uint8_t last_key_v;        //v切换打符正反向
    if(last_key_v==0&&dt7->kb.bit.V==1)  virtual_tx_data.reset_tracker=!virtual_tx_data.reset_tracker;  //上升沿修改打符正反方向
    
    if(last_key_x==0&&dt7->kb.bit.N==1)  //上升沿修改视觉自瞄目标//N就是X
    {//以下为切换视觉瞄准模式
        if(virtual_tx_data.now_mode==1)
        {
            virtual_tx_data.now_mode=2;
        }
        else if(virtual_tx_data.now_mode==2)
        {
            virtual_tx_data.now_mode=3;
        }
        else 
        {
            virtual_tx_data.now_mode=1;
        }
    }
    
    last_key_x=dt7->kb.bit.N;//N就是x;
    last_key_v=dt7->kb.bit.V;
//    if(last_aim_target==1&&virtual_tx_data->now_mode==2) isSemi=1;//初次切入打符：单发
//    else if(last_aim_target==3&&virtual_tx_data->now_mode==1) isSemi=0;//初次切入自瞄：连发

}


void gimbal_ctrl_c::Gimbal_Mode_Set()
{
    fire_mode_set();

    gimbal_behaviour_e last_behaviour;
    static gimbal_behaviour_e rc_behaviour = gimbal_mode;
    static gimbal_behaviour_e kb_behaviour = gimbal_mode;
    static uint8_t last_rc_ = RC_SW_DOWN;
    static uint8_t last_press_r_ = 0;
    
    // 手柄
    last_behaviour = rc_behaviour;
    if(dt7->rc.s2==RC_SW_DOWN)
    {
        rc_behaviour = GIMBAL_ZERO_FORCE;
    }
    else if(dt7->rc.s2!=last_rc_)
    {
        is_first_reset = true;
        is_reset = false;
        rc_behaviour = GIMBAL_RESET;
        yaw_motor->MotorMeasure.MeasureClear();   //用于自锁
    }
    else if(dt7->rc.s2==RC_SW_UP && is_reset == true)
    {
        rc_behaviour=gimbal_set_virtual_Mode();
    }
    else if(dt7->rc.s2==RC_SW_MID && is_reset == true)
    {
        rc_behaviour=GIMBAL_MANUAL;
    }
    last_rc_ = dt7->rc.s2;

    if (last_behaviour != rc_behaviour)    
    {
        gimbal_mode = rc_behaviour;
    }

    //键鼠
    last_behaviour = kb_behaviour;

    if(last_press_r_ != dt7->mouse.press_r)
    {
        is_first_reset = true;
        is_reset = false;
        kb_behaviour = GIMBAL_RESET;
        yaw_motor->MotorMeasure.MeasureClear();
    }
    else if(dt7->mouse.press_r == 1 && is_reset == true)
    {
        kb_behaviour=gimbal_set_virtual_Mode();
    }
    else if(dt7->mouse.press_r == 0 && is_reset == true)
    {
        kb_behaviour=GIMBAL_MANUAL;
    }
    last_press_r_ = dt7->mouse.press_r;
	
    if (last_behaviour != kb_behaviour)     
    {
        gimbal_mode = kb_behaviour;
    }
    
}


/**
 * @brief          更新云台反馈数据，包括设定值更新
 * @param[in]      控制云台结构体指针
 * @retval         none
 */
void gimbal_ctrl_c::gimbal_feedbackset()//更新反馈值
{   

    /***云台实际数据的更新***/
    pitch_motor_data.actual_data.actual_imu_pos = (-imu->Roll);
    pitch_motor_data.actual_data.actual_gyro = (-imu->Gyro[1]);
    pitch_motor_data.actual_data.actual_speed = Pitch_motor->get_data_.velocity;
    pitch_motor_data.actual_data.actual_ecd_pos = Pitch_motor->get_data_.postion;
    pitch_motor_data.actual_data.actual_current = Pitch_motor->get_data_.toeque;

    yaw_motor_data.actual_data.actual_imu_pos = (imu->Yaw);
    yaw_motor_data.actual_data.actual_gyro = (imu->Gyro[2]);
    yaw_motor_data.actual_data.actual_speed = yaw_motor->MotorMeasure.measure.feedback_speed;
    yaw_motor_data.actual_data.actual_ecd_pos = yaw_motor->MotorMeasure.measure.record_ecd;     //使用总积累编码值，用于自锁
    yaw_motor_data.actual_data.actual_current = yaw_motor->MotorMeasure.measure.feedback_real_current;

    #if PITCH_LIMIT_WAY==ENCODER  //编码值限位
    IMU_AngleLimit(-angle_limit.PITCH_MOTOR_MIN,-angle_limit.PITCH_MOTOR_MAX,
            -pitch_motor_data.actual_data.actual_ecd_pos,pitch_motor_data.actual_data.actual_imu_pos,//Pitch
            &angle_limit.PITCH_IMU_MIN_p,&angle_limit.PITCH_IMU_MAX_p);//PITCH采用绝对角度限位值设定更新
    gimbal_maths.loop_fp32_constrain(angle_limit.PITCH_IMU_MIN_p,-180.0f,180.0f);
    gimbal_maths.loop_fp32_constrain(angle_limit.PITCH_IMU_MAX_p,-180.0f,180.0f);
    
    #endif

    /***手瞄设定数据更新***/
    if(gimbal_mode==GIMBAL_MANUAL && gimbal_power==1)//只有手瞄模式允许输入遥控器通道值//需要等待电管有电
    {
        pitch_motor_data.mannal_set += (float)(dt7->rc.ch[1]*PITCH_DPI);   
        pitch_motor_data.mannal_set -= (float)(dt7->mouse.y*PITCH_DPI_MOUSE);
        
        #if PITCH_LIMIT_WAY==ENCODER
        VAL_LIMIT(pitch_motor_data.mannal_set,angle_limit.PITCH_IMU_MIN_p,angle_limit.PITCH_IMU_MAX_p);
        #else
        VAL_LIMIT(pitch_motor_data.mannal_set,angle_limit.Pitch_IMU_MIN,angle_limit.Pitch_IMU_MAX);
        #endif        
    }
    else
    {
        pitch_motor_data.mannal_set = pitch_motor_data.actual_data.actual_imu_pos;//   //pitch
        yaw_motor_data.mannal_set = yaw_motor_data.actual_data.actual_imu_pos;//    imu->Yaw;   
        

        #if PITCH_LIMIT_WAY==ENCODER
        VAL_LIMIT(pitch_motor_data.mannal_set,angle_limit.PITCH_IMU_MIN_p,angle_limit.PITCH_IMU_MAX_p);
        #else
        VAL_LIMIT(pitch_motor_data.mannal_set,angle_limit.Pitch_IMU_MIN,angle_limit.Pitch_IMU_MAX);
        #endif
    }

    if(gimbal_mode==GIMBAL_RESET && is_first_reset==true)//复位模式
    {
        is_first_reset = false;
        pitch_motor_data.reset_angle = pitch_motor_data.actual_data.actual_imu_pos;
        yaw_motor_data.reset_angle = yaw_motor_data.actual_data.actual_imu_pos;
    }

    /***自瞄设定数据更新***/
    pitch_motor_data.virtual_set = virtual_pitch_filter->first_order_filter(virtual_data->pitch);
    yaw_motor_data.virtual_set = virtual_yaw_filter->sliding_mean_filter(virtual_data->yaw,30);
    
    //根据模式更改设定值
    switch(gimbal_mode)
    {
        case GIMBAL_MANUAL://手瞄设定值由遥感输入
            pitch_motor_data.final_set = pitch_motor_data.mannal_set;
            yaw_motor_data.final_set = yaw_motor_data.actual_data.actual_imu_pos;
        break;

        case GIMBAL_RESET://复位模式，小yaw锁imu不动
            pitch_motor_data.final_set = pitch_motor_data.reset_angle;
            yaw_motor_data.final_set = yaw_motor_data.reset_angle;
        break;
        
        case GIMBAL_AUTOATTACK://自瞄设定值由视觉提供
            //传入视觉设定值
            pitch_motor_data.final_set = pitch_motor_data.virtual_set;
            yaw_motor_data.final_set = yaw_motor_data.virtual_set;
        break;
        
        case GIMBAL_ZERO_FORCE://无力模式，直接改变设定值和实际值相同，电机此时计算值约等于没输出
        default:
            pitch_motor_data.final_set = pitch_motor_data.actual_data.actual_imu_pos;
            yaw_motor_data.final_set = yaw_motor_data.actual_data.actual_imu_pos;
        break;
    }
    /***设定值限幅***/
    #if PITCH_LIMIT_WAY==ENCODER
    VAL_LIMIT(pitch_motor_data.final_set,angle_limit.PITCH_IMU_MIN_p,angle_limit.PITCH_IMU_MAX_p);
    #else
    VAL_LIMIT(pitch_motor_data.final_set,angle_limit.Pitch_IMU_MIN,angle_limit.Pitch_IMU_MAX);
    #endif
    

    /***视觉偏差值计算***/
    pitch_dif=(pitch_motor_data.virtual_set - pitch_motor_data.actual_data.actual_imu_pos);
    yaw_dif  =(yaw_motor_data.virtual_set   - yaw_motor_data.actual_data.actual_imu_pos);
}



/**
 * @brief          云台控制设定值设定，PID\LQR计算
 * @param[in]      控制云台结构体指针
 * @retval         none
 */
void gimbal_ctrl_c::Gimbal_CtrlLoop(const shoot_msg_t& temp_shoot_info)
{
    Shoot_V_Get(temp_shoot_info);
    Gimbal_Mode_Set();
    gimbal_feedbackset();//更新反馈值

    float Pitch_motortarget = 0;
    float Pitch_distance = 0;

    float Yaw_virtual_motortarget = 0;
    float Yaw_virtual_distance = 0;
    
    //yaw轴6020大概需要给个七八千的值才能动起来，逆时针为正
    switch(gimbal_mode)
    {
        //手瞄设定值由遥感输入
        case GIMBAL_MANUAL:
            //手瞄pitch使用LQR控制
            Pitch_motortarget = gimbal_maths.float_min_distance(pitch_motor_data.final_set,pitch_motor_data.actual_data.actual_imu_pos, -180.0f, 180.0f);
            Pitch_distance = (0.0f - Pitch_motortarget) / 57.295779513f;
            lqr_pitch->GetValPoint(&Pitch_distance,&pitch_motor_data.actual_data.actual_gyro);
            pitch_motor_data.output = -lqr_pitch->LQR_Calc();
            
            //手瞄yaw自锁
            lqr_virtual_yaw->OnlyIpid.ECF_PID_CLEAR();//切换为手瞄时清空自瞄的PID值

            yaw_motor_data.output = pid_yaw_lock->ECF_PID_Calculate(0,yaw_motor_data.actual_data.actual_ecd_pos);
        break;

        //复位模式，小yaw锁imu不动
        case GIMBAL_RESET:
            Pitch_motortarget = gimbal_maths.float_min_distance(pitch_motor_data.final_set,pitch_motor_data.actual_data.actual_imu_pos, -180.0f, 180.0f);
            Pitch_distance = (0.0f - Pitch_motortarget) / 57.295779513f;
            lqr_pitch->GetValPoint(&Pitch_distance,&pitch_motor_data.actual_data.actual_gyro);
            pitch_motor_data.output = -lqr_pitch->LQR_Calc();
            
            //手瞄yaw自锁
            Yaw_virtual_motortarget = gimbal_maths.float_min_distance(yaw_motor_data.final_set, yaw_motor_data.actual_data.actual_imu_pos, -180.0f, 180.0f);
            Yaw_virtual_distance = (0.0f - Yaw_virtual_motortarget) / 57.295779513f;
            lqr_reset_yaw->GetValPoint(&Yaw_virtual_distance, &yaw_motor_data.actual_data.actual_gyro);
            yaw_motor_data.output = torque_to_voltage_6020(lqr_reset_yaw->LQR_Calc());
        break;

        
        //自瞄设定值由视觉提供
        case GIMBAL_AUTOATTACK:
            #if PITCH_VIRTUAL_CONTROL_MODE==PID //自瞄pitch使用PID控制
            {
                pid_virtual_pitch->Cascade_pid_Calculate(&pitch_motor_data.final_set,&pitch_motor_data.actual_data.actual_imu_pos,&pitch_motor_data.actual_data.actual_speed);
            }
            #else                       //自瞄pitch使用LQR控制
            Pitch_motortarget = gimbal_maths.float_min_distance(pitch_motor_data.final_set,pitch_motor_data.actual_data.actual_imu_pos, -180.0f, 180.0f);
            Pitch_distance = (0.0f - Pitch_motortarget) / 57.295779513f;
            lqr_virtual_pitch->GetValPoint(&Pitch_distance,&pitch_motor_data.actual_data.actual_gyro);
            pitch_motor_data.output = -lqr_virtual_pitch->LQR_Calc();
            #endif
        
            Yaw_virtual_motortarget = gimbal_maths.float_min_distance(yaw_motor_data.final_set, yaw_motor_data.actual_data.actual_imu_pos, -180.0f, 180.0f);
            Yaw_virtual_distance = (0.0f - Yaw_virtual_motortarget) / 57.295779513f;
            //视觉控制云台使用PID辅助提高精确度与响应
            lqr_virtual_yaw->GetValPoint(&Yaw_virtual_distance, &yaw_motor_data.actual_data.actual_gyro);
            yaw_motor_data.output = torque_to_voltage_6020(lqr_virtual_yaw->LQR_Calc());
        break;
        
        case GIMBAL_ZERO_FORCE:
        default:
             pitch_motor_data.output = 0;
             yaw_motor_data.output = 0;
        break;
    }
    //Pitch_motor->Transmit(0,0,0,0,pitch_motor_data.output);
    yaw_motor->DJIMotorSetRef(yaw_motor_data.output);
    //摩擦轮控制值设定
    motor_left_fire->DJIMotorSetRef(left_motor_speed_set);
    motor_right_fire->DJIMotorSetRef(right_motor_speed_set);

    if(pitch_dif < 1 && yaw_dif < 1 && isFricReady == 1)
    {
        allow_fire_ = true;
    }
    else
    {
        allow_fire_ = false;
    }

    if(abs(yaw_motor->MotorMeasure.measure.relative_angle - big_yaw_motor->MotorMeasure.measure.relative_angle) < 5 && big_yaw_motor->MotorMeasure.measure.feedback_speed < 300)
    {
        is_reset = true;
    }
}


//获取唯一实例
gimbal_ctrl_c* gimbal_ctrl_c::Get_Gimbal_Instance()
{
    return Gimbal_Instance;
}

gimbal_ctrl_c* gimbal_ctrl_c::Gimbal_Instance = new gimbal_ctrl_c();


void gimbal_ctrl_c::pitch_rx_call_back(BSP_CAN_Part_n::CANInstance_c *register_instance) // 够潮的dm框架
    {
        int p_int, v_int, t_int;
        uint8_t rx_buf[8] = {0};
        memcpy(rx_buf, register_instance->rx_buff, 8); // 存储数据，防止变化
        Gimbal_Instance->Pitch_motor->get_data_.nowState = (DM_Motor_n::DM_NowState_e)(rx_buf[0]  >> 4);
        p_int = (rx_buf[1] << 8) | rx_buf[2];
        v_int = (rx_buf[3] << 4) | (rx_buf[4] >> 4);
        t_int = ((rx_buf[4] & 0xF) << 8) | rx_buf[5];
        Gimbal_Instance->Pitch_motor->get_data_.mos_temperture = rx_buf[6];
        Gimbal_Instance->Pitch_motor->get_data_.motor_temperture = rx_buf[7];
        Gimbal_Instance->Pitch_motor->get_data_.postion = DM_Motor_n::uint_to_float(p_int, -2, 2, 16); // (-12.5,12.5)  //单位是转
        Gimbal_Instance->Pitch_motor->get_data_.velocity = DM_Motor_n::uint_to_float(v_int, -45, 45, 12);    // (-45.0,45.0)
        Gimbal_Instance->Pitch_motor->get_data_.toeque = DM_Motor_n::uint_to_float(t_int, -18, 18, 12);      //(-18.0,18.0)
    }


