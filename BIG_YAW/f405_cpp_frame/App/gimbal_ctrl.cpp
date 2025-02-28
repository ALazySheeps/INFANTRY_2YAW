#include "gimbal_ctrl.hpp"
#include "user_maths.hpp"
#include "user_lib.h"
#include "gimbal_task.hpp"
#include "Motor_Ctrl_Alg.hpp"

user_maths_c gimbal_maths;

//extern uint8_t gimbal_power;
uint8_t gimbal_power = 1;
/*************************************pitch轴限位设定**********************************************/
//相对角度限位（MI电机反馈角度为-3.35 - 53）


//**********************************lqr参数设定****************************************************/
//手瞄lqr参数设定
float k_pitch_lqr[2] = {PITCH_K_0, PITCH_K_1};
float k_yaw_lqr[2] = {YAW_K_0, YAW_K_1};
//自瞄lqr参数设定
float k_virtual_pitch_lqr[2] = {PITCH_VIRTUAL_K_0,PITCH_VIRTUAL_K_1};
float k_virtual_yaw_lqr[2] = {YAW_VIRTUAL_K_0,YAW_VIRTUAL_K_1};


gimbal_ctrl_c::gimbal_ctrl_c():
dt7(nullptr),yaw_motor(nullptr),lqr_yaw(nullptr)
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
      .tx_id = 3,// 看电调闪几下就填几
    }
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

    yaw_motor = new DJI_Motor_n::DJI_Motor_Instance(yaw_motor_confit);

    DR16_n::DR16_c *gimbal_dr16 = DR16_n::DR16_c::dr16_->GetClassPtr();
    dt7 = gimbal_dr16->GetDataStructPtr();


    //控制算法参数设定
    lqr_yaw = new MotorCalc_c(YAW_K_0,YAW_K_1,25000);

    gimbal_mode = GIMBAL_MANUAL;
    memset(&pitch_motor_data,0,sizeof(Motot_Data_t));
    memset(&yaw_motor_data,0,sizeof(Motot_Data_t));
    
    //初始化电机

    //yaw电机待使能（这句是必要的，不然电机会抖，有点神奇）
    yaw_motor->DJIMotorStop();

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

void gimbal_ctrl_c::Gimbal_Mode_Set()
{
    gimbal_behaviour_e last_behaviour;
    static gimbal_behaviour_e rc_behaviour = gimbal_mode;
    static gimbal_behaviour_e kb_behaviour = gimbal_mode;
  
    
    // 手柄
    last_behaviour = rc_behaviour;
    switch(dt7->rc.s2)
    {
        case RC_SW_UP://遥控器右侧拨杆向上拨，视觉控制云台自瞄
            rc_behaviour=gimbal_set_virtual_Mode();
        break;
        
        case RC_SW_MID://遥控器右侧拨杆向中间拨，手动控制云台
            rc_behaviour=GIMBAL_MANUAL;
        break;
        
        case RC_SW_DOWN://往下或其他错误情况：无力模式
        default:
            rc_behaviour=GIMBAL_ZERO_FORCE;
        break;
    }
    if (last_behaviour != rc_behaviour) gimbal_mode = rc_behaviour;

    //键鼠
    last_behaviour = kb_behaviour;
	if(dt7->mouse.press_r)   
        kb_behaviour =gimbal_set_virtual_Mode();
	else    
        kb_behaviour = GIMBAL_MANUAL;

    if (last_behaviour != kb_behaviour) 
        gimbal_mode = kb_behaviour;
    
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
    pitch_motor_data.actual_data.actual_speed = pitch_motor->speed;
    pitch_motor_data.actual_data.actual_ecd_pos = pitch_motor->postion;
    pitch_motor_data.actual_data.actual_current = pitch_motor->torque;

    yaw_motor_data.actual_data.actual_imu_pos = (imu->Yaw);
    yaw_motor_data.actual_data.actual_gyro = (imu->Gyro[2]);
    yaw_motor_data.actual_data.actual_speed = yaw_motor->MotorMeasure.measure.feedback_speed;
    yaw_motor_data.actual_data.actual_ecd_pos = yaw_motor->MotorMeasure.measure.feedback_ecd;
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
        
        yaw_motor_data.mannal_set-=(float)(dt7->rc.ch[0]*YAW_DPI);
        yaw_motor_data.mannal_set-=(float)(dt7->mouse.x*YAW_DPI_MOUSE);
        #if PITCH_LIMIT_WAY==ENCODER
        VAL_LIMIT(pitch_motor_data.mannal_set,angle_limit.PITCH_IMU_MIN_p,angle_limit.PITCH_IMU_MAX_p);
        #else
        VAL_LIMIT(pitch_motor_data.mannal_set,angle_limit.Pitch_IMU_MIN,angle_limit.Pitch_IMU_MAX);
        #endif        
    }
    else//实时更新pitch与yaw轴手瞄设定值，防止因模式切换导致的数据跳变
    {
        pitch_motor_data.mannal_set = pitch_motor_data.actual_data.actual_imu_pos;//   //pitch
        yaw_motor_data.mannal_set = yaw_motor_data.actual_data.actual_imu_pos;//    imu->Yaw;   
        #if PITCH_LIMIT_WAY==ENCODER
        VAL_LIMIT(pitch_motor_data.mannal_set,angle_limit.PITCH_IMU_MIN_p,angle_limit.PITCH_IMU_MAX_p);
        #else
        VAL_LIMIT(pitch_motor_data.mannal_set,angle_limit.Pitch_IMU_MIN,angle_limit.Pitch_IMU_MAX);
        #endif   
    }

    /***自瞄设定数据更新***/
    pitch_motor_data.virtual_set = virtual_pitch_filter->first_order_filter(virtual_data->pitch);
    yaw_motor_data.virtual_set = virtual_yaw_filter->sliding_mean_filter(virtual_data->yaw,30);
    
    //根据模式更改设定值
    switch(gimbal_mode)
    {
        case GIMBAL_MANUAL://手瞄设定值由遥感输入
            pitch_motor_data.final_set = pitch_motor_data.mannal_set;
            yaw_motor_data.final_set = yaw_motor_data.mannal_set;
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
void gimbal_ctrl_c::Gimbal_CtrlLoop()
{
    Gimbal_Mode_Set();
    gimbal_feedbackset();//更新反馈值
    yaw_motor->DJIMotorEnable();

    float Pitch_motortarget = 0,Yaw_motortarget = 0;
    float Yaw_distance = 0,Pitch_distance = 0;

    float temp = 0;
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
            
            //手瞄yaw用LQR控制
            lqr_virtual_yaw->OnlyIpid.ECF_PID_CLEAR();//切换为手瞄时清空自瞄的PID值
            Yaw_motortarget = gimbal_maths.float_min_distance(yaw_motor_data.final_set, yaw_motor_data.actual_data.actual_imu_pos, -180.0f, 180.0f);
            Yaw_distance = (0.0f - Yaw_motortarget) / 57.295779513f;
            lqr_yaw->GetValPoint(&Yaw_distance,&yaw_motor_data.actual_data.actual_gyro);
            yaw_motor_data.output = torque_to_voltage_6020(lqr_yaw->LQR_Calc());
        break;

        
        //自瞄设定值由视觉提供
        case GIMBAL_AUTOATTACK:
            #if PITCH_VIRTUAL_CONTROL_MODE==PID //自瞄pitch使用PID控制
            {
                pid_virtual_pitch->Cascade_pid_Calculate(&pitch_motor_data.final_set,&pitch_motor_data.actual_data.actual_imu_pos,&pitch_motor_data.actual_data.actual_speed);
            }
            #else                       //自瞄pitch使用LQR控制
            float Pitch_virtual_motortarget = float_min_distance(gimbal_ctrl->pitch_set,gimbal_ctrl->imu->Pitch, -180.0f, 180.0f);//Pitch
            float Pitch_virtual_system_state[2] = {((-Pitch_virtual_motortarget) / 57.295779513f), gimbal_ctrl->imu->Gyro[0]}; //Gyro[0]
            LQR_Data_Update(&gimbal_ctrl->lqr_virtual_pitch, Pitch_virtual_system_state);
            LQR_Calculate(&gimbal_ctrl->lqr_virtual_pitch);
            Pitch_output =  gimbal_ctrl->lqr_virtual_pitch.Output[0];
            value_limit(Pitch_output, -6 , 6);
            gimbal_ctrl->pitch_moto.output = -Pitch_output;
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
    pitch_motor->MI_motor_controlmode(pitch_motor_data.output,0,0,0,0);
    yaw_motor->DJIMotorSetRef(yaw_motor_data.output);
}


//米狗电机初始化码盘值()
void gimbal_ctrl_c::MiInit()
{
    static int ReadyCount=0;
    pitch_motor->MI_motor_controlmode(1.2f,0 ,0,0, 0);//运控模式 //让电机低头堵转
    if( (abs(pitch_motor->speed < 0.13f) && (ReadyCount< 100)))
    {
        pitch_motor->MI_motor_setMechPosition2Zero();//零点设置
        ReadyCount++;
    }
    else if (ReadyCount < 100)
    {
        ReadyCount = 0;
    }

    if(ReadyCount>99)
    {
        ReadyCount = 0;
        InitialMi=1;
    }
}

 //获取云台模式指针
gimbal_behaviour_e* gimbal_ctrl_c::Get_Gimbal_Mode()
{
    return &gimbal_mode;
}

float* gimbal_ctrl_c::Get_Gimbal_Pitch_Dif()
{
    return &pitch_dif;
}

float* gimbal_ctrl_c::Get_Gimbal_Yaw_Dif()
{
    return &yaw_dif;
}

//获取唯一实例
gimbal_ctrl_c* gimbal_ctrl_c::Get_Gimbal_Instance()
{
    return Gimbal_Instance;
}

gimbal_ctrl_c* gimbal_ctrl_c::Gimbal_Instance = new gimbal_ctrl_c();





