#include "chassis_ctrl.hpp"
#include "user_maths.hpp"
#include "chassis_confit.hpp"

using namespace chassis_ctrl_n;
using namespace Motor_General_Def_n;
using namespace pid_alg_n;

//自定义数学库
user_maths_c math_;

#define LIMIT_SPEED 0       //TODO
#define LIMIT_CURRENT 1     //TODO
//功率限制方式 2：不限制
#define POWER_LIMIT 2

float rotate_speed = CHASSIS_ROTATION_SPEED; //小陀螺旋转速度

//底盘构造函数
#if FOLLOW_CONTRAL_MODE==PID
chassis_ctrl_c::chassis_ctrl_c():
mode(MANNUL),state(ZERO_FORCE),
yaw_motor(nullptr),
pid_chassis_x(nullptr),pid_chassis_y(nullptr),pid_chassis_z(nullptr),
speedX_filter(nullptr),speedY_filter(nullptr),pid_follow(nullptr)
{
    
}

#else
chassis_ctrl_c::chassis_ctrl_c():
mode(MANNUL),state(ZERO_FORCE),
yaw_motor(nullptr),
pid_chassis_x(nullptr),pid_chassis_y(nullptr),pid_chassis_z(nullptr),
speedX_filter(nullptr),speedY_filter(nullptr),lqr_follow(nullptr)
{
    
}

#endif

void chassis_ctrl_c::Chassis_Init()
{
Motor_General_Def_n::Motor_Init_Config_s wheelmotor_config[4];

//底盘麦轮电机只使用速度环，自锁模式下只使用位置环
Motor_General_Def_n::Motor_Control_Setting_s motor_setting = { 
        .outer_loop_type       =  Motor_General_Def_n::OPEN_LOOP,
        .close_loop_type       =  Motor_General_Def_n::OPEN_LOOP,
        .motor_reverse_flag    =  Motor_General_Def_n::MOTOR_DIRECTION_NORMAL,
        .feedback_reverse_flag =  Motor_General_Def_n::FEEDBACK_DIRECTION_NORMAL,
        .speed_feedback_source =  Motor_General_Def_n::MOTOR_FEED,
        .feedforward_flag      =  Motor_General_Def_n::FEEDFORWARD_NONE,
      };

Motor_General_Def_n::Motor_Init_Config_s wheelconfig = {
      .controller_setting_init_config = motor_setting,
      .motor_type = Motor_General_Def_n::M3508,
      .can_init_config = { 
        .can_handle = &hcan1,
      }
  };

wheelconfig.can_init_config.tx_id = LF_ID;
wheelmotor_config[0] = wheelconfig;
wheelconfig.can_init_config.tx_id = RF_ID;
wheelmotor_config[1] = wheelconfig;
wheelconfig.can_init_config.tx_id = LB_ID;
wheelmotor_config[2] = wheelconfig;
wheelconfig.can_init_config.tx_id = RB_ID;
wheelmotor_config[3] = wheelconfig;

//底盘接收yaw电机的反馈数据，但是不进行控制，用于获得偏航角，所以不需要过多配置
Motor_General_Def_n::Motor_Init_Config_s yawconfig = {
      .motor_type = Motor_General_Def_n::GM6020,
      .can_init_config = { 
        .can_handle = &hcan2,
        .tx_id = 3,
      },
      .zero_offset = YAW_OFFSET,  //初始偏移角度（M6020）是单圈编码，目前安装零点方向与主控方向差角是114.0度
  };

//底盘电机速度模式pid
pid_alg_n::PID_Init_Config_t motor_speed_pid_cof = {
    .Kp = CHASSIS_MOTOR_KP,
    .Ki = CHASSIS_MOTOR_KI,
    .Kd = CHASSIS_MOTOR_KD,
    .Kfa = 75.0f,
    .Kfb = 0.0f,
    .ActualValueSource = NULL,
    .mode = Output_Limit | Feedforward | OutputFilter,
    .max_out = MOTOR_3508_CURRENT_LIMIT,
    .out_filter_num = CHASSIS_MOTOR_OUT_FILITER,
    };

//底盘电机自锁模式pid
pid_alg_n::PID_Init_Config_t motor_lock_pid_cof = {
    .Kp = CHASSIS_MOTOR_LOCK_KP,
    .Ki = CHASSIS_MOTOR_LOCK_KI,
    .Kd = CHASSIS_MOTOR_LOCK_KD,
    .ActualValueSource = NULL,
    .mode = Output_Limit ,
    .max_out = MOTOR_3508_CURRENT_LIMIT/2,
    };

//底盘速度PID配置(x,y方向)
pid_alg_n::PID_Init_Config_t speed_pid_cof = {
    .Kp = CHASSIS_SPEED_KP,
    .Ki = CHASSIS_SPEED_KI,
    .Kd = CHASSIS_SPEED_KD,
    .ActualValueSource = NULL,
    .mode = Output_Limit | StepIn | OutputFilter,
    .max_out = CHASSIS_SPEED_XY_MAX,
    .out_filter_num = CHASSIS_FIRST_ORDER_FILTER_K,
    .stepIn = CHASSIS_STEPIN_K
    };

//底盘旋转PID配置z
pid_alg_n::PID_Init_Config_t z_pid_cof = {
    .Kp = CHASSIS_SPEED_Z_KP,
    .Ki = CHASSIS_SPEED_Z_KI,
    .Kd = CHASSIS_SPEED_Z_KD,
    .ActualValueSource = NULL,
    .mode = Output_Limit | StepIn,
    .max_out = CHASSIS_SPEED_Z_MAX,
    .stepIn = CHASSIS_Z_STEPIN_K
    };

//底盘跟随角度环PID配置
pid_alg_n::PID_Init_Config_t follow_p_pid_cof = {
    .Kp = CHASSIS_FOLLOW_P_KP,
    .Ki = CHASSIS_FOLLOW_P_KI,
    .Kd = CHASSIS_FOLLOW_P_KD,
    .ActualValueSource = NULL,
    .mode = Output_Limit|Deadzone|Integral_Limit|OutputFilter,
    .max_out = CHASSIS_FOLLOW_P_MAX,
    .max_Ierror = CHASSIS_FOLLOW_IMAX,
    .deadband = CHASSIS_FOLLOW_P_DEADZONE,
    .out_filter_num = 0.5
    };

//底盘跟随速度环PID配置
pid_alg_n::PID_Init_Config_t follow_s_pid_cof = {
    .Kp = CHASSIS_FOLLOW_S_KP,
    .Ki = CHASSIS_FOLLOW_S_KI,
    .Kd = CHASSIS_FOLLOW_S_KD,
    .Kfa = CHASSIS_FOLLOW_S_KF,
    .ActualValueSource = NULL,
    .mode = OutputFilter|Feedforward,
    .out_filter_num = CHASSIS_FOLLOW_S_FILTER,
    };

    //底盘指针初始化
    dt7 = DR16_n::DR16_c::dr16_->GetClassPtr()->GetDataStructPtr();
    referee = Get_referee_Address();
    //电机初始化
    wheel_motor[LF] = new DJI_Motor_n::DJI_Motor_Instance(wheelmotor_config[0]);
    wheel_motor[RF] = new DJI_Motor_n::DJI_Motor_Instance(wheelmotor_config[1]);
    wheel_motor[LB] = new DJI_Motor_n::DJI_Motor_Instance(wheelmotor_config[2]);
    wheel_motor[RB] = new DJI_Motor_n::DJI_Motor_Instance(wheelmotor_config[3]);
    yaw_motor = new DJI_Motor_n::DJI_Motor_Instance(yawconfig);
    //电机PID算法初始化
    motor_speed_pid[RF] = new pid_alg_n::pid_alg_c(motor_speed_pid_cof);
    motor_speed_pid[RB] = new pid_alg_n::pid_alg_c(motor_speed_pid_cof);
    motor_speed_pid[LB] = new pid_alg_n::pid_alg_c(motor_speed_pid_cof);
    motor_speed_pid[LF] = new pid_alg_n::pid_alg_c(motor_speed_pid_cof);
    motor_lock_pid[RF] = new pid_alg_n::pid_alg_c(motor_lock_pid_cof);
    motor_lock_pid[RB] = new pid_alg_n::pid_alg_c(motor_lock_pid_cof);
    motor_lock_pid[LB] = new pid_alg_n::pid_alg_c(motor_lock_pid_cof);
    motor_lock_pid[LF] = new pid_alg_n::pid_alg_c(motor_lock_pid_cof);
    //底盘控制算法初始化
    pid_chassis_x = new pid_alg_n::pid_alg_c(speed_pid_cof);
    pid_chassis_y = new pid_alg_n::pid_alg_c(speed_pid_cof);
    pid_chassis_z = new pid_alg_n::pid_alg_c(z_pid_cof);
    speedX_filter = new filter_alg_n::first_order_filter_c(CHASSIS_FILTER_K);
    speedY_filter = new filter_alg_n::first_order_filter_c(CHASSIS_FILTER_K);
    #if FOLLOW_CONTRAL_MODE==PID
    pid_follow = new Motor_General_Def_n::Motor_Controller_c();
    pid_follow->speed_PID.pid_init(follow_s_pid_cof);
    pid_follow->angle_PID.pid_init(follow_p_pid_cof);
    #else //lqr更加丝滑
    float k[2] = {CHASSIS_FOLLOW_LQR_K1,CHASSIS_FOLLOW_LQR_K2};
    lqr_follow = new lqr_alg_n::lqr_alg_c(2,1,k);
    #endif
    
    dt7->rc.s1 = RC_SW_DOWN;
    dt7->rc.s2 = RC_SW_DOWN;
    dt7->kb.key_code = 0;    //有时候会误触发小陀螺
    mode = MANNUL;
    state = ZERO_FORCE;
    have_deline = 0;

    wheel_motor[LF]->DJIMotorEnable();
    wheel_motor[LB]->DJIMotorEnable();
    wheel_motor[RF]->DJIMotorEnable();
    wheel_motor[RB]->DJIMotorEnable();
    yaw_motor->DJIMotorOnlyRecevie();
}


/**
 * @brief          底盘模式设定
 * @retval         none
 */
void chassis_ctrl_c::chassis_mode_set()
{
    //用于记录上一次数据
    Chassis_Mode_e last_behaviour;
    static Chassis_Mode_e rc_behaviour = MANNUL;
    static Chassis_Mode_e kb_behaviour = MANNUL;

    static Chassis_Mode_e last_chassis_mode;

    //非锁定状态如果真实速度小则设置为锁定，锁定状态如果设定速度大则启动
    if(this->state!=LOCK)
    {
        if (((abs(this->ChassisVelocity.x) < 1 && abs(this->ChassisVelocity.y) < 1 )) //
                && ((abs(this->wheel_motor[LB]->MotorMeasure.measure.feedback_speed) + abs(this->wheel_motor[LF]->MotorMeasure.measure.feedback_speed) +
                    abs(this->wheel_motor[RB]->MotorMeasure.measure.feedback_speed) + abs(this->wheel_motor[RF]->MotorMeasure.measure.feedback_speed)) < 500))//500
        {
            this->state=LOCK;
        }
        else
        {
            this->state=SPEED;
        }

    }
    //锁定状态如果设置速度较大，则退出锁定
    else
    {
        if((abs(wheel_data[RF].motor_set_ref) + abs(wheel_data[RB].motor_set_ref) + abs(wheel_data[LB].motor_set_ref) + abs(wheel_data[LF].motor_set_ref)) > 1000)
        {
            this->state=SPEED;
        }
    }
    //遥控器设置底盘无力状态
    if(this->dt7->rc.s2==RC_SW_DOWN||this->dt7->rc.s2==RC_SW_ERROR)
    {
        this->state=ZERO_FORCE;
    }
    
    
    
    //手柄
    last_behaviour = rc_behaviour;
    switch(this->dt7->rc.s1)
    {
        case RC_SW_UP:
            rc_behaviour=ROTATION;
        break;
        
        case RC_SW_MID:
            rc_behaviour=FOLLOW;
        break;
        
        case RC_SW_DOWN:
        default:
            rc_behaviour=MANNUL;
        break;
    }

    #if NAVIGATION_MODE==1
    if(this->dt7->rc.s2==RC_SW_UP)
        rc_behaviour=NAVIGATION;   //优先级最高
    #endif

    if (last_behaviour != rc_behaviour) 
    {
        this->mode = rc_behaviour;
        if(this->mode==ROTATION)
            rotate_speed =- rotate_speed; //每次小陀螺方向取反
    }

    //键鼠
    last_behaviour = kb_behaviour;
    //**Q —— 小陀螺
    if (this->dt7->kb.bit.Q)    kb_behaviour = ROTATION; //小陀螺
   
    //**E —— 跟随模式
    if (this->dt7->kb.bit.E)    kb_behaviour = FOLLOW; //跟随

    //**R —— 取消跟随
    if (this->dt7->kb.bit.R)    kb_behaviour = MANNUL; //不跟随
    
    if (this->dt7->kb.bit.M)
    {
        //kb_behaviour = CRAZY; //不跟随，变速陀螺（比赛违规）
    }
    if (last_behaviour != kb_behaviour)
    {
        this->mode=kb_behaviour;
        if(this->mode==ROTATION)
            rotate_speed=-rotate_speed;
    }

    if(have_deline > 1)  //掉线数量超过两个，采取非常措施(这个时候只有前进)
    {
        this->mode = FOLLOW;
        last_behaviour = FOLLOW;
    }
    
    
    if(last_chassis_mode!=this->mode)
    {
        this->pid_chassis_x->ECF_PID_CLEAR();
        this->pid_chassis_y->ECF_PID_CLEAR();
        this->pid_chassis_z->ECF_PID_CLEAR();
        #if FOLLOW_CONTRAL_MODE==PID
        this->pid_follow->speed_PID.ECF_PID_CLEAR();
        this->pid_follow->angle_PID.ECF_PID_CLEAR();
        #else
        this->lqr_follow->ECF_LQR_Data_Clear();
        #endif
        memset(&this->Vset_gimbal,0,sizeof(chassis_ctrl_n::V_t));
        memset(&this->Vset_chassis,0,sizeof(chassis_ctrl_n::V_t));
        memset(&this->ChassisVelocity,0,sizeof(this->ChassisVelocity));
    }
    
    last_chassis_mode = this->mode;
}


/**
 * @brief          底盘操控设定
 * @retval         none
 */
void chassis_ctrl_c::Chassis_SetBehavior()
{

    /***设定量更新***/  //跟新与底盘的差角
    this->chassis_yaw_diff_angle = user_maths_c().loop_fp32_constrain(this->yaw_motor->MotorMeasure.measure.relative_angle,-180.0f,180.0f);//yaw轴电机反馈 （初始化时已减去偏移量）
    this->Vset_gimbal.x = (float)(this->dt7->rc.ch[2]) * REMOTE_TO_SPEED + (this->dt7->kb.bit.D-this->dt7->kb.bit.A)*CHASSIS_NORMAL_SPEED;   
    this->Vset_gimbal.y = (float)(this->dt7->rc.ch[3]) * REMOTE_TO_SPEED + (this->dt7->kb.bit.W-this->dt7->kb.bit.S)*CHASSIS_NORMAL_SPEED; 
    
    //微操模式(1m/s)
    if(this->dt7->kb.bit.CTRL==1)
    {   
        this->Vset_gimbal.x *=0.5f;   
        this->Vset_gimbal.y *=0.5f;        
    }
    
    //提速(4m/s)
    if(this->dt7->kb.bit.SHIFT==1)
    {   
        this->Vset_gimbal.x *=2.0f;   
        this->Vset_gimbal.y *=2.0f;        
    }
    /***************/
   
   //根据模式设定旋转速度z
    switch (this->mode)
    {
        case FOLLOW:
            if(have_deline <= 1) 
            {
                #if DOUBLEHEAD==1
                //跟灯条
                #if FOLLOW_LIGHT==1
                //获取需要旋转的角度
                if(user_abs(this->chassis_yaw_diff_angle) >= 90.0f)
                {
                    this->Vset_chassis.z_pos  = -math_.float_min_distance(180.0f,math_.loop_fp32_constrain(this->chassis_yaw_diff_angle,0.0f,360.0f),-180.0f,180.0f);
                }
                else
                {
                    this->Vset_chassis.z_pos  = this->chassis_yaw_diff_angle;
                }
                #else
                //跟侧面
                if(math_.loop_fp32_constrain(this->chassis_yaw_diff_angle,0.0f,360.0f)>= 180.0f)
                {
                    this->Vset_chassis.z_pos =-math_.float_min_distance(270,math_.loop_fp32_constrain(this->chassis_yaw_diff_angle,0.0f,360.0f),-180.0f,180.0f);
                }    
                else
                {
                    this->Vset_chassis.z_pos =-math_.float_min_distance(90,math_.loop_fp32_constrain(this->chassis_yaw_diff_angle,0.0f,360.0f),-180.0f,180.0f);
                }
                #endif
                #else
                if(math_.loop_fp32_constrain(this->chassis_yaw_diff_angle,0.0f,360.0f)>= 315.0f)
                {
                    this->Vset_chassis.z_pos =-math_.float_min_distance(360,math_.loop_fp32_constrain(this->chassis_yaw_diff_angle,0.0f,360.0f),-180.0f,180.0f);
                }
                else if(math_.loop_fp32_constrain(this->chassis_yaw_diff_angle,0.0f,360.0f)>= 225.0f)
                {
                    this->Vset_chassis.z_pos =-math_.float_min_distance(270,math_.loop_fp32_constrain(this->chassis_yaw_diff_angle,0.0f,360.0f),-180.0f,180.0f);
                }
                else if(math_.loop_fp32_constrain(this->chassis_yaw_diff_angle,0.0f,360.0f)>= 135.0f)
                {
                    this->Vset_chassis.z_pos =-math_.float_min_distance(180,math_.loop_fp32_constrain(this->chassis_yaw_diff_angle,0.0f,360.0f),-180.0f,180.0f);
                }
                else if(math_.loop_fp32_constrain(this->chassis_yaw_diff_angle,0.0f,360.0f)>= 45.0f)
                {
                    this->Vset_chassis.z_pos =-math_.float_min_distance(90,math_.loop_fp32_constrain(this->chassis_yaw_diff_angle,0.0f,360.0f),-180.0f,180.0f);
                }
                else
                {
                    this->Vset_chassis.z_pos = this->chassis_yaw_diff_angle;
                }
                #endif
            }
            else if(have_deline == 2)  //两个轮子掉线
            {
                int deline_id = 0;
                for(auto id:deline_motor)
                    deline_id += id;
                this->Vset_chassis.z_pos  = -math_.float_min_distance(WHEEL_ID_FIND_ANGLE(deline_id/2),math_.loop_fp32_constrain(this->chassis_yaw_diff_angle,0.0f,360.0f),-180.0f,180.0f);
            }
        break;
        case MANNUL:
            this->ChassisVelocity.z=0;
        break;
      
        case ROTATION:
            this->Vset_chassis.z_spd = RPM_TO_RAD(rotate_speed);
        break;  

        case NAVIGATION:
            this->Vset_gimbal.x = navigation_p->x_speed;
            this->Vset_gimbal.y = navigation_p->y_speed;
            this->Vset_chassis.z_spd = navigation_p->z_speed;

        break;
        default:
            chassis_zeroForce_set();
        break;
    }
    gimbalToChassisCoord();
}

/**
 * @brief          底盘无力模式设定
 * @retval         none
 */
void chassis_ctrl_c::chassis_zeroForce_set()
{
    memset(&this->Vset_chassis,0,sizeof(chassis_ctrl_n::V_t));
}


/**
 * @brief          底盘速度分解
 * @retval         none 
 */
void chassis_ctrl_c::gimbalToChassisCoord()
{
     const float rotate_ff_z=-0.0060f; //固定的前馈增益，大了往右，小了往左（指的是小陀螺前进时会发生左右偏）
     //以云台为主，将云台坐标分解到底盘
    
    //此处更改步进值用于限制 起步/刹停 加速度控制机体平稳
    if(this->mode==ROTATION)
    {
        this->pid_chassis_x->ECF_PID_Reset_StepIn(0.03f);
        this->pid_chassis_y->ECF_PID_Reset_StepIn(0.03f);
    }
    else
    {
        float step_x,step_y;
        step_x = user_abs(this->Vset_gimbal.x) * 0.012f;
        step_y = user_abs(this->Vset_gimbal.y) * 0.012f;
        user_value_limit(step_x,0.01f,0.05f);
        user_value_limit(step_y,0.01f,0.05f);
        this->pid_chassis_x->ECF_PID_Reset_StepIn(step_x);
        this->pid_chassis_y->ECF_PID_Reset_StepIn(step_y); 
    }


    float Gimbal_x = this->pid_chassis_x->ECF_PID_Calculate(this->Vset_gimbal.x,0.0f); //实际值为0，斜坡函数
    float Gimbal_y = this->pid_chassis_y->ECF_PID_Calculate(this->Vset_gimbal.y,0.0f); //实际值为0
    

    //将云台速度分解到底盘
    if(this->mode==FOLLOW||this->mode==MANNUL)
    {
        this->Vset_chassis.y =   Gimbal_y * math_.cos_calculate( math_.loop_fp32_constrain((this->chassis_yaw_diff_angle),-180.0f,180.0f))
                               + Gimbal_x * math_.sin_calculate( math_.loop_fp32_constrain((this->chassis_yaw_diff_angle),-180.0f,180.0f));
        this->Vset_chassis.x = - Gimbal_y * math_.sin_calculate( math_.loop_fp32_constrain((this->chassis_yaw_diff_angle),-180.0f,180.0f))
                               + Gimbal_x * math_.cos_calculate( math_.loop_fp32_constrain((this->chassis_yaw_diff_angle),-180.0f,180.0f));
    }
    else//小陀螺前馈计算角度，防止小陀螺移动时偏移方向
    {
        float current_spd=((float)(abs(this->wheel_motor[LB]->MotorMeasure.measure.feedback_speed)+abs(this->wheel_motor[LF]->MotorMeasure.measure.feedback_speed)
                         +abs(this->wheel_motor[RB]->MotorMeasure.measure.feedback_speed)+abs(this->wheel_motor[RF]->MotorMeasure.measure.feedback_speed))/4);
        if(rotate_speed < 0)
            current_spd=-current_spd;
        this->Vset_chassis.y =   Gimbal_y * math_.cos_calculate( math_.loop_fp32_constrain((this->chassis_yaw_diff_angle-current_spd*rotate_ff_z),-180.0f,180.0f))
                               + Gimbal_x * math_.sin_calculate( math_.loop_fp32_constrain((this->chassis_yaw_diff_angle-current_spd*rotate_ff_z),-180.0f,180.0f));
        this->Vset_chassis.x = - Gimbal_y * math_.sin_calculate( math_.loop_fp32_constrain((this->chassis_yaw_diff_angle-current_spd*rotate_ff_z),-180.0f,180.0f))
                               + Gimbal_x * math_.cos_calculate( math_.loop_fp32_constrain((this->chassis_yaw_diff_angle-current_spd*rotate_ff_z),-180.0f,180.0f));
    }
}

/**
 * @brief          底盘输出计算
 * @retval         none
 */
void chassis_ctrl_c::Chassis_Act()
{
    chassis_speed_pid_calculate();
    chassis_motion_decomposition();
    motor_speed_pid_calculate();
    #if POWER_LIMIT==LIMIT_CURRENT
    //更新电容值与最大功率限制值
    // cap_remain=Calc_SuperPower_energy(supercap.output_voltage);
    // max_power=Calc_SuperPower_Out_power_limit(chassis_work_t,cap_remain);//TODO:按住shift//min取最小限制功率
    float max_power = referee->Robot_Status.chassis_power_limit;
    Chassis_power_limit(max_power);//根据功率限制重设电调电流
    #endif
    //给定电机设定值
    this->wheel_motor[RF]->DJIMotorSetRef(wheel_data[RF].motor_give_current);
    this->wheel_motor[LF]->DJIMotorSetRef(wheel_data[LF].motor_give_current);
    this->wheel_motor[RB]->DJIMotorSetRef(wheel_data[RB].motor_give_current);
    this->wheel_motor[LB]->DJIMotorSetRef(wheel_data[LB].motor_give_current);
}

/**
 * @brief          底盘pid速度计算
 * @retval         none
 */
void chassis_ctrl_c::chassis_speed_pid_calculate()
{
    //平滑滤波
    this->Vset_chassis.x = this->speedX_filter->first_order_filter(this->Vset_chassis.x);
    this->Vset_chassis.y = this->speedY_filter->first_order_filter(this->Vset_chassis.y);
    
    if(this->mode==ROTATION||this->mode==CRAZY||this->mode==NAVIGATION)
    {
        this->ChassisVelocity.z = -this->pid_chassis_z->ECF_PID_Calculate(this->Vset_chassis.z_spd,0.0f);//实际值为0
    }
    else if(this->mode==FOLLOW)
    {   
        float temp_set = 0.0f;
        //将需要旋转的角度转换到分解速度上   
        #if FOLLOW_CONTRAL_MODE==PID
        temp_set = this->pid_follow->angle_PID.ECF_PID_Calculate(this->Vset_chassis.z_pos); //实际值为0
        temp_set = this->pid_follow->speed_PID.ECF_PID_Calculate(temp_set);  //实际值为0
        #else
        if(abs(this->Vset_chassis.z_pos) < CHASSIS_FOLLOW_P_DEADZONE)    this->Vset_chassis.z_pos = 0;
    
        float Follow_system_state[2] = {this->Vset_chassis.z_pos,this->yaw_motor->MotorMeasure.measure.feedback_speed};
	    lqr_follow->ECF_LQR_Data_Update(Follow_system_state);
        temp_set = lqr_follow->ECF_LQR_Calculate();
        #endif
        this->ChassisVelocity.z = temp_set;  
    }
    
    this->ChassisVelocity.x = this->Vset_chassis.x;
    this->ChassisVelocity.y = this->Vset_chassis.y;
    {//防失真
        double div= chassis_motion_speed_distortion(this->ChassisVelocity.x,this->ChassisVelocity.y,this->ChassisVelocity.z);
        this->ChassisVelocity.x*=div;
        this->ChassisVelocity.y*=div;
        this->ChassisVelocity.z*=div;
    }
#if POWER_LIMIT==LIMIT_SPEED  //TODO
    float max_speed=5;
    //速度限制曲线：max_speed=17(v-10)³+1000;[10,17]
    if(supercap.output_voltage<16&&supercap.output_voltage>10)//超电电压过小,限制输出
    {
       max_speed=16*(supercap.output_voltage-10)*(supercap.output_voltage-10)*(supercap.output_voltage-10)+1000;
       float tmp=-max_speed;
       value_limit(this->ChassisVelocity.x,tmp,max_speed);
       value_limit(this->ChassisVelocity.y,tmp,max_speed);
       value_limit(this->ChassisVelocity.z,tmp,max_speed);
    }
    else if(supercap.output_voltage<10)
    {
        max_speed=16*(supercap.output_voltage)*(supercap.output_voltage);
        float tmp=-max_speed;
        value_limit(this->ChassisVelocity.x,tmp,max_speed);
        value_limit(this->ChassisVelocity.y,tmp,max_speed);
        value_limit(this->ChassisVelocity.z,tmp,max_speed);
    }
#endif
    
}

/**
 * @brief          防止底盘运动失真；对设定速度分量进行缩放
 * @param[in]      X方向速度
 * @param[in]      Y方向速度
 * @param[in]      Z方向速度
 * @retval         失真系数
 */
float chassis_ctrl_c::chassis_motion_speed_distortion(float x,float y,float z)
{
    float speed_length=abs(x)+abs(y)+abs(z*CHASSIS_WIDTH);
    float div=(float)(CHASSIS_SPEED_MAX/abs(speed_length));
    if(abs(speed_length>CHASSIS_SPEED_MAX))  return div;//失真
    else return 1; 
}


/**
 * @brief          全向轮底盘速度分解
 * @retval         none
 */
void chassis_ctrl_c::chassis_motion_decomposition()
{
    float speed_cal[4];

    //检测掉线轮子
    for(int i = RF;i <= LF;i++)
    {
        if(this->wheel_motor[i]->Get_Lost_Status() == true && deline_motor[i] == 0)
        {
            deline_motor[i] = 1;
            have_deline++;
        }
        else if(this->wheel_motor[i]->Get_Lost_Status() == false && deline_motor[i] == 1)
        {
            //恢复在线状态
            deline_motor[i] = 0;
            have_deline--;
        }
    }

    switch(have_deline)
    {
        case 0:
        // 计算每个轮子的线速度
        speed_cal[0] = (-this->ChassisVelocity.x - this->ChassisVelocity.y) / sqrt_2
            + CHASSIS_WIDTH * this->ChassisVelocity.z;
        speed_cal[1] = (+this->ChassisVelocity.x - this->ChassisVelocity.y) / sqrt_2
            + CHASSIS_WIDTH * this->ChassisVelocity.z;
        speed_cal[2] = (+this->ChassisVelocity.x + this->ChassisVelocity.y) / sqrt_2
            + CHASSIS_WIDTH * this->ChassisVelocity.z;
        speed_cal[3] = (-this->ChassisVelocity.x + this->ChassisVelocity.y) / sqrt_2
            + CHASSIS_WIDTH * this->ChassisVelocity.z;

        //转换成rpm
        this->wheel_data[LF].motor_set_ref = speed_cal[0] / CHASSIS_WHEEL_WIDTH * (60 / (2 * PI)) * M3508_RATIO;
        this->wheel_data[LB].motor_set_ref = speed_cal[1] / CHASSIS_WHEEL_WIDTH * (60 / (2 * PI)) * M3508_RATIO;
        this->wheel_data[RB].motor_set_ref = speed_cal[2] / CHASSIS_WHEEL_WIDTH * (60 / (2 * PI)) * M3508_RATIO;
        this->wheel_data[RF].motor_set_ref = speed_cal[3] / CHASSIS_WHEEL_WIDTH * (60 / (2 * PI)) * M3508_RATIO;
        break;

        case 1:
        float motor_setspd[WHEEL_NUM];
        // 获取掉线轮电机
        int8_t afk_motor; // 获取掉线电机
        for(int i = RF;i <= LF;i++)
        {
            if(deline_motor[i] == 1)    
            {
                afk_motor = i;
                break;
            }
        }
        // 设定相邻轮y方向分量
        if (afk_motor < 2)
        {
            motor_setspd[abs(afk_motor - 1)] = this->ChassisVelocity.y;
            afk_motor += 2;
            motor_setspd[afk_motor] = -this->ChassisVelocity.y + this->ChassisVelocity.x * (((afk_motor - 2) == 0) ? 1 : -1);
            //motor_setspd[afk_motor] -=this->ChassisVelocity.z;
            motor_setspd[abs(afk_motor - 3) + 2] = this->ChassisVelocity.x * (((afk_motor - 2) == 0) ? -1 : 1);
        }
        else
        {
            motor_setspd[abs(afk_motor - 3) + 2] = -this->ChassisVelocity.y;
            afk_motor -= 2;
            motor_setspd[afk_motor] = this->ChassisVelocity.y + this->ChassisVelocity.x * (((afk_motor) == 1) ? 1 : -1);
            //motor_setspd[afk_motor] -=this->ChassisVelocity.z;
            motor_setspd[abs(afk_motor - 1)] = this->ChassisVelocity.x * (((afk_motor) == 1) ? -1 : 1);
        }
        speed_cal[0] = motor_setspd[0]/sqrt_2 + CHASSIS_WIDTH * this->ChassisVelocity.z;
        speed_cal[1] = motor_setspd[1]/sqrt_2 + CHASSIS_WIDTH * this->ChassisVelocity.z;
        speed_cal[2] = motor_setspd[2]/sqrt_2 + CHASSIS_WIDTH * this->ChassisVelocity.z;
        speed_cal[3] = motor_setspd[3]/sqrt_2 + CHASSIS_WIDTH * this->ChassisVelocity.z;

        this->wheel_data[RF].motor_set_ref = speed_cal[0] / CHASSIS_WHEEL_WIDTH * (60 / (2 * PI)) * M3508_RATIO;
        this->wheel_data[RB].motor_set_ref = speed_cal[1] / CHASSIS_WHEEL_WIDTH * (60 / (2 * PI)) * M3508_RATIO;
        this->wheel_data[LB].motor_set_ref = speed_cal[2] / CHASSIS_WHEEL_WIDTH * (60 / (2 * PI)) * M3508_RATIO;
        this->wheel_data[LF].motor_set_ref = speed_cal[3] / CHASSIS_WHEEL_WIDTH * (60 / (2 * PI)) * M3508_RATIO;
        break;

        case 2:
        this->wheel_data[RF].motor_set_ref *= 2;
        this->wheel_data[RB].motor_set_ref *= 2;
        this->wheel_data[LB].motor_set_ref *= 2;
        this->wheel_data[LF].motor_set_ref *= 2;
        default: break;
    }

    // this->wheel_data[LF].motor_set_ref = ( -  this->ChassisVelocity.x - this->ChassisVelocity.y  + this->ChassisVelocity.z);//v1
    // this->wheel_data[LB].motor_set_ref = ( +  this->ChassisVelocity.x - this->ChassisVelocity.y  + this->ChassisVelocity.z);//v2   
    // this->wheel_data[RB].motor_set_ref = ( +  this->ChassisVelocity.x + this->ChassisVelocity.y  + this->ChassisVelocity.z);//v3
    // this->wheel_data[RF].motor_set_ref = ( -  this->ChassisVelocity.x + this->ChassisVelocity.y  + this->ChassisVelocity.z);//v4
    
    float divisor = 1.0f;
    float temp_max_spd = 0;
    temp_max_spd = user_max(user_max(abs(this->wheel_data[RF].motor_set_ref), abs(this->wheel_data[RB].motor_set_ref)), user_max(abs(this->wheel_data[LB].motor_set_ref), abs(this->wheel_data[LF].motor_set_ref)));
    if (temp_max_spd > MAX_MOTOR_SPEED) // 任一轮子速度大于电机最大速度
    {
        divisor = MAX_MOTOR_SPEED / temp_max_spd;
    }

    this->wheel_data[LF].motor_set_ref *= divisor;
    this->wheel_data[LB].motor_set_ref *= divisor;
    this->wheel_data[RB].motor_set_ref *= divisor;
    this->wheel_data[RF].motor_set_ref *= divisor;

}


/**
 * @brief          底盘电机速度pid计算
 * @retval         none
 */
void chassis_ctrl_c::motor_speed_pid_calculate()
{
    static chassis_ctrl_n::Chassis_State_e last_state = this->state;
    if(last_state != this->state)
    {
        this->wheel_motor[LB]->MotorMeasure.MeasureClear();
        this->wheel_motor[RB]->MotorMeasure.MeasureClear();
        this->wheel_motor[LF]->MotorMeasure.MeasureClear();
        this->wheel_motor[RF]->MotorMeasure.MeasureClear();
        if(this->state == SPEED)
        {
            motor_speed_pid[RF]->ECF_PID_CLEAR();
            motor_speed_pid[RB]->ECF_PID_CLEAR();
            motor_speed_pid[LB]->ECF_PID_CLEAR();
            motor_speed_pid[LF]->ECF_PID_CLEAR();
        }
        else if(this->state == LOCK)
        {
            motor_lock_pid[RF]->ECF_PID_CLEAR();
            motor_lock_pid[RB]->ECF_PID_CLEAR();
            motor_lock_pid[LB]->ECF_PID_CLEAR();
            motor_lock_pid[LF]->ECF_PID_CLEAR();
        }
        if(last_state == ZERO_FORCE)  //重新使能
        {
            wheel_motor[LF]->DJIMotorEnable();
            wheel_motor[LB]->DJIMotorEnable();
            wheel_motor[RF]->DJIMotorEnable();
            wheel_motor[RB]->DJIMotorEnable();
        }
    }
    last_state = this->state;

    switch(this->state)
    {
        case SPEED:  //切换为速度环移动
            wheel_data[RF].motor_give_current = motor_speed_pid[RF]->ECF_PID_Calculate(wheel_data[RF].motor_set_ref, wheel_motor[RF]->MotorMeasure.measure.feedback_speed);
            wheel_data[RB].motor_give_current = motor_speed_pid[RB]->ECF_PID_Calculate(wheel_data[RB].motor_set_ref, wheel_motor[RB]->MotorMeasure.measure.feedback_speed);
            wheel_data[LB].motor_give_current = motor_speed_pid[LB]->ECF_PID_Calculate(wheel_data[LB].motor_set_ref, wheel_motor[LB]->MotorMeasure.measure.feedback_speed);
            wheel_data[LF].motor_give_current = motor_speed_pid[LF]->ECF_PID_Calculate(wheel_data[LF].motor_set_ref, wheel_motor[LF]->MotorMeasure.measure.feedback_speed);
        break;
        
        case LOCK: //切换为角度环控制锁死
            wheel_data[RF].motor_give_current = motor_lock_pid[RF]->ECF_PID_Calculate(0, wheel_motor[RF]->MotorMeasure.measure.record_ecd);
            wheel_data[RB].motor_give_current = motor_lock_pid[RB]->ECF_PID_Calculate(0, wheel_motor[RB]->MotorMeasure.measure.record_ecd);
            wheel_data[LB].motor_give_current = motor_lock_pid[LB]->ECF_PID_Calculate(0, wheel_motor[LB]->MotorMeasure.measure.record_ecd);
            wheel_data[LF].motor_give_current = motor_lock_pid[LF]->ECF_PID_Calculate(0, wheel_motor[LF]->MotorMeasure.measure.record_ecd);
        break;
        
        case ZERO_FORCE:
        default:
            this->wheel_motor[RF]->DJIMotorStop();
            this->wheel_motor[LF]->DJIMotorStop();
            this->wheel_motor[RB]->DJIMotorStop();
            this->wheel_motor[LB]->DJIMotorStop();
        break;
        
    }
}



float k2 = 1.90000804e-07;                     // k2
float K_1 = 1.60300004e-07;                    // k1  已经除 K
void chassis_ctrl_c::Chassis_power_limit(uint8_t Power_Limit_Val)
{
    power_ctrl.rever[0] = 1;
    power_ctrl.rever[1] = 1;
    power_ctrl.rever[2] = 1;
    power_ctrl.rever[3] = 1;
        
    float P_C620 = 1.00f;//电调功率损耗
    float K = 1.99688994e-6f; // (20/16384)  电调电流与真实电流比     *(0.3)*(187/3591)/9.55   M3509扭矩比例参数   电调值转化为力矩


    /*---统一取真实方便运算---*/


    power_ctrl.give_current[RF] = wheel_data[RF].motor_give_current;
    power_ctrl.give_current[RB] = wheel_data[RB].motor_give_current;
    power_ctrl.give_current[LB] = wheel_data[LB].motor_give_current;
    power_ctrl.give_current[LF] = wheel_data[LF].motor_give_current;

    for (int i = RF; i <= LF; i++)
    {
        if (power_ctrl.give_current[i] < 0)
        {
            power_ctrl.rever[i] = -1;
            power_ctrl.give_current[i] *= power_ctrl.rever[i];
        }        
    }

    /*---转子转速---*/
    //前走 L正 R负
    power_ctrl.speed[RF] = this->wheel_motor[RF]->MotorMeasure.measure.feedback_speed * power_ctrl.rever[RF];
    power_ctrl.speed[RB] = this->wheel_motor[RB]->MotorMeasure.measure.feedback_speed * power_ctrl.rever[RB];
    power_ctrl.speed[LB] = this->wheel_motor[LB]->MotorMeasure.measure.feedback_speed * power_ctrl.rever[LB];
    power_ctrl.speed[LF] = this->wheel_motor[LF]->MotorMeasure.measure.feedback_speed * power_ctrl.rever[LF];
    /*---当前计算出的力矩电流---*/
    power_ctrl.I_out[RF] = this->wheel_motor[RF]->MotorMeasure.measure.feedback_real_current * power_ctrl.rever[RF];
    power_ctrl.I_out[RB] = this->wheel_motor[RB]->MotorMeasure.measure.feedback_real_current * power_ctrl.rever[RB];
    power_ctrl.I_out[LB] = this->wheel_motor[LB]->MotorMeasure.measure.feedback_real_current * power_ctrl.rever[LB];
    power_ctrl.I_out[LF] = this->wheel_motor[LF]->MotorMeasure.measure.feedback_real_current * power_ctrl.rever[LF];
    /*---输出功率---*/
    power_ctrl.P_out[RF] = K * power_ctrl.I_out[RF] * power_ctrl.speed[RF];
    power_ctrl.P_out[RB] = K * power_ctrl.I_out[RB] * power_ctrl.speed[RB];
    power_ctrl.P_out[LB] = K * power_ctrl.I_out[LB] * power_ctrl.speed[LB];
    power_ctrl.P_out[LF] = K * power_ctrl.I_out[LF] * power_ctrl.speed[LF];
    /*---输入功率---*/
    power_ctrl.P_in[RF] = power_ctrl.P_out[RF] + K_1 * power_ctrl.speed[RF]*power_ctrl.speed[RF] + k2 * power_ctrl.I_out[RF]*power_ctrl.I_out[RF] + P_C620;
    power_ctrl.P_in[RB] = power_ctrl.P_out[RB] + K_1 * power_ctrl.speed[RB]*power_ctrl.speed[RB] + k2 * power_ctrl.I_out[RB]*power_ctrl.I_out[RB] + P_C620;
    power_ctrl.P_in[LB] = power_ctrl.P_out[LB] + K_1 * power_ctrl.speed[LB]*power_ctrl.speed[LB] + k2 * power_ctrl.I_out[LB]*power_ctrl.I_out[LB] + P_C620;
    power_ctrl.P_in[LF] = power_ctrl.P_out[LF] + K_1 * power_ctrl.speed[LF]*power_ctrl.speed[LF] + k2 * power_ctrl.I_out[LF]*power_ctrl.I_out[LF] + P_C620;
    /*---底盘输出功率总和---*/
    power_ctrl.Chassis_Power_All = 0;
    for(int i = RF;i <= LF; i++)
    {
        if ( power_ctrl.P_in[i] > 0 )  power_ctrl.Chassis_Power_All += power_ctrl.P_in[i];//忽略瞬时负功率 反充电
    }    


    if ( power_ctrl.Chassis_Power_All > Power_Limit_Val )//若总和超功率，则缩放
    {
        float divisor = Power_Limit_Val / power_ctrl.Chassis_Power_All;//缩放因子
        /*---目标输入功率等比例缩小---*/
        power_ctrl.P_in[RF] = power_ctrl.P_in[RF] * divisor;
        power_ctrl.P_in[RB] = power_ctrl.P_in[RB] * divisor;
        power_ctrl.P_in[LB] = power_ctrl.P_in[LB] * divisor;
        power_ctrl.P_in[LF] = power_ctrl.P_in[LF] * divisor;
        /*---非线性缩放计算该输入功率下的输出力矩---*/
        for (int i = RF; i <= LF; i++)
        {
            if ( power_ctrl.P_in[i] < 0 )    continue;
            float b = K * power_ctrl.speed[i];
            float c = K_1 * power_ctrl.speed[i] * power_ctrl.speed[i] - power_ctrl.P_in[i] + P_C620;
            if ( power_ctrl.give_current[i] > 0 )//维持运动状态
            {
                float temp = (-b + sqrt(b * b - 4 * k2 * c)) / (2 * k2);
                if (temp > MOTOR_3508_CURRENT_LIMIT)    power_ctrl.I_max[i] = MOTOR_3508_CURRENT_LIMIT;
                else                                    power_ctrl.I_max[i] = temp;
            }
            else //制动状态
            {
                float temp = (-b - sqrt(b * b - 4 * k2 * c)) / (2 * k2);
                if (temp > MOTOR_3508_CURRENT_LIMIT)    power_ctrl.I_max[i] = MOTOR_3508_CURRENT_LIMIT;
                else                                    power_ctrl.I_max[i] = temp;
            }
        }

        /*---更改发送给电调的数据---*/
        wheel_data[RF].motor_give_current = power_ctrl.I_max[RF] * power_ctrl.rever[RF];
        wheel_data[RB].motor_give_current = power_ctrl.I_max[RB] * power_ctrl.rever[RB];
        wheel_data[LF].motor_give_current = power_ctrl.I_max[LF] * power_ctrl.rever[LF];
        wheel_data[LB].motor_give_current = power_ctrl.I_max[LB] * power_ctrl.rever[LB];
        }
}

void chassis_ctrl_c::Set_Chassis_navigation(navigation_control_data &navigation_data)
{
    navigation_p->x_speed = navigation_data.x_speed;
    navigation_p->y_speed = navigation_data.y_speed;
    if( navigation_data.start_rotate == 1 )
        navigation_p->z_speed = CHASSIS_NAVIGATION_ROTATION_SPEED;
    else
        navigation_p->z_speed = 0;
}

//底盘控制循环
void chassis_ctrl_c::Chassis_Ctrl_Loop()
{
    chassis_mode_set();
    Chassis_SetBehavior();
    Chassis_Act();
}

chassis_ctrl_c* chassis_ctrl_c::Get_Chassis_Instance()
{
    return chassis_ctrl_c::Chassis_Instance;
}

chassis_ctrl_c* chassis_ctrl_c::Chassis_Instance = new chassis_ctrl_c();
