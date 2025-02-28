#include "fire_ctrl.hpp"
#include "gimbal_ctrl.hpp"

//底盘发来的发射机构信息
extern shoot_msg_t temp_shoot_info;

fire_ctrl_c::fire_ctrl_c():dt7(nullptr),motor_left_fire(nullptr),motor_right_fire(nullptr),motor_pluck_fire(nullptr),pid_pluck(nullptr),pid_fire(nullptr),
gimbal_pitch_dif(nullptr),gimbal_yaw_dif(nullptr),virtual_rx_data(nullptr),gimbal_mode(nullptr),predict_shoot_speed(nullptr)
{
    motor_set_value.left_motor_speed_set = motor_set_value.right_motor_speed_set = motor_set_value.pluck_motor_semi_set = motor_set_value.pluck_motor_auto_set = 0;
    static_flag.isFricReady = static_flag.isStuck = 0;
    Set_physical_Val.shoot_speed_set = FIRE_SPEED;
}

void fire_ctrl_c::Fire_Init()
{
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

    Motor_General_Def_n::Motor_Init_Config_s fire_motor_confit = {
    .controller_setting_init_config = { 
      .outer_loop_type       =  Motor_General_Def_n::OPEN_LOOP,
      .close_loop_type       =  Motor_General_Def_n::OPEN_LOOP,
      .motor_reverse_flag    =  Motor_General_Def_n::MOTOR_DIRECTION_NORMAL,
      .feedback_reverse_flag =  Motor_General_Def_n::FEEDBACK_DIRECTION_NORMAL,
      // 反馈来源可设置为 MOTOR_FEED OTHER_FEED SPEED_APS ANGULAR_SPEED LINEAR_SPEED
      .speed_feedback_source =  Motor_General_Def_n::MOTOR_FEED,
    },
    .motor_type = Motor_General_Def_n::M2006,
    .can_init_config = { 
      .can_handle = &hcan1,
      .tx_id = 3,// 看电调闪几下就填几
    }
    };

    //单发控制初始化
    pid_alg_n::PID_Init_Config_t pid_pluck_p_cof = {
    .Kp = FIRE_PLUCK_P_KP,
    .Ki = FIRE_PLUCK_P_KI,
    .Kd = FIRE_PLUCK_P_KD,
    .ActualValueSource = nullptr,
    .mode = Output_Limit,
    .max_out = FIRE_PLUCK_P_MAX
    };

    pid_alg_n::PID_Init_Config_t pid_pluck_s_cof = {
    .Kp = FIRE_PLUCK_S_KP,
    .Ki = FIRE_PLUCK_S_KI,
    .Kd = FIRE_PLUCK_S_KD,
    .ActualValueSource = nullptr,
    .mode = Output_Limit|Integral_Limit,
    .max_out = FIRE_PLUCK_S_MAX,
    .max_Ierror = FIRE_PLUCK_S_IMAX
    };

    //连发控制初始化
    pid_alg_n::PID_Init_Config_t pid_fire_cof = {
    .Kp = FIRE_FIRE_KP,
    .Ki = FIRE_FIRE_KI,
    .Kd = FIRE_FIRE_KD,
    .ActualValueSource = nullptr,
    .mode = Output_Limit,
    .max_out = PLUCK_MAX,
    };

    //初始化电机
    motor_right_fire = new DJI_Motor_n::DJI_Motor_Instance(right_fric_motor_confit);
    motor_left_fire  = new DJI_Motor_n::DJI_Motor_Instance(left_fric_motor_confit);
    motor_pluck_fire = new DJI_Motor_n::DJI_Motor_Instance(fire_motor_confit);

    DR16_n::DR16_c *fire_dr16 = DR16_n::DR16_c::dr16_->GetClassPtr();
    dt7 = fire_dr16->GetDataStructPtr();

    virtual_rx_data = Get_virtual_recive_ptr();
    memset(&virtual_tx_data,0,sizeof(Visual_Tx_t));
    pid_pluck = new pid_alg_n::cascade_pid_c(pid_pluck_p_cof,pid_pluck_s_cof,10000);
    pid_fire = new pid_alg_n::pid_alg_c(pid_fire_cof);
    gimbal_mode = gimbal_ctrl_c::Get_Gimbal_Instance()->Get_Gimbal_Mode();
    gimbal_pitch_dif = gimbal_ctrl_c::Get_Gimbal_Instance()->Get_Gimbal_Pitch_Dif();
    gimbal_yaw_dif = gimbal_ctrl_c::Get_Gimbal_Instance()->Get_Gimbal_Yaw_Dif();

    motor_right_fire->motor_controller.speed_PID.ECF_PID_ChangeActValSource(&motor_right_fire->MotorMeasure.measure.feedback_speed);
    motor_left_fire->motor_controller.speed_PID.ECF_PID_ChangeActValSource(&motor_left_fire->MotorMeasure.measure.feedback_speed);

    //反馈弹速递推平均滤波
    predict_shoot_speed = new filter_alg_n::recursive_ave_filter_c();  
    //电机使能
    Shoot_ZeroForce_Set();
    motor_right_fire->DJIMotorEnable();
    motor_left_fire->DJIMotorEnable();
    motor_pluck_fire->DJIMotorEnable();
}


/**********************************发射机构无力设定*************************************************/
//摩擦轮无力
void fire_ctrl_c::set_fric_zero()
{
    motor_set_value.left_motor_speed_set = 0;
    motor_set_value.right_motor_speed_set = 0;
}

//拨弹盘无力
void fire_ctrl_c::set_fire_zero()
{
    motor_pluck_fire->MotorMeasure.MeasureClear();
    static_flag.singlefire_state = false;
    motor_set_value.pluck_motor_auto_set = 0;
    motor_set_value.pluck_motor_semi_set = 0;

}

/**
 * @brief          无力设定
 * @param[in]      发射机构结构体
 * @retval         none
 */
void fire_ctrl_c::Shoot_ZeroForce_Set()
{
    set_fric_zero();
    set_fire_zero();
}

/**************************************************************************************************/

/**
 * @brief          将电机转子rpm值转换到实际射速与射频
 * @retval         none
 */
void fire_ctrl_c::Shoot_V_Get()
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
    Real_physical_Val.shoot_rate = convert_shootrat(motor_pluck_fire->MotorMeasure.measure.feedback_speed);

    Real_physical_Val.shoot_hot=temp_shoot_info.shoot_barrel_heat_current;//获取当前热量

    virtual_tx_data.bullet_speed = Real_physical_Val.shoot_speed;
}

/**
 * @brief          根据遥控器控制摩擦轮开/关
 */
void fire_ctrl_c::fric_set()
{
    static uint8_t last_key_f;
     if((dt7->rc.ch[4] > -300 && dt7->rc.ch[4] < -50) && static_flag.fric_state==false)
     {
         #if NOW_STATE==TEST
            motor_set_value.left_motor_speed_set = FIRE_SPEED;
            motor_set_value.right_motor_speed_set = FIRE_SPEED;
         #else
         //TODO:加入裁判系统更新弹速设定
         //根据裁判系统设定弹速//联盟赛射速上限30，可不用设定
            motor_set_value.left_motor_speed_set = FIRE_SPEED;
            motor_set_value.right_motor_speed_set = FIRE_SPEED;
         #endif
            static_flag.isFricReady = true;
            static_flag.fric_state  = 1;
     }
     else if((dt7->rc.ch[4] > -300 && dt7->rc.ch[4] < -50) && static_flag.fric_state==2)
     {
         set_fric_zero();
         static_flag.isFricReady = false;//摩擦轮关闭，指示不能开火
         static_flag.fric_state  = 3;
     }
     else if((dt7->rc.ch[4] < 5 && dt7->rc.ch[4] > -5) && (static_flag.fric_state==1 || static_flag.fric_state==3))//获取归中状态
     {
         static_flag.fric_state=abs(static_flag.fric_state-3);
     }    

     if(((last_key_f==0)&&(dt7->kb.bit.F==1))&&static_flag.fric_state==0)
     {
         #if NOW_STATE==TEST
            motor_set_value.left_motor_speed_set = FIRE_SPEED;
            motor_set_value.right_motor_speed_set = FIRE_SPEED;
         #else
         //TODO:加入裁判系统更新弹速设定
         //根据裁判系统设定弹速//联盟赛射速上限30，可不用设定
            motor_set_value.left_motor_speed_set = FIRE_SPEED;
            motor_set_value.right_motor_speed_set = FIRE_SPEED;
         #endif
         static_flag.isFricReady=true;
         static_flag.fric_state=1;
     }
     else if(((last_key_f==0) && (dt7->kb.bit.F==1)) && static_flag.fric_state==2)
     {
         set_fric_zero();
         static_flag.isFricReady=false;//摩擦轮关闭，指示不能开火
         static_flag.fric_state=3;
     }
     else if(((last_key_f==1)&&(dt7->kb.bit.F==0))&&(static_flag.fric_state==1||static_flag.fric_state==3))//获取归中状态
     {
         static_flag.fric_state=abs(static_flag.fric_state-3);
     }    
     
     last_key_f=dt7->kb.bit.F;

}

static uint32_t semi_last_time=0;
const static uint16_t semi_dead_time=200;
/**
 * @brief          设定发射模式
 * @param[in]      控制发射模式相关结构体
 * @retval         当前发射模式
 */
void fire_ctrl_c::fire_mode_set()
{
    static uint8_t last_key_b;
    static uint8_t last_key_x;
    static uint8_t last_key_v;        //v切换打符正反向
    if(last_key_b==0&&dt7->kb.bit.B==1)  static_flag.isSemi=!static_flag.isSemi;                                            //上升沿修改单发状态
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
    
    last_key_b=dt7->kb.bit.B;//上次按下B状态
    last_key_x=dt7->kb.bit.N;//N就是x;
    last_key_v=dt7->kb.bit.V;
//    if(last_aim_target==1&&virtual_tx_data->now_mode==2) isSemi=1;//初次切入打符：单发
//    else if(last_aim_target==3&&virtual_tx_data->now_mode==1) isSemi=0;//初次切入自瞄：连发

    fire_set_mode_e now_mode = RST;
    //单发触发死区
    if((semi_last_time+semi_dead_time)>xTaskGetTickCount())
    {
        this->static_flag.mode = SEMI;//单发触发后，给定死区防止连发
        return;
    }

    if((dt7->rc.ch[4]<10&&dt7->rc.ch[4]>-5)&&dt7->mouse.press_l==0)//(-5,50)
    {
        now_mode = RST;//遥杆归中指示
    }
    else if((dt7->rc.ch[4]<500&&dt7->rc.ch[4]>100)||(dt7->mouse.press_l==1&&static_flag.isSemi==1))//(50,600)
    {                   //原：(50,600)
        semi_last_time=xTaskGetTickCount();
        now_mode = SEMI;//单发开火状态
    }
    else if((dt7->rc.ch[4]<661&&dt7->rc.ch[4]>600)||(dt7->mouse.press_l==1&&static_flag.isSemi==0))//拨到底(600,660]
    {
        now_mode = AUTO;//连发开火状态
    }

    this->static_flag.mode = now_mode;
}


float fire_range_yaw=0;  //打击容忍区间(暂时不知道干嘛)
/**
 * @brief          根据遥控器值做出相应操作
 * @param[in]      待修改的发射机构结构体
 * @retval         none
 */
void Shoot_Control_Get()
{

    //获取摩擦轮是否开启
    fric_set();
    //遥控器设定当前发射模式
    fire_mode_set();

    static fire_set_mode_e last_fire_mode;//指示上一个模式
    if(dt7->rc.s2==RC_SW_DOWN||dt7->rc.s2==RC_SW_ERROR)//在右上角拨杆拨至下时为无力状态，不允许开火
    {
        Shoot_ZeroForce_Set();
        return;//设定无力状态后退出该函数
    }
    //摩擦轮是否开启
    if(static_flag.isFricReady == false)
    {
        set_fire_zero();
        return;//设定拨弹盘无力后退出该函数
    }
    //TODO:裁判系统设定弹速/射频
    
    if(this->static_flag.mode != last_fire_mode)
    {
        pid_pluck->Cascade_pid_Clear();
        pid_fire->ECF_PID_CLEAR();
    }
    
    
    switch(static_flag.mode)
    {
        case SEMI:
            if((*gimbal_mode)==GIMBAL_MANUAL)
            {
                shoot_single_control(last_fire_mode);
                break;
            }
            
            //当视觉识别到目标时才开火
            if((*gimbal_mode)==GIMBAL_AUTOATTACK && abs(*gimbal_pitch_dif) < 0.5f && abs(*gimbal_yaw_dif) < 0.5f && virtual_rx_data->distance > 0)
            {
                shoot_single_control(last_fire_mode);
            }
            else
            {
                set_fire_zero();
            }
        break;
            
        case AUTO:
        //自瞄下全自动开火需要差值足够小才开火
            if((*gimbal_mode)==GIMBAL_MANUAL)
            {
                shoot_auto_control();
                break;
            }
            if((*gimbal_mode)==GIMBAL_AUTOATTACK && abs(*gimbal_pitch_dif) < 0.5f && abs(*gimbal_yaw_dif) < 0.5f && virtual_rx_data->distance > 0) 
            {
                //现在不需要定头打法
                // if(getShakeFire()==0)//定头
                // {
                //     motor_pluck_fire->DJIMotorSetRef(fire_rate(30));
                //     break;
                // }
                shoot_auto_control();
            }
            else
            {
                set_fire_zero();
            }
        break;
        
        case RST:
        default:
            set_fire_zero();//设定拨弹盘无力
        break;
    }
    
    //热量判断，快超热量时使拨蛋盘无力
    if(temp_shoot_info.shoot_barrel_heat_limit>50&&dt7->kb.bit.SHIFT==0){
        if(temp_shoot_info.shoot_barrel_heat_limit-temp_shoot_info.shoot_barrel_heat_current>temp_shoot_info.shoot_bullet_speed+10) ;
        else set_fire_zero();//设定拨弹盘无力
    }
    last_fire_mode=static_flag.mode;
}

int16_t dead_erro=3096;
/**
 * @brief          单发控制
 * @retval         none
 */
void fire_ctrl_c::shoot_single_control(fire_set_mode_e last_shoot_mode)
{

    static uint8_t singlefire_count=0;
    if(last_shoot_mode==AUTO && static_flag.mode==SEMI)//下降沿触发单发
    {
        set_fire_zero();
        return;
    }
    if(last_shoot_mode==RST && static_flag.mode==SEMI)//下降沿触发单发
    {
        //singlefire_count++;
        singlefire_count=1;
        motor_pluck_fire->MotorMeasure.MeasureClear();  //清空编码盘积累值
    }
    if(singlefire_count>0)
    {
        if(static_flag.singlefire_state==false)
        {//更新设定值
            motor_set_value.pluck_motor_semi_set=AN_BULLET*SEMI_NUM+dead_erro; //丢入速度位置环计算
            static_flag.singlefire_state=true;//指示下一个状态
        }
        //判断单发位置环是否转到位
        if((abs(motor_set_value.pluck_motor_semi_set - motor_pluck_fire->MotorMeasure.measure.record_ecd)<3000)&&(abs(motor_pluck_fire->MotorMeasure.measure.feedback_speed) < 1300))
        {
            set_fire_zero();
            //singlefire_count--;//指示该发子弹已经射出
            singlefire_count=0;
        }
    }
    else
    {
        set_fire_zero();
    }
}
/**
 * @brief          连发控制
 * @retval         none
 */
void fire_ctrl_c::shoot_auto_control()
{

   #if NOW_STATE==TEST
         motor_pluck_fire.spd_set= PLUCK_SPEED;   
   #else
      switch(temp_shoot_info.robot_level) //根据等级设定射频
         {
             case 1:
//                 Set_physical_Val.shoot_rate_set = fire_rate(10);
//             break;
             case 2:
//                 Set_physical_Val.shoot_rate_set = fire_rate(12);
//             break;
             case 3:
                 Set_physical_Val.shoot_rate_set = fire_rate(11);
             break;
             case 4:
//                 Set_physical_Val.shoot_rate_set = fire_rate(17);
//             break;
             case 5:
             case 6:
                 Set_physical_Val.shoot_rate_set = fire_rate(15);
             break;
             case 7:
             case 8:
                 Set_physical_Val.shoot_rate_set = fire_rate(20);
             break;
             case 9:
             case 10:
                 Set_physical_Val.shoot_rate_set = fire_rate(24);
             break;
             default:
                 Set_physical_Val.shoot_rate_set = fire_rate(20);
             break;
         }
         motor_set_value.pluck_motor_auto_set = Set_physical_Val.shoot_rate_set;
         //TODO:加入裁判系统更新射频设定
   #endif
}


/**
 * @brief          电机设定控制值
 * @retval         none
 */
void fire_ctrl_c::Shoot_Set_Val()
{
    //摩擦轮控制值设定
    motor_left_fire->DJIMotorSetRef(motor_set_value.left_motor_speed_set);
    motor_right_fire->DJIMotorSetRef(motor_set_value.right_motor_speed_set);

    //拨弹盘控制值设置
    /**
     *卡弹回退： 检测转矩电流与设定速度，当转矩电流>9000且设定速度>0，给个 周期时间
    *           只要设定的周期时间不等于0，就给个反向速度，设定一次减一次，直到为0
     */ 
    static uint16_t plunk_flag=0;
    static  uint8_t LOCK_FIRE=0;
    if(static_flag.mode==SEMI && plunk_flag==0)
    {
        if(motor_pluck_fire->MotorMeasure.measure.feedback_real_current > 6000)
        {
            LOCK_FIRE++;
        }
        else
        {
            LOCK_FIRE=0;
        }
    }
    else if(static_flag.mode==AUTO&&plunk_flag==0)
    {
        if(motor_pluck_fire->MotorMeasure.measure.feedback_real_current > 9600)
        {
            LOCK_FIRE++;
        }
        else
        {
            LOCK_FIRE=0;
        }
    }
    else
    {
        LOCK_FIRE=0;
    }

    if(LOCK_FIRE>200) plunk_flag=300;

    if(plunk_flag!=0)
    {
        plunk_flag--;
        static_flag.isStuck=true;
        motor_pluck_fire->DJIMotorSetRef(pid_fire->ECF_PID_Calculate(-1000,motor_pluck_fire->MotorMeasure.measure.feedback_speed));
    }
    else
    {
        static_flag.isStuck=false;   
        if(static_flag.mode==SEMI)//位置环
        {
            motor_pluck_fire->DJIMotorSetRef(pid_pluck->Cascade_pid_Calculate(&motor_set_value.pluck_motor_semi_set,&motor_pluck_fire->MotorMeasure.measure.record_ecd,&motor_pluck_fire->MotorMeasure.measure.feedback_speed));
        }
        else//速度环
        {
            motor_pluck_fire->DJIMotorSetRef(pid_fire->ECF_PID_Calculate(motor_set_value.pluck_motor_auto_set,motor_pluck_fire->MotorMeasure.measure.feedback_speed));
        }
    }
    
    //无力
    if(dt7->rc.s2==RC_SW_DOWN||dt7->rc.s2==RC_SW_ERROR)
    {
        motor_pluck_fire->DJIMotorSetRef(0);
    }
}

//发射控制循环
void fire_ctrl_c::Fire_Control_loop()
{
    //获取当前发射机构物理量
    Shoot_V_Get();
    //获取发射机构控制量
    Shoot_Control_Get();
    //发射机构控制量设定
    Shoot_Set_Val(); 
}       

fire_ctrl_c* fire_ctrl_c::Get_Fire_Instance()
{
    return Fire_Instance;
}

fire_ctrl_c* fire_ctrl_c::Fire_Instance = new fire_ctrl_c();
