#include "bigyaw_ctrl.hpp"

/**
 * @brief  拨弹盘构造函数
 * @param  id: 电调ID
 * @param  can_handle: CAN句柄
 */
pluck_ctrl_c::pluck_ctrl_c(uint8_t id,CAN_HandleTypeDef* can_handle):id_(id),can_handle_(can_handle),
                            pluck_motor(nullptr),pluck_semi_pid(nullptr),pluck_auto_pid(nullptr)
{
    fire_set_mode_ = RST;
    fire_mode_ = NO_FIRE;
}

void pluck_ctrl_c::Pluck_init()
{
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
          .can_handle = can_handle_,
          .tx_id = id_,// 看电调闪几下就填几
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

    pluck_motor = new DJI_Motor_n::DJI_Motor_Instance(fire_motor_confit);
    pluck_semi_pid = new pid_alg_n::cascade_pid_c(pid_pluck_p_cof,pid_pluck_s_cof,10000);
    pluck_auto_pid = new pid_alg_n::pid_alg_c(pid_fire_cof);

    pluck_motor->DJIMotorEnable();
}

/**
 * @brief  发射机构锁定
 */
void pluck_ctrl_c::Pluck_lock()
{
    pluck_motor_auto_set = 0;
    pluck_motor_semi_set = 0;
}

/**
 * @brief  设置发射机构无力
 */
void pluck_ctrl_c::Pluck_zero()
{
    fire_mode_ = ZERO_FIRE;
}


const static uint16_t semi_dead_time=200;
/**
 * @brief  拨弹盘设置控制数据
 * @param  dr16_data: 摇控器数据
 */
void pluck_ctrl_c::Pluck_set_ctrl_data(const DR16_n::RC_ctrl_t &dr16_data)
{
    static uint32_t semi_last_time=0;
    static uint8_t last_key_b = 0;

    if(last_key_b==0&&dr16_data.kb.bit.B==1)  isSemi=!isSemi;   //上升沿修改发射状态
    last_key_b = dr16_data.kb.bit.B;

    fire_set_mode_e now_mode = RST;
    //单发触发死区
    if((semi_last_time+semi_dead_time)>xTaskGetTickCount())
    {
        fire_set_mode_ = SEMI;//单发触发后，给定死区防止连发
        return;
    }

    if((dr16_data.rc.ch[4] < 20 && dr16_data.rc.ch[4] > -5)&& dr16_data.mouse.press_l == 0)//(-5,50)
    {
        now_mode = RST;//遥杆归中指示
    }
    else if(((dr16_data.rc.ch[4] < 600&&dr16_data.rc.ch[4] > 50))||(dr16_data.mouse.press_l==1&&isSemi==1))//(50,600)
    {                   //原：(50,600)
        semi_last_time=xTaskGetTickCount();
        now_mode = SEMI;//单发开火状态
    }
    else if(((dr16_data.rc.ch[4] < 661&&dr16_data.rc.ch[4] > 600))||(dr16_data.mouse.press_l==1&&isSemi==0))//拨到底(600,660]
    {
        now_mode = AUTO;//连发开火状态
    }

    fire_set_mode_ = now_mode;
}

/**
 * @brief  设置发射模式
 */
void pluck_ctrl_c::Pluck_set_mode()
{
    static fire_set_mode_e last_fire_behavier = RST;

    switch(fire_set_mode_)
    {
        case RST:
            fire_mode_ = NO_FIRE;
            break;
        case SEMI:
            if(last_fire_behavier==RST)
            {
                fire_mode_ = SEMI_FIRE;
                singlefire_state=false;
                pluck_motor->MotorMeasure.MeasureClear();  //首次进入单发，清空编码值积累
            }
            else if(last_fire_behavier==SEMI)
            {
                if(singlefire_state==false)  //子弹未发射
                {
                    fire_mode_ = SEMI_FIRE;
                }
                else                         //子弹已经发射
                {
                    fire_mode_ = NO_FIRE;
                }
            }
            else
            {
                fire_mode_ = NO_FIRE;
            }
            break;
        case AUTO:
            fire_mode_ = AUTO_FIRE;
        default : break;
    }

    // 未开启摩擦轮或未瞄准到目标时不发射
    if(allow_fire_ == false)
    {
        fire_mode_ = NO_FIRE;
    }

    last_fire_behavier = fire_set_mode_;
}

/**
 * @brief  拨弹盘单发控制

 */
int16_t dead_erro=3096;
void pluck_ctrl_c::Pluck_semi_ctrl()
{
    pluck_motor_semi_set = AN_BULLET*SEMI_NUM + dead_erro;
    if((pluck_motor_semi_set - pluck_motor->MotorMeasure.measure.feedback_ecd) < 3000 && pluck_motor->MotorMeasure.measure.feedback_speed < 1300)
    {
        singlefire_state = true; //子弹已经发射
    }
}

/**
 * @brief  拨弹盘连发频率设置
 * @param  robot_level: 机器人等级
 */
void pluck_ctrl_c::Pluck_auto_ctrl(uint8_t robot_level)
{
    switch(robot_level)
    {
        case 1:
        case 2:
        case 3:
            fire_rate_ = 11;
            break;
        case 4:
        case 5:
        case 6:
            fire_rate_ = 15;
            break;
        case 7:
        case 8:
            fire_rate_ = 20;
            break;
        case 9:
        case 10:
            fire_rate_ = 24;
            break;
        default:
            fire_rate_ = 20;
            break;
    }
    pluck_motor_auto_set = fire_rate(fire_rate_);  //射频转rpm
}

/**
 * @brief  拨弹盘设置控制数据
 */
void pluck_ctrl_c::Pluck_set_target_data()
{
    static fire_mode_t last_fire_mode = fire_mode_t::ZERO_FIRE;
    switch(fire_mode_)
    {
        case NO_FIRE:
            Pluck_lock();
            break;
        case SEMI_FIRE:
            Pluck_semi_ctrl();
            break;
        case AUTO_FIRE:
            Pluck_auto_ctrl(robot_level_);
            break;
        case ZERO_FIRE:
        default: 
            pluck_motor_auto_set = 0.0f;
        break;
    }

    //模式切换清空累计值
    if(last_fire_mode != fire_mode_)
    {
        pluck_motor->MotorMeasure.MeasureClear();
        pluck_semi_pid->Cascade_pid_Clear();
        pluck_auto_pid->ECF_PID_CLEAR();
    }

    last_fire_mode = fire_mode_;
}

/**
 * @brief  拨弹盘控制数据计算
 */
void pluck_ctrl_c::Pluck_ctrl_cauculate()
{
    //拨弹盘控制值设置
    /**
     *卡弹回退： 检测转矩电流与设定速度，当转矩电流>9000且设定速度>0，给个 周期时间
    *           只要设定的周期时间不等于0，就给个反向速度，设定一次减一次，直到为0
     */ 
    static uint16_t plunk_flag=0;
    static  uint8_t LOCK_FIRE=0;

    if(fire_mode_==SEMI_FIRE && plunk_flag==0)
    {
        if(pluck_motor->MotorMeasure.measure.feedback_real_current > 6000)  //单发卡弹判定电流
        {
            LOCK_FIRE++;
        }
        else
        {
            LOCK_FIRE=0;
        }
    }
    else if(fire_mode_==AUTO_FIRE&&plunk_flag==0)
    {
        if(pluck_motor->MotorMeasure.measure.feedback_real_current > 9600)  //连发卡弹判定电流
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

    if(plunk_flag!=0)  //卡弹回退
    {
        plunk_flag--;
        isStuck=true;
        output = pluck_auto_pid->ECF_PID_Calculate(-1000,pluck_motor->MotorMeasure.measure.feedback_speed);
    }
    else
    {
        isStuck=false;   
        if(fire_mode_==SEMI_FIRE)//位置环
        {
            output = pluck_semi_pid->Cascade_pid_Calculate(&pluck_motor_semi_set,&pluck_motor->MotorMeasure.measure.record_ecd,&pluck_motor->MotorMeasure.measure.feedback_speed);
        }
        else//速度环
        {
            output = pluck_auto_pid->ECF_PID_Calculate(pluck_motor_auto_set,pluck_motor->MotorMeasure.measure.feedback_speed);
        }
    }


    if(fire_mode_ == ZERO_FIRE)
    {
        output = 0.0f;
    }

}

/**
 * @brief  热量判断，快超热量时使拨蛋盘无力
 */
void pluck_ctrl_c::Pluck_hurt_ctrl(const DR16_n::RC_ctrl_t &dr16_data,const shoot_msg_t& temp_shoot_info)
{
    if(temp_shoot_info.shoot_barrel_heat_limit>50&&dr16_data.kb.bit.SHIFT==0){
        if(temp_shoot_info.shoot_barrel_heat_limit-temp_shoot_info.shoot_barrel_heat_current>temp_shoot_info.shoot_bullet_speed+10) ;
        else fire_mode_ = NO_FIRE;
    }
}

/**
 * @brief  拨弹盘控制循环
 */
void pluck_ctrl_c::Pluck_ctrl_loop(const DR16_n::RC_ctrl_t &dr16_data,const shoot_msg_t& temp_shoot_info)
{
    //设置发射模式
    if(dr16_data.rc.s2 == RC_SW_DOWN || dr16_data.rc.s2 == RC_SW_ERROR)
    {
        Pluck_zero();
    }
    else
    {
        Pluck_set_ctrl_data(dr16_data);
        Pluck_set_mode();
    }
    Pluck_hurt_ctrl(dr16_data,temp_shoot_info);

    robot_level_ = temp_shoot_info.robot_level;
    Pluck_set_target_data();
    Pluck_ctrl_cauculate();
    pluck_motor->DJIMotorSetRef(output);
}

/**
 * @brief  允许开火
 */
void pluck_ctrl_c::Pluck_allow_fire()
{
    allow_fire_ = true;
}

/**
 * @brief  停止开火
 */
void pluck_ctrl_c::Pluck_stop_fire()
{
    allow_fire_ = false;
}
