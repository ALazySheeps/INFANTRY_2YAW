#include "Motor_General_def.hpp"

void Motor_General_Def_n::Motor_Controller_c::RefValChange(float ref_val)
{
    this->pid_ref = ref_val;
}

float Motor_General_Def_n::Motor_Controller_c::GetRefVal(void)
{
    return this->pid_ref;
}


void Motor_General_Def_n::Motor_Controller_c::All_PID_Clear(void)
{
    this->angle_PID.ECF_PID_CLEAR();
    this->speed_PID.ECF_PID_CLEAR();
    this->current_PID.ECF_PID_CLEAR();
}
