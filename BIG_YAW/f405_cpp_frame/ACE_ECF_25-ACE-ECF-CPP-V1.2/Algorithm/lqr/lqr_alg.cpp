/**
 * @file lqr_alg.cpp
 * @brief lqr算法层
 *
 * @version 1.0
 * @date 2024-9
 * @example
 * 	//设定参数矩阵
    float k_pitch_lqr[2] = {PITCH_K_0, PITCH_K_1};

	//lqr初始化
	lqr_alg_c lqr(2,1,k_pitch_lqr);

	//设定目标需要旋转的角度以及当前轴陀螺仪的值，进行数据更新
	float Pitch_system_state[2] = {((-Pitch_motortarget) / 57.295779513f), gimbal_ctrl->imu->Gyro[0]};
	lqr.ECF_LQR_Data_Update(Pitch_system_state);
	
	//计算得到输出值
	float output = lqr.ECF_LQR_Calculate();
 *
 */

#include "lqr_alg.hpp"
using namespace lqr_alg_n;


/**
 * ************************* Dongguan-University of Technology -ACE**************************
 * @brief LQR计算
 * @param  void 
 * @return output             
 * ************************* Dongguan-University of Technology -ACE**************************
 */
float lqr_alg_n::lqr_alg_c::ECF_LQR_Calculate(void)
{
	int i,j;
	for(i = 0; i < this->lqr_data_.Control_Size; i++)
	{
		this->lqr_data_.Output[i] = 0;
		for(j = 0; j < this->lqr_data_.System_State_Size; j++)
		{
			this->lqr_data_.Output[i] += this->lqr_data_.Input[j] * this->lqr_data_.k[i * this->lqr_data_.System_State_Size + j];
		}
	}
	return this->lqr_data_.Output[0];
}


/**
 * ************************* Dongguan-University of Technology -ACE**************************
 * @brief  LQR构造初始化
 * @param  system_state_size
 * @param  control_size     
 * @param  k     
 * ************************* Dongguan-University of Technology -ACE**************************
 */
lqr_alg_c::lqr_alg_c(uint8_t system_state_size, uint8_t control_size, float *k)
{
  this->lqr_data_.System_State_Size = system_state_size;
  this->lqr_data_.Control_Size = control_size;
	
  //lqr->Control_Area = control_area;
  if(system_state_size != 0)
  {
    this->lqr_data_.Input = (float *)user_malloc(sizeof(float) * system_state_size * control_size);
	memset(this->lqr_data_.Input, 0, sizeof(float) * system_state_size * control_size);
	this->lqr_data_.k = (float *)user_malloc(sizeof(float) * system_state_size * control_size);
	memset(this->lqr_data_.k, 0, sizeof(float) * system_state_size * control_size);

  }
  if(control_size != 0)
  {
    this->lqr_data_.Output = (float *)user_malloc(sizeof(float) * control_size);
	memset(this->lqr_data_.Output, 0, sizeof(float) * control_size);

  }
  
	  this->lqr_data_.k = k;
  
}

/**
 * ************************* Dongguan-University of Technology -ACE**************************
 * @brief  LQR手动初始化
 * @param  system_state_size  
 * @param  control_size     
 * @param  k    lqr参数
 * ************************* Dongguan-University of Technology -ACE**************************
 */
void lqr_alg_c::ECF_LQR_Init(uint8_t system_state_size, uint8_t control_size, float *k)
{
  this->lqr_data_.System_State_Size = system_state_size;
  this->lqr_data_.Control_Size = control_size;
	
  //lqr->Control_Area = control_area;
  if(system_state_size != 0)
  {
    this->lqr_data_.Input = (float *)user_malloc(sizeof(float) * system_state_size * control_size);
	memset(this->lqr_data_.Input, 0, sizeof(float) * system_state_size * control_size);
	this->lqr_data_.k = (float *)user_malloc(sizeof(float) * system_state_size * control_size);
	memset(this->lqr_data_.k, 0, sizeof(float) * system_state_size * control_size);

  }
  if(control_size != 0)
  {
    this->lqr_data_.Output = (float *)user_malloc(sizeof(float) * control_size);
	memset(this->lqr_data_.Output, 0, sizeof(float) * control_size);

  }
  
	  this->lqr_data_.k = k;
}



/**
 * ************************* Dongguan-University of Technology -ACE**************************
 * @brief  LQR数据清除
 * @param  void 
 * ************************* Dongguan-University of Technology -ACE**************************
 */
void lqr_alg_c::ECF_LQR_Data_Clear(void)
{
  memset(this->lqr_data_.Input, 0, sizeof(float) * this->lqr_data_.System_State_Size * this->lqr_data_.Control_Size);
  
  memset(this->lqr_data_.Output, 0, sizeof(float) * this->lqr_data_.Control_Size);
}


/**
 * ************************* Dongguan-University of Technology -ACE**************************
 * @brief  LQR数据更新
 * @param  system_state  系统状态矩阵            
 * ************************* Dongguan-University of Technology -ACE**************************
 */
void lqr_alg_c::ECF_LQR_Data_Update(float* system_state)
{
	int i = 0;

	for(; i < this->lqr_data_.System_State_Size; i++)
	{
		this->lqr_data_.Input[i] = system_state[i];
	}
}


