/**
 * @file filter.cpp
 * @author Lann 梁健蘅 (rendezook@qq.com)
 * @brief 
 * @version 0.1
 * @date 2024-10-10
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "filter.hpp"


/***************************一阶低通滤波********************************************** */
/**
 * @brief 一阶低通滤波初始化
 * @param num 一阶滤波系数
 * @note 滤波系数在 0~1.0 区间, 超出这个范围则默认为 1
 * @note 系数越小, 得到的波形越平滑, 但是也更加迟钝
*/
filter_alg_n::first_order_filter_c::first_order_filter_c(float num)
{
    if(num > 0.0 && num < 1.0)
    {
        this->measure.input = 0;
        this->measure.last_input = 0;
        this->measure.num = num;
        this->measure.out = 0;
    }
    else
    {
        this->measure.input = 0;
        this->measure.last_input = 0;
        this->measure.num = 1;
        this->measure.out = 0;
    }
}
                                                


/**
 * @brief 一阶低通滤波计算
 * @param input 采样值
 * @return 输出值
*/
float filter_alg_n::first_order_filter_c::first_order_filter(float input)        
{
  this->measure.input = input;
  this->measure.out = this->measure.input * this->measure.num + (1 - this->measure.num) * this->measure.last_input;
  this->measure.last_input = this->measure.out;

  return this->measure.out;
}

/**
 * @brief 一阶低通滤波初始化
 * @param num 一阶滤波系数
 * @note 滤波系数在 0~1.0 区间, 超出这个范围则默认为 1
 * @note 系数越小, 得到的波形越平滑, 但是也更加迟钝
*/
void filter_alg_n::first_order_filter_c::first_order_filter_init(float num)
{
    if(num > 0.0 && num < 1.0)
    {
        this->measure.input = 0;
        this->measure.last_input = 0;
        this->measure.num = num;
        this->measure.out = 0;
    }
    else
    {
        this->measure.input = 0;
        this->measure.last_input = 0;
        this->measure.num = 1;
        this->measure.out = 0;
    }
}


/**
 * @brief          一阶低通滤波清除
 * @retval         none
 * @attention      只是清除所有计算的数据，不会清除系数
 */
void filter_alg_n::first_order_filter_c::first_order_filter_clear(void)
{
    this->measure.last_input = 0;
    this->measure.input = 0;
    this->measure.out = 0;
}


/***************************滑动均值滤波**************************************** */

/**
 * @brief 滑动均值滤波初始化
 * 
 */
filter_alg_n::sliding_mean_filter_c::sliding_mean_filter_c()
{
    this->measure.count_num = 0;
    for (int i = 0; i < 20; i++)
        this->measure.FIFO[i] = 0.0f;
    this->measure.Input = 0.0f;
    this->measure.Output = 0.0f;
    this->measure.Sum = 0.0f;
    this->measure.sum_flag = 0;
}


/*
 *功能：滑动均值滤波参数初始化(浮点型)
 *输入：滤波对象结构体
 */
void filter_alg_n::sliding_mean_filter_c::sliding_mean_filter_init()
{
  this->measure.count_num = 0;
  for (int i = 0; i < 20; i++)
    this->measure.FIFO[i] = 0.0f;
  this->measure.Input = 0.0f;
  this->measure.Output = 0.0f;
  this->measure.Sum = 0.0f;
  this->measure.sum_flag = 0;
}

/*
 *功能：滑动均值滤波（浮点型）------抑制小幅度高频噪声
 *传入：1.滤波对象结构体  2.更新值 3.均值数量
 *传出：滑动滤波输出值（250次）
 */
float filter_alg_n::sliding_mean_filter_c::sliding_mean_filter(float Input, int num)
{
  // 更新
  this->measure.Input = Input;
  this->measure.FIFO[this->measure.count_num] = this->measure.Input;
  this->measure.count_num++;

  if (this->measure.count_num == num)
  {
    this->measure.count_num = 0;
    this->measure.sum_flag = 1;
  }
  // 求和
  if (this->measure.sum_flag == 1)
  {
    for (int count = 0; count < num; count++)
    {
      this->measure.Sum += this->measure.FIFO[count];
    }
  }
  // 均值
  this->measure.Output = this->measure.Sum / num;
  this->measure.Sum = 0;

  return this->measure.Output;
}


/**********************递推平均滤波*************************************************** */
filter_alg_n::recursive_ave_filter_c::recursive_ave_filter_c()
{
    for (int i = 0; i < 300; ++i) {
        measure.count_num = 0;
    }
    measure.sum = 0;
    measure.filter_out = 0;
}

float filter_alg_n::recursive_ave_filter_c::Recursive_ave_filter(float input, int num)
{
    int i;
    for(i = num-1; i > 0;i--)//搬运
    {
        this->measure.fifo[i] = this->measure.fifo[i-1];
    }
    this->measure.fifo[0] = input;

    if(this->measure.count_num < num)//终止条件为count_num = num;
    {
        this->measure.count_num ++;
    }else
    {
        this->measure.count_num = num;
    }

    this->measure.sum = 0;
    for(i = 0; i < this->measure.count_num; i++)//防止取样数量 < max时 滤波错误
    {
        this->measure.sum += this->measure.fifo[i];
    }
    this->measure.filter_out = this->measure.sum/this->measure.count_num;
    return this->measure.filter_out;
}
