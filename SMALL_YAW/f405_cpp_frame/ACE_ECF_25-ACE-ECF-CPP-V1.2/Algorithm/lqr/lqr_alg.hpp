#ifndef __LQR_H
#define __LQR_H


#ifdef __cplusplus
extern "C" {
#endif


/* Include */
#ifdef  STM32F405xx
#include "stm32f405xx.h"
#elif STM32H723xx
#include "stm32h723xx.h"
#endif 
#include "cmsis_os.h"
#include <string.h>



#ifdef __cplusplus
}




/* Define */
#ifndef lqr_abs
#define lqr_abs(x) ((x > 0) ? x : -x)
#endif

#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif
#endif


namespace lqr_alg_n{

  typedef struct 
  {
    float k0; 
    float k1; 
  }system_K_t;  //lqr控制参数

  typedef union
  {
    system_K_t struct_k;
    float array_k[2];
  }system_K_u;

  typedef struct 
  {
    uint8_t System_State_Size;     //对应u与input
    uint8_t Control_Size;          //对应Output

    //非线性控制量
    float Control_Variable;
    float Control_Area;      //控制区域
    
    float *Input;
    float *Output;
    float *k;           //最优反馈增益矩阵
    
    float *target;
    
    void (*User_Func_f)(void);

  }lqr_alg_t;

  //lqr算法类
  class lqr_alg_c{
    public:
      lqr_alg_c(uint8_t system_state_size, uint8_t control_size, float *k);
      void ECF_LQR_Init(uint8_t system_state_size, uint8_t control_size, float *k);
      void ECF_LQR_Data_Update(float* system_state);
      float ECF_LQR_Calculate(void);
      void ECF_LQR_Data_Clear(void);

    //private:
      lqr_alg_t lqr_data_;
    
  };


  

}



#endif


#endif


