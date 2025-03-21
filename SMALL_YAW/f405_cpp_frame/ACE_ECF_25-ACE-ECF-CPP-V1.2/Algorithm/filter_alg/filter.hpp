#ifndef __FILTER_H
#define __FILTER_H


#ifdef __cplusplus
extern "C"{
#endif

#include <stdint.h>
#include "arm_math.h"
#include "main.h"
//#include "struct_typedef.h"

#ifdef __cplusplus
}

#endif

/**
 * @brief 滤波算法命名空间
*/
namespace filter_alg_n
{
    /****************************一阶低通滤波******************************************** */

    //一阶低通滤波参数
    typedef  struct
    {
        float input;		 //输入数据
        float last_input; //上次数据
        float out;		 //滤波输出的数据
        float num;		 //滤波参数
    } first_order_filter_t;
    
    // 一阶低通滤波类
    class first_order_filter_c
    {
        public:
        float first_order_filter(float input);
        void first_order_filter_init(float num);
        void first_order_filter_clear(void);
        first_order_filter_c(float num);
        private:
        first_order_filter_t measure;
    };


    /**********************滑动均值滤波*************************************************** */
    
    //滑动均值滤波参数（浮点）
    typedef struct
    {
        float Input;        //当前取样值
        int32_t count_num; //取样次数
        float Output;       //滤波输出
        float Sum;          //累计总和
        float FIFO[250];    //队列
        int32_t sum_flag;  //已经够250个标志
    } sliding_mean_filter_t;

    //滑动均值滤波类
    class sliding_mean_filter_c
    {
        public:
        sliding_mean_filter_c();
        void sliding_mean_filter_init();                      //均值滑窗滤波初始化（可不用，直接定义结构体时给初值）
        float sliding_mean_filter(float Input, int num); //均值滑窗滤波
        
        private:
        sliding_mean_filter_t measure;
    };

    /**********************递推平均滤波*************************************************** */
    //递推平均滤波参数
    typedef __attribute__((packed)) struct
    {
        int32_t count_num;
        float fifo[300];
        float sum;
        float filter_out;
    } Recursive_ave_filter_type_t;

    class recursive_ave_filter_c
    {
    public:
        recursive_ave_filter_c();
        float Recursive_ave_filter(float input, int num);
        Recursive_ave_filter_type_t measure;
    };

}

#endif // !__FILTER_H
