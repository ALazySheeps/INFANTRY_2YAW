#ifndef __FIRE_CONFIG_H__
#define __FIRE_CONFIG_H__



//当前模式
#define TEST 0
#define GAME 1
#define NOW_STATE GAME

#define POKER_GRID    8           //拨盘格数
#define SEMI_NUM 1                //连发设置
#define AN_BULLET         (36864.0f)    //单个子弹电机位置增加值

/******************************************射频转换************************************************/
//发/s——>rpm
#define FIRERATE_TO_MOTORRPM 266
//rpm——>发/s
#define MOTORRPM_TO_FIRERATE 0.00416666666666666666666666666667

#define fire_rate(x) x*FIRERATE_TO_MOTORRPM      

/**************************************************************************************************/

/******************************************弹速转换************************************************/
//17mm弹丸射击初速度上限为25m/s

#define THEORY 0
#define ACTUAL 1
//选择实践派/理论派
#define FIRE_SPEED_CHANGE ACTUAL

//理论上
#if FIRE_SPEED_CHANGE==THEORY
//摩擦轮半径
#define FRIC_R 0.03
#define RPM_TO_RADS 0.1047
//FRIC_R*RPM_TO_RADS
#define K_FSPD2RPM 318.36994587710920089143584845591
//m/s->rpm
#define FIRESPEED_TO_MOTORRPM 318.36994587710920089143584845591
//rpm->m/s
#define MOTORRPM_TO_FIRESPEED 0.003141

//实际上
#else
//m/s->rpm
#define FIRESPEED_TO_MOTORRPM 264
//rpm->m/s 
#define MOTORRPM_TO_FIRESPEED 0.00329//0.00333333333333333
#endif

#define fire_speed(x) x*FIRESPEED_TO_MOTORRPM

/**************************************************************************************************/

/***************************************设定值控制方式*********************************************/
#define ABSTRACT 0
#define REALUNIT 1
//选择 rpm/真实量
#define SET_UNIT REALUNIT

//真实实际量控制
#if SET_UNIT==REALUNIT
//射频/HZ(0~30)
#define REAL_PLUCK_SPEED 20//20
//弹速/m/s(0~30)
#define REAL_FIRE_SPEED 25//22

//射频/rpm(0~10000)
#define PLUCK_SPEED (REAL_PLUCK_SPEED*FIRERATE_TO_MOTORRPM)
//摩擦轮转速(0~9000)
#define FIRE_SPEED (REAL_FIRE_SPEED*FIRESPEED_TO_MOTORRPM)

#else
//射频/rpm(0~10000)
#define PLUCK_SPEED 2500
//摩擦轮转速(0~9000)
#define FIRE_SPEED 6000

#endif

/**************************************************************************************************/

/****************************************控制参数设定*********************************************/
#define PLUCK_MAX 10000
#define FIRE_MAX 16000

//拨蛋单发位置速度环
#define FIRE_PLUCK_P_KP 0.25//1
#define FIRE_PLUCK_P_KI 0//0
#define FIRE_PLUCK_P_KD 0//0
#define FIRE_PLUCK_P_MAX 4500.f

#define FIRE_PLUCK_S_KP 7
#define FIRE_PLUCK_S_KI 0.5f
#define FIRE_PLUCK_S_KD 0.1f
#define FIRE_PLUCK_S_MAX 7000.0f
#define FIRE_PLUCK_S_IMAX 3000.0f

//拨弹连发速度环
#define FIRE_FIRE_KP 10
#define FIRE_FIRE_KI 0
#define FIRE_FIRE_KD 0

#define FIRE_LEFT_KP 10
#define FIRE_LEFT_KI 0
#define FIRE_LEFT_KD 0

#define FIRE_RIGHT_KP 10
#define FIRE_RIGHT_KI 0
#define FIRE_RIGHT_KD 0

/**************************************************************************************************/


#endif

