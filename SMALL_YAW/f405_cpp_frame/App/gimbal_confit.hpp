#ifndef __GIMBAL_CONFIG_H__
#define __GIMBAL_CONFIG_H__

#define LQR 0
#define PID 1
#define PITCH_CONTROL_MODE LQR  //pitch轴控制模式
#define YAW_CONTROL_MODE LQR    //yaw轴控制模式

#define PITCH_VIRTUAL_CONTROL_MODE LQR //pitch轴自瞄控制模式
#define YAW_VIRTUAL_CONTROL_MODE LQR   //yaw轴自瞄控制模式

#define ENCODER 0
#define IMU 1
#define PITCH_LIMIT_WAY ENCODER  //pitch限位方式

/*******************************************PID参数*************************************************/
/***PITCH***/
//内环位置环，外环速度环
#define PITCH_P_KP 0.85//0.75
#define PITCH_P_KI 0.0018//0.0035
#define PITCH_P_KD 0.0001
#define PITCH_P_MAX 3//60
#define PITCH_P_IMAX 320//320

#define PITCH_S_KP 1.15//0.6//1
#define PITCH_S_KI 0
#define PITCH_S_KD 0
#define PITCH_S_MAX 6//6

#define PITCH_MIN -12
#define PITCH_MAX 30

/***YAW***/
#define YAW_P_KP 2//12
#define YAW_P_KI 0
#define YAW_P_KD 0.16
#define YAW_P_MAX 320
#define YAW_P_IMAX 130

#define YAW_S_KP 460//200
#define YAW_S_KI 5
#define YAW_S_KD 0
#define YAW_S_MAX 30000
#define YAW_S_IMAX 2000
#define YAW_S_FILTER 0.1f

/***加上视觉的pid***/
/***PITCH***/
#define PITCH_VIRTUAL_P_KP 0.81//0.75
#define PITCH_VIRTUAL_P_KI 0.0073//0.0032
#define PITCH_VIRTUAL_P_KD 0.0015//0.00035//0

#define PITCH_VIRTUAL_S_KP 1.3//1.15
#define PITCH_VIRTUAL_S_KI 0.000//0
#define PITCH_VIRTUAL_S_KD 0//0


/***YAW***/
#define YAW_VIRTUAL_P_KP 5//8
#define YAW_VIRTUAL_P_KI 0
#define YAW_VIRTUAL_P_KD 0
#define YAW_STEP_K 1.5f

#define YAW_VIRTUAL_S_KP 1100//260
#define YAW_VIRTUAL_S_KI 0
#define YAW_VIRTUAL_S_KD 220

/*******************************************LQR参数*************************************************/
/***PITCH***/
#define PITCH_K_0 -28.0f
#define PITCH_K_1 -0.9f

/***YAW***/
#define YAW_K_0 -10.0f
#define YAW_K_1 -1.0f

/***加上视觉的lqr***/
/***PITCH***/
#define PITCH_VIRTUAL_K_0 -8.0f
#define PITCH_VIRTUAL_K_1 -0.4f

/***YAW***/
#define YAW_VIRTUAL_K_0 -60.0f
#define YAW_VIRTUAL_K_1 -2.0f
#define VIRTUAL_LQR_KI 0
#define VIRTUAL_KF 30000
/***************************************遥感灵敏度*************************************************/
#define PITCH_DPI 0.0004f
#define YAW_DPI 0.0004f

#define PITCH_DPI_MOUSE  0.0008f
#define YAW_DPI_MOUSE 0.002f

#endif
