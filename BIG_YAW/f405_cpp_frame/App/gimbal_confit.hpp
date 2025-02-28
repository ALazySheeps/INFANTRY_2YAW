#ifndef __GIMBAL_CONFIG_H__
#define __GIMBAL_CONFIG_H__


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
