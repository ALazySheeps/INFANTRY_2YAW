#ifndef __CHASSIS_CONFIG_H__
#define __CHASSIS_CONFIG_H__
//双头龙为1，四头龙给0
#define DOUBLEHEAD 1
//跟灯条给1，跟侧面给0
#define FOLLOW_LIGHT 1

//遥控器是否安装在底盘上，1为安装，0为不安装
#define DR16_INSTALL_CHASSIS 0
//是否使用导航模式
#define NAVIGATION_MODE 1

#ifndef PI
#define PI 3.1415926f
#endif

//yaw电机（M6020）初始偏移角度
#define YAW_OFFSET 114.0f


#define CHASSIS_MOTOR_REDUCATION_RATIO 19.0f  //底盘电机（M3508）减速比

#define MOTOR_3508_CURRENT_LIMIT 12000 //3508最大电流值
#define MOTOR_3508_MAX_GIVEN_CURRENT 3000 //给定最大电流值

#define MAX_MOTOR_SPEED 9000

//底盘电机id
#define LF_ID 1
#define RF_ID 2
#define LB_ID 3
#define RB_ID 4

#define WHEEL_ID_FIND_ANGLE(x) ((x)*90.0f+45.0f)  //通过掉线轮子ID找到其对应角度

/***************************************底盘速度量化*****************************************/
#define CHASSIS_MAX_SPEED 4.0f  //底盘最大平移速度
#define CHASSIS_ROTATION_SPEED 80.0f  //小陀螺的旋转速度rpm
#define CHASSIS_NAVIGATION_ROTATION_SPEED 40.0f  //导航模式小陀螺的旋转速度rpm

#define REMOTE_TO_SPEED (CHASSIS_MAX_SPEED/(660.0f))  //遥控器转移速映射
#define CHASSIS_NORMAL_SPEED 2.0f  //键鼠操作下默认速度m/s

#define CHASSIS_WIDTH  0.2f   //轮子到中心的距离
#define CHASSIS_WHEEL_WIDTH 0.08f //轮子半径

#define M3508_RATIO 19 //3508减速比

#define RPM_TO_RAD(x) (2*PI*((x)/60.0f))  //rpm转角速度

/***************************************底盘速度分解控制量*****************************************/
//x、y方向速度滤波
#define CHASSIS_FILTER_K 0.1f//0.9f

//底盘速度pid(X,Y方向)
#define CHASSIS_SPEED_KP 1.0f
#define CHASSIS_SPEED_KI 0.0f
#define CHASSIS_SPEED_KD 0//10.8f//accel
#define CHASSIS_SPEED_MAX 5     //底盘最大速度值
#define CHASSIS_SPEED_XY_MAX 4     //底盘平移最大速度值
#define CHASSIS_SPEED_Z_MAX 50  //底盘旋转最大角速度

//底盘速度pid步进系数
#define CHASSIS_STEPIN_K 0.03f
//低通滤波比例 底盘速度pid
#define CHASSIS_FIRST_ORDER_FILTER_K 0.02f//0.02f // 越小越平稳，灵敏度越低；越高输出不稳，但灵敏度更高//0.041f

//底盘旋转pid
#define CHASSIS_SPEED_Z_KP 1.0f  
#define CHASSIS_SPEED_Z_KI 0     // 0.01f
#define CHASSIS_SPEED_Z_KD 0.5f  // 0.5f
#define CHASSIS_Z_STEPIN_K 0.08f


#define PID 0
#define LQR 1
#define FOLLOW_CONTRAL_MODE LQR
#define CHASSIS_FOLLOW_LQR_K1 0.4f
#define CHASSIS_FOLLOW_LQR_K2 0.1f

//底盘跟随位置环pid
#define CHASSIS_FOLLOW_P_KP 3  //10
#define CHASSIS_FOLLOW_P_KI 0.0
#define CHASSIS_FOLLOW_P_KD 0.0f
#define CHASSIS_FOLLOW_IMAX 1000
//前馈系数
#define CHASSIS_FOLLOW_P_KF 0//800000

#define CHASSIS_FOLLOW_P_DEADZONE 3.0f//3°以内为死区
#define CHASSIS_FOLLOW_P_MAX 230

//底盘跟随速度环pid

#define CHASSIS_FOLLOW_S_KP 20.0f //30
#define CHASSIS_FOLLOW_S_KI 0.000f//0.005
#define CHASSIS_FOLLOW_S_KD 500.0f //10000.0f

#define CHASSIS_FOLLOW_S_FILTER 0.032f//0.032f //底盘跟随速度环输出低通滤波比例
#define CHASSIS_FOLLOW_S_KF 200      //底盘跟随速度环前馈系数

/**************************************************************************************************/
/************底盘pid************/
//电机速度环pid
#define CHASSIS_MOTOR_KP 2.45f
#define CHASSIS_MOTOR_KI 0.0f
#define CHASSIS_MOTOR_KD 0.24f
#define CHASSIS_MOTOR_OUT_FILITER 0.15f//0.15f
//电机位置环pid
#define CHASSIS_MOTOR_LOCK_KP 0.65f//1.0f  
#define CHASSIS_MOTOR_LOCK_KI 0.0f
#define CHASSIS_MOTOR_LOCK_KD 50//30.0f

#endif

