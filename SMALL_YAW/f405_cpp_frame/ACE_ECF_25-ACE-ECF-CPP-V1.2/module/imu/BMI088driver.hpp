#ifndef BMI088DRIVER_H
#define BMI088DRIVER_H

#include "stdint.h"
#include "quaternion_EKF.hpp"
#include "real_main.hpp"

#define BMI088_TEMP_FACTOR 0.125f
#define BMI088_TEMP_OFFSET 23.0f

#define BMI088_WRITE_ACCEL_REG_NUM 6
#define BMI088_WRITE_GYRO_REG_NUM 6

#define BMI088_GYRO_DATA_READY_BIT 0
#define BMI088_ACCEL_DATA_READY_BIT 1
#define BMI088_ACCEL_TEMP_DATA_READY_BIT 2

#define BMI088_LONG_DELAY_TIME 80
#define BMI088_COM_WAIT_SENSOR_TIME 150

#define BMI088_ACCEL_IIC_ADDRESSE (0x18 << 1)
#define BMI088_GYRO_IIC_ADDRESSE (0x68 << 1)

#define BMI088_ACCEL_3G_SEN 0.0008974358974f
#define BMI088_ACCEL_6G_SEN 0.00179443359375f
#define BMI088_ACCEL_12G_SEN 0.0035888671875f
#define BMI088_ACCEL_24G_SEN 0.007177734375f

#define BMI088_GYRO_2000_SEN 0.00106526443603169529841533860381f
#define BMI088_GYRO_1000_SEN 0.00053263221801584764920766930190693f
#define BMI088_GYRO_500_SEN 0.00026631610900792382460383465095346f
#define BMI088_GYRO_250_SEN 0.00013315805450396191230191732547673f
#define BMI088_GYRO_125_SEN 0.000066579027251980956150958662738366f

// 需手动修改
#if INFANTRY_ID == 0
#define GxOFFSET 0.00247530174f
#define GyOFFSET 0.000393082853f
#define GzOFFSET 0.000393082853f
#define gNORM 9.69293118f
#elif INFANTRY_ID == 1
#define GxOFFSET 0.0007222f
#define GyOFFSET -0.001786f
#define GzOFFSET 0.0004346f
#define gNORM 9.876785f
#elif INFANTRY_ID == 2
#define GxOFFSET 0.0007222f
#define GyOFFSET -0.001786f
#define GzOFFSET 0.0004346f
#define gNORM 9.876785f
#elif INFANTRY_ID == 3
#define GxOFFSET 0.00270364084f
#define GyOFFSET -0.000532632112f
#define GzOFFSET 0.00478090625f
#define gNORM 9.73574924f
#elif INFANTRY_ID == 4
#define GxOFFSET 0.0007222f
#define GyOFFSET -0.001786f
#define GzOFFSET 0.0004346f
#define gNORM 9.876785f
#endif

typedef enum
{
    X = 0,
    Y = 1,
    Z = 2,
}INS_idx_e;

/* 用于修正安装误差的参数 */
typedef struct
{
    uint8_t flag;

    float scale[3];

    float Yaw;
    float Pitch;
    float Roll;
} IMU_Param_t;

/* 六轴原始数据结构体 */
typedef struct
{
    float Accel[3];

    float Gyro[3];

    float TempWhenCali;
    float Temperature;

    float AccelScale;
    float GyroOffset[3];

    float gNorm;
} IMU_Data_t;

/* 状态解算数据结构体 */
typedef struct
{
    public:
    float q[4]; // 四元数估计值

    float MotionAccel_b[3]; // 机体坐标加速度
    float MotionAccel_n[3]; // 绝对系加速度

    float AccelLPF; // 加速度低通滤波系数

    // bodyframe在绝对系的向量表示
    float xn[3];
    float yn[3];
    float zn[3];

    // 加速度在机体系和XY两轴的夹角
    // float atanxz;
    // float atanyz;

    // IMU量测值
    float Gyro[3];  // 角速度
    float Accel[3]; // 加速度
    // 位姿
    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;

    uint8_t init;
} INS_t;

/* BMI088错误码枚举 */
enum
{
    BMI088_NO_ERROR = 0x00,
    BMI088_ACC_PWR_CTRL_ERROR = 0x01,
    BMI088_ACC_PWR_CONF_ERROR = 0x02,
    BMI088_ACC_CONF_ERROR = 0x03,
    BMI088_ACC_SELF_TEST_ERROR = 0x04,
    BMI088_ACC_RANGE_ERROR = 0x05,
    BMI088_INT1_IO_CTRL_ERROR = 0x06,
    BMI088_INT_MAP_DATA_ERROR = 0x07,
    BMI088_GYRO_RANGE_ERROR = 0x08,
    BMI088_GYRO_BANDWIDTH_ERROR = 0x09,
    BMI088_GYRO_LPM1_ERROR = 0x0A,
    BMI088_GYRO_CTRL_ERROR = 0x0B,
    BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
    BMI088_GYRO_INT3_INT4_IO_MAP_ERROR = 0x0D,

    BMI088_SELF_TEST_ACCEL_ERROR = 0x80,
    BMI088_SELF_TEST_GYRO_ERROR = 0x40,
    BMI088_NO_SENSOR = 0xFF,
};

extern IMU_Data_t BMI088; // 外部用到姿态解算算法时extern出去

class BMI088Instance_c
{
    public:
        BMI088Instance_c();
        uint8_t BMI088Init(SPI_HandleTypeDef *bmi088_SPI, uint8_t calibrate);
        int16_t caliCount = 0;// 调试用的变量
        uint8_t caliOffset = 1;// 是否开启较准零飘

        float gyroDiff[3], gNormDiff;
        IMU_Data_t BMI088;  //六轴原始数据
        INS_t INS;          //状态解算数据
        SPI_HandleTypeDef *BMI088_WHO;
        IMU_Param_t IMU_Param;
        static void BMI_UpData(void);//更新陀螺仪数据
        const INS_t* Get_INS_Data_Point(void);//返回陀螺仪数据结构体
        QEKF_INS_t QEKF_INS;//EKF滤波结构体
    private:// 私有变量
        
        // 用于获取两次采样之间的时间间隔
        float BMI088_ACCEL_SEN = BMI088_ACCEL_6G_SEN;
        float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;
        uint32_t INS_DWT_Count = 0;
        uint8_t res = 0;
        uint8_t write_reg_num = 0;
        uint8_t error = BMI088_NO_ERROR;
        void BMI088_Read(void);
        void Attitude_Calc(void);
        uint8_t bmi088_accel_init(void);
        uint8_t bmi088_gyro_init(void);
        void Calibrate_MPU_Offset();
        void IMU_Param_Correction();
        void BodyFrameToEarthFrame(const float *vecBF, float *vecEF);
        void EarthFrameToBodyFrame(const float *vecEF, float *vecBF);
        void InitQuaternion(float *init_q4);
        uint8_t add_flag = 0;
    public:
        static BMI088Instance_c* BMI_List[10];
};

#endif
