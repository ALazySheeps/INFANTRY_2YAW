/*************************** Dongguan-University of Technology -ACE**************************
 * @file    BMI088driver.cpp
 * @author  study-sheep
 * @version V1.0
 * @date    2024/9/29
 * @brief   BMI088使用的主要文件
 ******************************************************************************
 * @verbatim
 *  目前仅支持1个陀螺仪，至于要多个陀螺仪，需要在BMI088Middleware.cpp里面添加拉低拉高不同片选的GPIO，等实际确定
 *  H7用的那几个片选再添加，更新版本。
 *  使用方法：
 *  // 创建对象，然后调用成员函数(创建为全局或静态)
    BMI088Instance_c bmi088_test;
	bmi088_test.BMI088Init(&hspi1, 1);
    // 读取陀螺仪数据
    const INS_t* INS = bmi088_test.Get_INS_Data_Point(void)
 
 * @attention
 *      1、如果使用一块板子多个SPI的情况，SPI_HandleTypeDef *BMI088_SPI的值会频繁变换(SPI1、SPI2)，我不确定他会不会寄掉
 *      2、下版本双陀螺仪的方案1：在BMI088Middleware.cpp文件里面，BMI088_ACCEL_NS_L这类函数里面加上应该if(BMI088_SPI == &hspi1)
 *      else if(BMI088_SPI == &hspi2) 然后用上不同的CS1/CS2_ACCEL_GPIO_Port... 
 * @version           time
 * v1.0   基础版本     2024-9-29   已测试
 * v2.0   支持解算     2024-12-5   已测试
 * @todo
 * CS片选引脚封装为类内成员变量，实现不同陀螺仪区分
 ************************** Dongguan-University of Technology -ACE***************************/
// 本文件头文件
#include "BMI088driver.hpp"
#include "BMI088reg.hpp"
#include "BMI088Middleware.hpp"
// C++库文件
#include <math.h>
// 依赖文件
#include "bsp_dwt.hpp"
// 快速运算库
extern  "C"
{
    #include "user_lib.h"
    #include "arm_math.h"
}

#include "quaternion_EKF.hpp"


const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};
// 类外初始化
BMI088Instance_c* BMI088Instance_c::BMI_List[10] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};

static BSP_DWT_n::BSP_DWT_c* dwt_bmi088 = BSP_DWT_n::BSP_DWT_c::ECF_Get_DwtInstance();

#if defined(BMI088_USE_SPI)

#define BMI088_accel_write_single_reg(reg, data) \
    {                                            \
        BMI088_ACCEL_NS_L();                     \
        BMI088_write_single_reg((reg), (data));  \
        BMI088_ACCEL_NS_H();                     \
    }
#define BMI088_accel_read_single_reg(reg, data) \
    {                                           \
        BMI088_ACCEL_NS_L();                    \
        BMI088_read_write_byte((reg) | 0x80);   \
        BMI088_read_write_byte(0x55);           \
        (data) = BMI088_read_write_byte(0x55);  \
        BMI088_ACCEL_NS_H();                    \
    }
#define BMI088_accel_read_muli_reg(reg, data, len) \
    {                                              \
        BMI088_ACCEL_NS_L();                       \
        BMI088_read_write_byte((reg) | 0x80);      \
        BMI088_read_muli_reg(reg, data, len);      \
        BMI088_ACCEL_NS_H();                       \
    }

#define BMI088_gyro_write_single_reg(reg, data) \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_write_single_reg((reg), (data)); \
        BMI088_GYRO_NS_H();                     \
    }
#define BMI088_gyro_read_single_reg(reg, data)  \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_read_single_reg((reg), &(data)); \
        BMI088_GYRO_NS_H();                     \
    }
#define BMI088_gyro_read_muli_reg(reg, data, len)   \
    {                                               \
        BMI088_GYRO_NS_L();                         \
        BMI088_read_muli_reg((reg), (data), (len)); \
        BMI088_GYRO_NS_H();                         \
    }

// 前向声明
static void BMI088_write_single_reg(uint8_t reg, uint8_t data);
static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data);
static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);

#elif defined(BMI088_USE_IIC)
#endif

static uint8_t BMI088_Accel_Init_Table[BMI088_WRITE_ACCEL_REG_NUM][3] =
    {
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G, BMI088_ACC_RANGE_ERROR},
        {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}

};

static uint8_t BMI088_Gyro_Init_Table[BMI088_WRITE_GYRO_REG_NUM][3] =
    {
        {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
        {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_2000_230_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
        {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}

};

/**
 * @brief 校准零飘，向BMI_List添加本陀螺仪
 */
BMI088Instance_c::BMI088Instance_c()
{
    // uint8_t BMI_List_Idx = 0;
    // while (BMI_List[BMI_List_Idx] != nullptr)   BMI_List_Idx++;//越界风险
    // BMI_List[BMI_List_Idx] = this;
}

const INS_t* BMI088Instance_c::Get_INS_Data_Point(void)
{
    return &INS;
}

/**
 * @brief 所有陀螺仪的数值更新
 */
void BMI088Instance_c::BMI_UpData(void)
{
    uint8_t BMI_List_Idx = 0;
    while (BMI_List[BMI_List_Idx] != nullptr)
    {
        BMI_List[BMI_List_Idx]->BMI088_Read();
        BMI_List[BMI_List_Idx]->Attitude_Calc();
        BMI_List_Idx++;
    }   
}

// 较准零飘
void BMI088Instance_c::Calibrate_MPU_Offset()
{
    // 多个陀螺仪，用到多个SPI时，进行选择
    BMI088_SPI = this->BMI088_WHO;
    static float startTime;
    static uint16_t CaliTimes = 6000; // 需要足够多的数据才能得到有效陀螺仪零偏校准结果
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
    int16_t bmi088_raw_temp;
    float gyroMax[3], gyroMin[3];
    float gNormTemp = 0.0f, gNormMax = 0.0f, gNormMin = 0.0f;

    startTime = dwt_bmi088->ECF_DWT_GetTimeline_s();
    do
    {
        if (dwt_bmi088->ECF_DWT_GetTimeline_s() - startTime > 12)
        {
            // 校准超时
            BMI088.GyroOffset[0] = GxOFFSET;
            BMI088.GyroOffset[1] = GyOFFSET;
            BMI088.GyroOffset[2] = GzOFFSET;
            BMI088.gNorm = gNORM;
            BMI088.TempWhenCali = 40;
            break;
        }
        dwt_bmi088->ECF_DWT_Delay_s(0.005);
        BMI088.gNorm = 0;
        BMI088.GyroOffset[0] = 0;
        BMI088.GyroOffset[1] = 0;
        BMI088.GyroOffset[2] = 0;

        for (uint16_t i = 0; i < CaliTimes; ++i)
        {
            BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);
            bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
            BMI088.Accel[0] = bmi088_raw_temp * this->BMI088_ACCEL_SEN;
            bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
            BMI088.Accel[1] = bmi088_raw_temp * this->BMI088_ACCEL_SEN;
            bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
            BMI088.Accel[2] = bmi088_raw_temp * this->BMI088_ACCEL_SEN;
            gNormTemp = sqrtf(BMI088.Accel[0] * BMI088.Accel[0] +
                              BMI088.Accel[1] * BMI088.Accel[1] +
                              BMI088.Accel[2] * BMI088.Accel[2]);
            BMI088.gNorm += gNormTemp;

            BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
            if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
            {
                bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
                BMI088.Gyro[0] = bmi088_raw_temp * this->BMI088_GYRO_SEN;
                BMI088.GyroOffset[0] += BMI088.Gyro[0];
                bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
                BMI088.Gyro[1] = bmi088_raw_temp * this->BMI088_GYRO_SEN;
                BMI088.GyroOffset[1] += BMI088.Gyro[1];
                bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
                BMI088.Gyro[2] = bmi088_raw_temp * this->BMI088_GYRO_SEN;
                BMI088.GyroOffset[2] += BMI088.Gyro[2];
            }
            // 记录数据极差
            if (i == 0)
            {
                gNormMax = gNormTemp;
                gNormMin = gNormTemp;
                for (uint8_t j = 0; j < 3; ++j)
                {
                    gyroMax[j] = BMI088.Gyro[j];
                    gyroMin[j] = BMI088.Gyro[j];
                }
            }
            else
            {
                if (gNormTemp > gNormMax)
                    gNormMax = gNormTemp;
                if (gNormTemp < gNormMin)
                    gNormMin = gNormTemp;
                for (uint8_t j = 0; j < 3; ++j)
                {
                    if (BMI088.Gyro[j] > gyroMax[j])
                        gyroMax[j] = BMI088.Gyro[j];
                    if (BMI088.Gyro[j] < gyroMin[j])
                        gyroMin[j] = BMI088.Gyro[j];
                }
            }
            // 数据差异过大认为收到外界干扰，需重新校准
            this->gNormDiff = gNormMax - gNormMin;
            for (uint8_t j = 0; j < 3; ++j)
                this->gyroDiff[j] = gyroMax[j] - gyroMin[j];
            if (this->gNormDiff > 0.5f ||
                this->gyroDiff[0] > 0.15f ||
                this->gyroDiff[1] > 0.15f ||
                this->gyroDiff[2] > 0.15f)
            {
                break;
            }
            dwt_bmi088->ECF_DWT_Delay_s(0.0005);
        }
        // 取平均值得到标定结果
        BMI088.gNorm /= (float)CaliTimes;
        for (uint8_t i = 0; i < 3; ++i)
            BMI088.GyroOffset[i] /= (float)CaliTimes;
        // 记录标定时IMU温度
        BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);
        bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
        if (bmi088_raw_temp > 1023)
            bmi088_raw_temp -= 2048;
        BMI088.TempWhenCali = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

        this->caliCount++;
    } while (this->gNormDiff > 0.5f ||
             fabsf(BMI088.gNorm - 9.8f) > 0.5f ||
             this->gyroDiff[0] > 0.15f ||
             this->gyroDiff[1] > 0.15f ||
             this->gyroDiff[2] > 0.15f ||
             fabsf(BMI088.GyroOffset[0]) > 0.01f ||
             fabsf(BMI088.GyroOffset[1]) > 0.01f ||
             fabsf(BMI088.GyroOffset[2]) > 0.01f);
    // 根据标定结果校准加速度计标度因数
    BMI088.AccelScale = 9.81f / BMI088.gNorm;
}

/**
 * @brief 加速计初始化
 * 
 * @return uint8_t 
 */
uint8_t BMI088Instance_c::bmi088_accel_init(void)
{
    // 多个陀螺仪，用到多个SPI时，进行选择
    BMI088_SPI = this->BMI088_WHO;
    // check commiunication
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, this->res);
    dwt_bmi088->ECF_DWT_Delay_s(0.001);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, this->res);
    dwt_bmi088->ECF_DWT_Delay_s(0.001);
    // accel software reset
    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    // HAL_Delay(BMI088_LONG_DELAY_TIME);
    dwt_bmi088->ECF_DWT_Delay_s(0.08);
    // check commiunication is normal after reset
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, this->res);
    dwt_bmi088->ECF_DWT_Delay_s(0.001);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, this->res);
    dwt_bmi088->ECF_DWT_Delay_s(0.001);

    // check the "who am I"
    if (this->res != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    // set accel sonsor config and check
    for (this->write_reg_num = 0; this->write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; this->write_reg_num++)
    {

        BMI088_accel_write_single_reg(BMI088_Accel_Init_Table[this->write_reg_num][0], BMI088_Accel_Init_Table[this->write_reg_num][1]);
        dwt_bmi088->ECF_DWT_Delay_s(0.001);

        BMI088_accel_read_single_reg(BMI088_Accel_Init_Table[this->write_reg_num][0], this->res);
        dwt_bmi088->ECF_DWT_Delay_s(0.001);

        if (this->res != BMI088_Accel_Init_Table[this->write_reg_num][1])
        {
            this->error |= BMI088_Accel_Init_Table[this->write_reg_num][2];
        }
    }
    return BMI088_NO_ERROR;
}

/**
 * @brief 陀螺仪初始化
 * 
 * @return uint8_t 
 */
uint8_t BMI088Instance_c::bmi088_gyro_init(void)
{
    // 多个陀螺仪，用到多个SPI时，进行选择
    BMI088_SPI = this->BMI088_WHO;
    // check commiunication
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, this->res);
    dwt_bmi088->ECF_DWT_Delay_s(0.001);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, this->res);
    dwt_bmi088->ECF_DWT_Delay_s(0.001);

    // reset the gyro sensor
    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    // HAL_Delay(BMI088_LONG_DELAY_TIME);
    dwt_bmi088->ECF_DWT_Delay_s(0.08);
    // check commiunication is normal after reset
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, this->res);
    dwt_bmi088->ECF_DWT_Delay_s(0.001);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, this->res);
    dwt_bmi088->ECF_DWT_Delay_s(0.001);

    // check the "who am I"
    if (this->res != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    // set gyro sonsor config and check
    for (this->write_reg_num = 0; this->write_reg_num < BMI088_WRITE_GYRO_REG_NUM; this->write_reg_num++)
    {

        BMI088_gyro_write_single_reg(BMI088_Gyro_Init_Table[this->write_reg_num][0], BMI088_Gyro_Init_Table[this->write_reg_num][1]);
        dwt_bmi088->ECF_DWT_Delay_s(0.001);

        BMI088_gyro_read_single_reg(BMI088_Gyro_Init_Table[this->write_reg_num][0], this->res);
        dwt_bmi088->ECF_DWT_Delay_s(0.001);

        if (this->res != BMI088_Gyro_Init_Table[this->write_reg_num][1])
        {
            this->write_reg_num--;
            this->error |= BMI088_Accel_Init_Table[this->write_reg_num][2];
        }
    }

    return BMI088_NO_ERROR;
}

/**
 * @brief 初始化BMI088,传入连接的SPI总线handle,以及是否进行在线标定
 * 
 * @param bmi088_SPI handle
 * @param calibrate  1为进行在线标定,0使用离线数据
 * @return uint8_t   成功则返回BMI088_NO_ERROR
 */
float init_quaternion[4] = {0};
uint8_t BMI088Instance_c::BMI088Init(SPI_HandleTypeDef *bmi088_SPI, uint8_t calibrate)
{
    if (add_flag == 0)
    {
        add_flag = 1;
        uint8_t BMI_List_Idx = 0;
        while (BMI_List[BMI_List_Idx] != nullptr)   BMI_List_Idx++;//越界风险
        BMI_List[BMI_List_Idx] = this;
    }

    
    this->BMI088_WHO = bmi088_SPI;
    

    do{
        this->error = BMI088_NO_ERROR;
        this->error |= bmi088_accel_init();
        this->error |= bmi088_gyro_init();
        if (calibrate)
            Calibrate_MPU_Offset();
        else
        {
            this->BMI088.GyroOffset[0] = GxOFFSET;
            this->BMI088.GyroOffset[1] = GyOFFSET;
            this->BMI088.GyroOffset[2] = GzOFFSET;
            this->BMI088.gNorm = gNORM;
            this->BMI088.AccelScale = 9.81f / this->BMI088.gNorm;
            this->BMI088.TempWhenCali = 40;
        }
    }while (this->error != BMI088_NO_ERROR);
    
    InitQuaternion(init_quaternion);
    IMU_QuaternionEKF_Init(&QEKF_INS, init_quaternion, 10, 0.001, 1000000, 1, 0);

    return this->error;
}

/**
 * @brief 读取BMI088的原始六轴数据
 */
void BMI088Instance_c::BMI088_Read(void)
{
    // 多个陀螺仪，用到多个SPI时，进行选择
    BMI088_SPI = this->BMI088_WHO;
    static uint8_t buf[8] = {0};
    static int16_t bmi088_raw_temp;
    //轴加速度计原始数据读取
    BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

    bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
    BMI088.Accel[0] = bmi088_raw_temp * this->BMI088_ACCEL_SEN * BMI088.AccelScale;
    bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    BMI088.Accel[1] = bmi088_raw_temp * this->BMI088_ACCEL_SEN * BMI088.AccelScale;
    bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    BMI088.Accel[2] = bmi088_raw_temp * this->BMI088_ACCEL_SEN * BMI088.AccelScale;
    //角加速度计原始数据读取
    BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
    if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
    {
        if (caliOffset)
        {
            bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
            BMI088.Gyro[0] = bmi088_raw_temp * this->BMI088_GYRO_SEN - BMI088.GyroOffset[0];
            bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
            BMI088.Gyro[1] = bmi088_raw_temp * this->BMI088_GYRO_SEN - BMI088.GyroOffset[1];
            bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
            BMI088.Gyro[2] = bmi088_raw_temp * this->BMI088_GYRO_SEN - BMI088.GyroOffset[2];
        }
        else
        {
            bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
            BMI088.Gyro[0] = bmi088_raw_temp * this->BMI088_GYRO_SEN;
            bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
            BMI088.Gyro[1] = bmi088_raw_temp * this->BMI088_GYRO_SEN;
            bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
            BMI088.Gyro[2] = bmi088_raw_temp * this->BMI088_GYRO_SEN;
        }
    }
    BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);

    bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

    if (bmi088_raw_temp > 1023)
    {
        bmi088_raw_temp -= 2048;
    }
    BMI088.Temperature = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

/**
 * @brief 根据BMI088的原始六轴数据进行状态解算
 */
void BMI088Instance_c::Attitude_Calc(void)
{
    const float gravity[3] = {0, 0, 9.81f};
    
    float dt = BSP_DWT_n::BSP_DWT_c::ECF_Get_DwtInstance()->ECF_DWT_GetDeltaT(&INS_DWT_Count);
    
    INS.Accel[X] = BMI088.Accel[X];
    INS.Accel[Y] = BMI088.Accel[Y];
    INS.Accel[Z] = BMI088.Accel[Z];
    INS.Gyro[X] = BMI088.Gyro[X];
    INS.Gyro[Y] = BMI088.Gyro[Y];
    INS.Gyro[Z] = BMI088.Gyro[Z];

    // demo function,用于修正安装误差,可以不管,本demo暂时没用
    // IMU_Param_Correction();

    // 计算重力加速度矢量和b系的XY两轴的夹角,可用作功能扩展,本demo暂时没用
    // INS.atanxz = -atan2f(INS.Accel[X], INS.Accel[Z]) * 180 / PI;
    // INS.atanyz = atan2f(INS.Accel[Y], INS.Accel[Z]) * 180 / PI;

    // 核心函数,EKF更新四元数
    IMU_QuaternionEKF_Update(&QEKF_INS, INS.Gyro[X], INS.Gyro[Y], INS.Gyro[Z], INS.Accel[X], INS.Accel[Y], INS.Accel[Z], dt);

    memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));

    // 机体系基向量转换到导航坐标系，本例选取惯性系为导航系
    BodyFrameToEarthFrame(xb, INS.xn);
    BodyFrameToEarthFrame(yb, INS.yn);
    BodyFrameToEarthFrame(zb, INS.zn);

    // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
    float gravity_b[3];
    EarthFrameToBodyFrame(gravity, gravity_b);
    for (uint8_t i = 0; i < 3; ++i) // 同样过一个低通滤波
    {
        INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt) + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
    }
    BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n); // 转换回导航系n

    INS.Yaw = QEKF_INS.Yaw;
    INS.Pitch = QEKF_INS.Pitch;
    INS.Roll = QEKF_INS.Roll;
    INS.YawTotalAngle = QEKF_INS.YawTotalAngle;
}

/**
 * @brief reserved.用于修正IMU安装误差与标度因数误差,即陀螺仪轴和云台轴的安装偏移
 * @param param IMU参数
 * @param gyro  角速度
 * @param accel 加速度
 */
void BMI088Instance_c::IMU_Param_Correction(void)
{
    IMU_Param_t *param = &this->IMU_Param;
    float *gyro = this->INS.Gyro;
    float *accel = this->INS.Accel;

    static float lastYawOffset, lastPitchOffset, lastRollOffset;
    static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

    if (fabsf(param->Yaw - lastYawOffset) > 0.001f ||
        fabsf(param->Pitch - lastPitchOffset) > 0.001f ||
        fabsf(param->Roll - lastRollOffset) > 0.001f || param->flag)
    {
        cosYaw = arm_cos_f32(param->Yaw / 57.295779513f);
        cosPitch = arm_cos_f32(param->Pitch / 57.295779513f);
        cosRoll = arm_cos_f32(param->Roll / 57.295779513f);
        sinYaw = arm_sin_f32(param->Yaw / 57.295779513f);
        sinPitch = arm_sin_f32(param->Pitch / 57.295779513f);
        sinRoll = arm_sin_f32(param->Roll / 57.295779513f);

        // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
        c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
        c_12 = cosPitch * sinYaw;
        c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
        c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
        c_22 = cosYaw * cosPitch;
        c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
        c_31 = -cosPitch * sinRoll;
        c_32 = sinPitch;
        c_33 = cosPitch * cosRoll;
        param->flag = 0;
    }
    float gyro_temp[3];
    for (uint8_t i = 0; i < 3; ++i)
        gyro_temp[i] = gyro[i] * param->scale[i];

    gyro[X] = c_11 * gyro_temp[X] +
              c_12 * gyro_temp[Y] +
              c_13 * gyro_temp[Z];
    gyro[Y] = c_21 * gyro_temp[X] +
              c_22 * gyro_temp[Y] +
              c_23 * gyro_temp[Z];
    gyro[Z] = c_31 * gyro_temp[X] +
              c_32 * gyro_temp[Y] +
              c_33 * gyro_temp[Z];

    float accel_temp[3];
    for (uint8_t i = 0; i < 3; ++i)
        accel_temp[i] = accel[i];

    accel[X] = c_11 * accel_temp[X] +
               c_12 * accel_temp[Y] +
               c_13 * accel_temp[Z];
    accel[Y] = c_21 * accel_temp[X] +
               c_22 * accel_temp[Y] +
               c_23 * accel_temp[Z];
    accel[Z] = c_31 * accel_temp[X] +
               c_32 * accel_temp[Y] +
               c_33 * accel_temp[Z];

    lastYawOffset = param->Yaw;
    lastPitchOffset = param->Pitch;
    lastRollOffset = param->Roll;
}

/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param          vector in BodyFrame
 */
void BMI088Instance_c::BodyFrameToEarthFrame(const float *vecBF, float *vecEF)
{
    float *q = this->INS.q;
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void BMI088Instance_c::EarthFrameToBodyFrame(const float *vecEF, float *vecBF)
{
    float *q = this->INS.q;
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

// 使用加速度计的数据初始化Roll和Pitch,而Yaw置0,这样可以避免在初始时候的姿态估计误差
void BMI088Instance_c::InitQuaternion(float *init_q4)
{
    float acc_init[3] = {0};
    float gravity_norm[3] = {0, 0, 1}; // 导航系重力加速度矢量,归一化后为(0,0,1)
    float axis_rot[3] = {0};           // 旋转轴
    // 读取100次加速度计数据,取平均值作为初始值
    for (uint8_t i = 0; i < 100; ++i)
    {
        BMI088_Read();
        acc_init[X] += BMI088.Accel[X];
        acc_init[Y] += BMI088.Accel[Y];
        acc_init[Z] += BMI088.Accel[Z];
        BSP_DWT_n::BSP_DWT_c::ECF_Get_DwtInstance()->ECF_DWT_Delay_s(0.001);
    }

    for (uint8_t i = 0; i < 3; ++i)
        acc_init[i] /= 100;

    Norm3d(acc_init);
    // 计算原始加速度矢量和导航系重力加速度矢量的夹角
    float angle = acosf(Dot3d(acc_init, gravity_norm));
    Cross3d(acc_init, gravity_norm, axis_rot);
    Norm3d(axis_rot);
    init_q4[0] = cosf(angle / 2.0f);
    for (uint8_t i = 0; i < 2; ++i)
        init_q4[i + 1] = axis_rot[i] * sinf(angle / 2.0f); // 轴角公式,第三轴为0(没有z轴分量)
}

#if defined(BMI088_USE_SPI)

static void BMI088_write_single_reg(uint8_t reg, uint8_t data)
{
    BMI088_read_write_byte(reg);
    BMI088_read_write_byte(data);
}

static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data)
{
    BMI088_read_write_byte(reg | 0x80);
    *return_data = BMI088_read_write_byte(0x55);
}

static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    BMI088_read_write_byte(reg | 0x80);

    while (len != 0)
    {
        *buf = BMI088_read_write_byte(0x55);
        buf++;
        len--;
    }
}
#elif defined(BMI088_USE_IIC)

#endif
