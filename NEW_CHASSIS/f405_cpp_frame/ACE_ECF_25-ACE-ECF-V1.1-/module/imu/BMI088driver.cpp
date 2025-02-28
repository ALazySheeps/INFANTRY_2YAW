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
 *  // 创建对象，然后调用成员函数
    BMI088Instance_c bmi088_test;
	while (bmi088_test.BMI088Init(&hspi1, 1) != BMI088_NO_ERROR);
    // 读取陀螺仪数据
    bmi088_test.BMI088_Read(&bmi088_test.BMI088);  
    // debug的时候想查找读取的数据直接定义应该相同的全局结构体变量，这样子就可以看见数据了
    IMU_Data_t value;
    value = bmi088_test.BMI088;  
 * @attention
 *      1、如果使用一块板子多个SPI的情况，SPI_HandleTypeDef *BMI088_SPI的值会频繁变换(SPI1、SPI2)，我不确定他会不会寄掉
 *      2、下版本双陀螺仪的方案1：在BMI088Middleware.cpp文件里面，BMI088_ACCEL_NS_L这类函数里面加上应该if(BMI088_SPI == &hspi1)
 *      else if(BMI088_SPI == &hspi2) 然后用上不同的CS1/CS2_ACCEL_GPIO_Port... 
 * @version           time
 * v1.0   基础版本     2024-9-29   已测试
 ************************** Dongguan-University of Technology -ACE***************************/
// 本文件头文件
#include "BMI088driver.hpp"
#include "BMI088reg.hpp"
#include "BMI088Middleware.hpp"
// C++库文件
#include <math.h>
// 依赖文件
#include "bsp_dwt.hpp"

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

BMI088Instance_c::BMI088Instance_c()
{
    // 空的，暂时不知道写什么
}

// 较准零飘
void BMI088Instance_c::Calibrate_MPU_Offset(IMU_Data_t *bmi088)
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
            bmi088->GyroOffset[0] = GxOFFSET;
            bmi088->GyroOffset[1] = GyOFFSET;
            bmi088->GyroOffset[2] = GzOFFSET;
            bmi088->gNorm = gNORM;
            bmi088->TempWhenCali = 40;
            break;
        }
        dwt_bmi088->ECF_DWT_Delay_s(0.005);
        bmi088->gNorm = 0;
        bmi088->GyroOffset[0] = 0;
        bmi088->GyroOffset[1] = 0;
        bmi088->GyroOffset[2] = 0;

        for (uint16_t i = 0; i < CaliTimes; ++i)
        {
            BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);
            bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
            bmi088->Accel[0] = bmi088_raw_temp * this->BMI088_ACCEL_SEN;
            bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
            bmi088->Accel[1] = bmi088_raw_temp * this->BMI088_ACCEL_SEN;
            bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
            bmi088->Accel[2] = bmi088_raw_temp * this->BMI088_ACCEL_SEN;
            gNormTemp = sqrtf(bmi088->Accel[0] * bmi088->Accel[0] +
                              bmi088->Accel[1] * bmi088->Accel[1] +
                              bmi088->Accel[2] * bmi088->Accel[2]);
            bmi088->gNorm += gNormTemp;

            BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
            if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
            {
                bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
                bmi088->Gyro[0] = bmi088_raw_temp * this->BMI088_GYRO_SEN;
                bmi088->GyroOffset[0] += bmi088->Gyro[0];
                bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
                bmi088->Gyro[1] = bmi088_raw_temp * this->BMI088_GYRO_SEN;
                bmi088->GyroOffset[1] += bmi088->Gyro[1];
                bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
                bmi088->Gyro[2] = bmi088_raw_temp * this->BMI088_GYRO_SEN;
                bmi088->GyroOffset[2] += bmi088->Gyro[2];
            }
            // 记录数据极差
            if (i == 0)
            {
                gNormMax = gNormTemp;
                gNormMin = gNormTemp;
                for (uint8_t j = 0; j < 3; ++j)
                {
                    gyroMax[j] = bmi088->Gyro[j];
                    gyroMin[j] = bmi088->Gyro[j];
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
                    if (bmi088->Gyro[j] > gyroMax[j])
                        gyroMax[j] = bmi088->Gyro[j];
                    if (bmi088->Gyro[j] < gyroMin[j])
                        gyroMin[j] = bmi088->Gyro[j];
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
        bmi088->gNorm /= (float)CaliTimes;
        for (uint8_t i = 0; i < 3; ++i)
            bmi088->GyroOffset[i] /= (float)CaliTimes;
        // 记录标定时IMU温度
        BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);
        bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
        if (bmi088_raw_temp > 1023)
            bmi088_raw_temp -= 2048;
        bmi088->TempWhenCali = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

        this->caliCount++;
    } while (this->gNormDiff > 0.5f ||
             fabsf(bmi088->gNorm - 9.8f) > 0.5f ||
             this->gyroDiff[0] > 0.15f ||
             this->gyroDiff[1] > 0.15f ||
             this->gyroDiff[2] > 0.15f ||
             fabsf(bmi088->GyroOffset[0]) > 0.01f ||
             fabsf(bmi088->GyroOffset[1]) > 0.01f ||
             fabsf(bmi088->GyroOffset[2]) > 0.01f);
    // 根据标定结果校准加速度计标度因数
    bmi088->AccelScale = 9.81f / bmi088->gNorm;
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
uint8_t BMI088Instance_c::BMI088Init(SPI_HandleTypeDef *bmi088_SPI, uint8_t calibrate)
{
    this->BMI088_WHO = bmi088_SPI;
    // BMI088_SPI = bmi088_SPI;
    this->error = BMI088_NO_ERROR;

    this->error |= bmi088_accel_init();
    this->error |= bmi088_gyro_init();
    if (calibrate)
        Calibrate_MPU_Offset(&this->BMI088);
    else
    {
        this->BMI088.GyroOffset[0] = GxOFFSET;
        this->BMI088.GyroOffset[1] = GyOFFSET;
        this->BMI088.GyroOffset[2] = GzOFFSET;
        this->BMI088.gNorm = gNORM;
        this->BMI088.AccelScale = 9.81f / this->BMI088.gNorm;
        this->BMI088.TempWhenCali = 40;
    }

    return this->error;
}

/**
 * @brief 读取一次BMI088的数据,包括gyro和accel
 * 
 * @param bmi088 传入BMI088实例(结构体)
 */
void BMI088Instance_c::BMI088_Read(IMU_Data_t *bmi088)
{
    // 多个陀螺仪，用到多个SPI时，进行选择
    BMI088_SPI = this->BMI088_WHO;
    static uint8_t buf[8] = {0};
    static int16_t bmi088_raw_temp;

    BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

    bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
    bmi088->Accel[0] = bmi088_raw_temp * this->BMI088_ACCEL_SEN * bmi088->AccelScale;
    bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    bmi088->Accel[1] = bmi088_raw_temp * this->BMI088_ACCEL_SEN * bmi088->AccelScale;
    bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    bmi088->Accel[2] = bmi088_raw_temp * this->BMI088_ACCEL_SEN * bmi088->AccelScale;

    BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
    if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
    {
        if (caliOffset)
        {
            bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
            bmi088->Gyro[0] = bmi088_raw_temp * this->BMI088_GYRO_SEN - bmi088->GyroOffset[0];
            bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
            bmi088->Gyro[1] = bmi088_raw_temp * this->BMI088_GYRO_SEN - bmi088->GyroOffset[1];
            bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
            bmi088->Gyro[2] = bmi088_raw_temp * this->BMI088_GYRO_SEN - bmi088->GyroOffset[2];
        }
        else
        {
            bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
            bmi088->Gyro[0] = bmi088_raw_temp * this->BMI088_GYRO_SEN;
            bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
            bmi088->Gyro[1] = bmi088_raw_temp * this->BMI088_GYRO_SEN;
            bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
            bmi088->Gyro[2] = bmi088_raw_temp * this->BMI088_GYRO_SEN;
        }
    }
    BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);

    bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

    if (bmi088_raw_temp > 1023)
    {
        bmi088_raw_temp -= 2048;
    }

    bmi088->Temperature = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
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
