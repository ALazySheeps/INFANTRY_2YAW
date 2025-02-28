/*************************** Dongguan-University of Technology -ACE**************************
 * @file    RC.cpp
 * @author  zhengNannnn
 * @version V1.0
 * @date    2024/9/13
 * @brief
 ******************************************************************************
 * @verbatim
 *  RC 融合 dr16 裁判系统数据
 * @attention
 *  依赖 safe_task 文件
 * @version           time
 * v1.0   基础版本
 * v2.0   C++升级版本  2024-8-27
 ************************** Dongguan-University of Technology -ACE***************************/
#include "RC.hpp"
#include "safe_task.hpp"

#ifdef  __cplusplus
extern "C"
{
#endif
    
#include "CRC.h"
// #include "bsp_can.h"
#include "RC.h"
#include "usart.h"
#include <string.h>
    
#ifdef  __cplusplus
}
#endif

#define ABS(NUM) (((NUM) > 0 ? (NUM) : (-NUM)))

/**
 * @brief DMA双缓冲接收初始化
 */
void DMA_Init(UART_HandleTypeDef *huart, uint8_t *Rx1_Buff, uint8_t *Rx2_Buff,
              uint16_t Data_Buff_Lenth)
{
    // 使能DMA串口接收
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
    // 使能空闲中断
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    //    //设置DMA传输，将串口1的数据搬运到recvive_buff中
    //    HAL_UART_Receive_DMA(&huart1, sbus_rx_buf[0], 36 );
    // 失效DMA
    __HAL_DMA_DISABLE(huart->hdmarx);
    while (huart->hdmarx->Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(huart->hdmarx);
    }
    huart->hdmarx->Instance->PAR = (uint32_t) & (huart->Instance->DR);
    // 内存缓冲区1
    huart->hdmarx->Instance->M0AR = (uint32_t)(Rx1_Buff);
    // 内存缓冲区2
    huart->hdmarx->Instance->M1AR = (uint32_t)(Rx2_Buff);
    // 数据长度
    huart->hdmarx->Instance->NDTR = Data_Buff_Lenth;
    // 使能双缓冲区
    SET_BIT(huart->hdmarx->Instance->CR, DMA_SxCR_DBM);
    // 使能DMA
    __HAL_DMA_ENABLE(huart->hdmarx);
}

ECF_RC::ECF_RC()
{
#ifdef RECIVE_DT7_CONTROL
    this->Dt7_huart = &DT7_USART;
#endif

#ifdef RECIVE_REFFEREE
    this->Referee_huart = &REFFEREE_USART;
#endif

#ifdef RECIVE_POHOTO_CONTROL
    this->Photo_huart = &POHOTO_USART;
#endif
    // 使能接收中断,实测该步先于main函数的MX_USART3_UART_Init()函数,因此开放ECF_RC_Init()供再次初始化
    // RC_Init(this->huart, this->Sbus_RX_Buffer[0], this->Sbus_RX_Buffer[1], Sbus_RX_Buffer_Num);
}

/**
 * @brief DT7数据解析
 * @note  一次转发
 */
void ECF_RC::DT7_DataProcess(uint8_t idx)
{
    uint8_t *pData = this->Sbus_RX_Buffer[idx];
    this->Dt7.rc.ch[0] = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;        //!< Channel 0
    this->Dt7.rc.ch[1] = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF; //!< Channel 1
    this->Dt7.rc.ch[2] = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |          //!< Channel 2
                          ((int16_t)pData[4] << 10)) &
                         0x07FF;

    this->Dt7.rc.ch[3] = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5] << 7)) & 0x07FF; //!< Channel 3

    this->Dt7.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2; //!< Switch left
    this->Dt7.rc.s2 = ((pData[5] >> 4) & 0x0003);      //!< Switch right

    this->Dt7.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);   //!< Mouse X axis
    this->Dt7.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);   //!< Mouse Y axis
    this->Dt7.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8); //!< Mouse Z axis

    this->Dt7.mouse.press_l = pData[12]; //!< Mouse Left Is Press ?
    this->Dt7.mouse.press_r = pData[13]; //!< Mouse Right Is Press ?

    this->Dt7.kb.key_code = pData[14] | (pData[15] << 8);                  //!< KeyBoard value
    this->Dt7.rc.ch[4] = ((int16_t)pData[16]) | ((int16_t)pData[17] << 8); // 左上角滑轮

    this->Dt7.rc.ch[0] -= RC_CH_VALUE_OFFSET;
    this->Dt7.rc.ch[1] -= RC_CH_VALUE_OFFSET;
    this->Dt7.rc.ch[2] -= RC_CH_VALUE_OFFSET;
    this->Dt7.rc.ch[3] -= RC_CH_VALUE_OFFSET;
    this->Dt7.rc.ch[4] -= RC_CH_VALUE_OFFSET;

    // 死区限制
    this->Dt7.rc.ch[0] = (ABS(this->Dt7.rc.ch[0]) < this->deadline_limt[0] ? 0 : this->Dt7.rc.ch[0]);
    this->Dt7.rc.ch[1] = (ABS(this->Dt7.rc.ch[1]) < this->deadline_limt[1] ? 0 : this->Dt7.rc.ch[1]);
    this->Dt7.rc.ch[2] = (ABS(this->Dt7.rc.ch[2]) < this->deadline_limt[2] ? 0 : this->Dt7.rc.ch[2]);
    this->Dt7.rc.ch[3] = (ABS(this->Dt7.rc.ch[3]) < this->deadline_limt[3] ? 0 : this->Dt7.rc.ch[3]);
    this->Dt7.rc.ch[4] = (ABS(this->Dt7.rc.ch[4]) < this->deadline_limt[4] ? 0 : this->Dt7.rc.ch[4]);

    this->Updata_ctrl(false);
#ifdef SEND_FORWARD
    this->Forward_by_Can(false);
#endif
}
/**
 * @brief 图传控制/裁判系统共用数据解析
 * @param DMA_RX_Buffer_idx 本次DMA接收数据缓冲池下标
 * @note  图传控制一次转发
 */
void ECF_RC::REFFEREE_DataProcess(uint8_t (*RX_Buffer)[REFEREE_RX_Buffer_Num], uint8_t DMA_RX_Buffer_idx, uint8_t datalen)
{
    uint8_t i;
    for (i = 0; i < datalen; i++)
    {
        if (RX_Buffer[DMA_RX_Buffer_idx][i] == 0xA5) // 帧头
        {
            if (Verify_CRC8_Check_Sum(&RX_Buffer[DMA_RX_Buffer_idx][i], HEADER_LEN) == 1) // 帧头CRC8校验
            {
                uint16_t Cmd_ID = ((RX_Buffer[DMA_RX_Buffer_idx][i + HEADER_LEN]) | (RX_Buffer[DMA_RX_Buffer_idx][i + HEADER_LEN + 1] << 8)); // 命令码ID
                switch (Cmd_ID)
                {
                case ID_STATE:
                    RefereeDataCRC16Deal(&this->REFFEREE.Game_Status, &RX_Buffer[DMA_RX_Buffer_idx][i], DATA_STATUS_LEN);
                    i = i + (DATA_STATUS_LEN + 9) + 9 - 1;
                    break;
                case ID_RESULT:
                    RefereeDataCRC16Deal(&this->REFFEREE.Game_Result, &RX_Buffer[DMA_RX_Buffer_idx][i], DATA_RESULT_LEN);
                    i = i + (DATA_RESULT_LEN + 9) - 1;
                    break;
                case ID_ROBOT_HP:
                    RefereeDataCRC16Deal(&this->REFFEREE.Robot_HP, &RX_Buffer[DMA_RX_Buffer_idx][i], DATA_ROBOT_HP_LEN);
                    i = i + (DATA_ROBOT_HP_LEN + 9) - 1;
                    break;
                case ID_EVENT_DATA:
                    RefereeDataCRC16Deal(&this->REFFEREE.Event_Data, &RX_Buffer[DMA_RX_Buffer_idx][i], DATA_EVENT_DATA_LEN);
                    i = i + (DATA_EVENT_DATA_LEN + 9) - 1;
                    break;
                case ID_SUPPLY_PROJECTILE_ACTION:
                    RefereeDataCRC16Deal(&this->REFFEREE.Supply_Action, &RX_Buffer[DMA_RX_Buffer_idx][i], DATA_SUPPLY_PROJECTILE_ACTION_LEN);
                    i = i + (DATA_SUPPLY_PROJECTILE_ACTION_LEN + 9) - 1;
                    break;
                case ID_REFEREE_WARNING:
                    RefereeDataCRC16Deal(&this->REFFEREE.Referee_Warning, &RX_Buffer[DMA_RX_Buffer_idx][i], DATA_REFEREE_WARNING_LEN);
                    i = i + (DATA_REFEREE_WARNING_LEN + 9) - 1;
                    break;
                case ID_DART_REMAINING_TIME:
                    RefereeDataCRC16Deal(&this->REFFEREE.Dart_Remaining_Time, &RX_Buffer[DMA_RX_Buffer_idx][i], DATA_DART_REMAINING_TIME_LEN);
                    i = i + (DATA_DART_REMAINING_TIME_LEN + 9) - 1;
                    break;
                case ID_ROBOT_STATE:
                    RefereeDataCRC16Deal(&this->REFFEREE.Robot_Status, &RX_Buffer[DMA_RX_Buffer_idx][i], DATA_ROBOT_STATUS_LEN);
                    i = i + (DATA_ROBOT_STATUS_LEN + 9) - 1;
                    break;
                case ID_POWER_HEAT_DATA:
                    RefereeDataCRC16Deal(&this->REFFEREE.Power_Heat, &RX_Buffer[DMA_RX_Buffer_idx][i], DATA_POWER_HEAT_DATA_LEN);
                    i = i + (DATA_POWER_HEAT_DATA_LEN + 9) - 1;
                    break;
                case ID_ROBOT_POS:
                    RefereeDataCRC16Deal(&this->REFFEREE.Robot_Position, &RX_Buffer[DMA_RX_Buffer_idx][i], DATA_ROBOT_POS_LEN);
                    i = i + (DATA_ROBOT_POS_LEN + 9) - 1;
                    break;
                case ID_BUFF:
                    RefereeDataCRC16Deal(&this->REFFEREE.Buff, &RX_Buffer[DMA_RX_Buffer_idx][i], DATA_BUFF_LEN);
                    i = i + (DATA_BUFF_LEN + 9) - 1;
                    break;
                case ID_AERIAL_ROBOT_ENERGY:
                    RefereeDataCRC16Deal(&this->REFFEREE.Aerial_Energy, &RX_Buffer[DMA_RX_Buffer_idx][i], DATA_AERIAL_ROBOT_ENERGY_LEN);
                    i = i + (DATA_AERIAL_ROBOT_ENERGY_LEN + 9) - 1;
                    break;
                case ID_ROBOT_HURT:
                    RefereeDataCRC16Deal(&this->REFFEREE.Robot_Hurt, &RX_Buffer[DMA_RX_Buffer_idx][i], DATA_ROBOT_HURT_LEN);
                    i = i + (DATA_ROBOT_HURT_LEN + 9) - 1;
                    break;
                case ID_SHOOT_DATA:
                    RefereeDataCRC16Deal(&this->REFFEREE.Shoot_Data, &RX_Buffer[DMA_RX_Buffer_idx][i], DATA_SHOOT_DATA_LEN);
                    i = i + (DATA_SHOOT_DATA_LEN + 9) - 1;
                    break;
                case ID_BULLET_REMAINING:
                    RefereeDataCRC16Deal(&this->REFFEREE.Bullet_Num, &RX_Buffer[DMA_RX_Buffer_idx][i], DATA_BULLET_REMAINING_LEN);
                    i = i + (DATA_BULLET_REMAINING_LEN + 9) - 1;
                    break;
                case ID_RFID_STATUS:
                    RefereeDataCRC16Deal(&this->REFFEREE.RFID_Status, &RX_Buffer[DMA_RX_Buffer_idx][i], DATA_RFID_STATUS_LEN);
                    i = i + (DATA_RFID_STATUS_LEN + 9) - 1;
                    break;
                case ID_DART_CLIENT_CMD:
                    RefereeDataCRC16Deal(&this->REFFEREE.Dart_Client, &RX_Buffer[DMA_RX_Buffer_idx][i], DATA_DART_CLIENT_CMD_LEN);
                    i = i + (DATA_DART_CLIENT_CMD_LEN + 9) - 1;
                    break;
                case ID_GROUND_ROBOT_POSITION:
                    RefereeDataCRC16Deal(&this->REFFEREE.Robot_Position_Al, &RX_Buffer[DMA_RX_Buffer_idx][i], DATA_ROBOT_POSITION_LEN);
                    i = i + (DATA_ROBOT_POSITION_LEN + 9) - 1;
                    break;
                case ID_RARD_MRAK_DATA:
                    RefereeDataCRC16Deal(&this->REFFEREE.radar_mark, &RX_Buffer[DMA_RX_Buffer_idx][i], DATA_RADAR_MARK_LEN);
                    i = i + (DATA_RADAR_MARK_LEN + 9) - 1;
                    break;
                case ID_SENTRY:
                    RefereeDataCRC16Deal(&this->REFFEREE.sentry, &RX_Buffer[DMA_RX_Buffer_idx][i], DATA_SENTRY_INFO_LEN);
                    i = i + (DATA_SENTRY_INFO_LEN + 9) - 1;
                    break;
                case ID_RADAR:
                    RefereeDataCRC16Deal(&this->REFFEREE.radar_info, &RX_Buffer[DMA_RX_Buffer_idx][i], DATA_RADAR_INFO_LEN);
                    i = i + (DATA_RADAR_INFO_LEN + 9) - 1;
                    break;
                // case 0x301:
                case ID_DIY_CONTROLLER:
                    RefereeDataCRC16Deal(&this->REFFEREE.DIY_control, &RX_Buffer[DMA_RX_Buffer_idx][i], DATA_DIY_CONTROLLER_LEN);
                    i = i + (DATA_DIY_CONTROLLER_LEN + 9) - 1;
                    break;
                case ID_CLIENT_DOWMLOAD: // 修订
                    RefereeDataCRC16Deal(&this->REFFEREE.ClientMapData, &RX_Buffer[DMA_RX_Buffer_idx][i], DATA_CLIENT_DOWMLOAD_LEN);
                    i = i + (DATA_CLIENT_DOWMLOAD_LEN + 9) - 1;
                    break;
                case ID_PICTURE_TRANSMISSION:
                    RefereeDataCRC16Deal(&this->REFFEREE.PHOTO_ctrl, &RX_Buffer[DMA_RX_Buffer_idx][i], DATA_PICTURE_TRANSMISSION_LEN);
                    i = i + (DATA_PICTURE_TRANSMISSION_LEN + 9) - 1;
                    break;
                default:
                    break;
                }
            }
        }
    }
}
/**
 * @brief DT7数据串口中断服务函数
 */
void ECF_RC::DT7_UART_Handler()
{
    uint16_t this_time_rx_len = 0;
    if (this->Dt7_huart->Instance->SR & UART_FLAG_RXNE) // 接收到数据
                                                        // //SR寄存器是状态寄存器，若其与UART_FLAG_RXNE（00010100）与运算有1，则说明有接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(this->Dt7_huart);
    }
    else if (this->Dt7_huart->Instance->SR & UART_FLAG_IDLE) // 串口处于空闲状态  （UART_FLAG_IDLE =
                                                             // 0：未检测到空闲线路 1：检测到空闲线路）
    {                                                        // 在空闲中断里判断数据帧的传送是否正确
        // 当串口开始接收数据后，检测到1字节数据的时间内没有数据发送，则认为串口空闲了。

        __HAL_UART_CLEAR_PEFLAG(this->Dt7_huart);

        if ((this->Dt7_huart->hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            // disable DMA`
            // 失效DMA
            __HAL_DMA_DISABLE(this->Dt7_huart->hdmarx);

            // get receive data length, length = set_data_length - remain_length
            // 获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = Sbus_RX_Buffer_Num - this->Dt7_huart->hdmarx->Instance->NDTR;
            // reset set_data_lenght
            // 重新设定数据长度
            this->Dt7_huart->hdmarx->Instance->NDTR = Sbus_RX_Buffer_Num;

            // set memory buffer 1
            // 设定缓冲区1
            this->Dt7_huart->hdmarx->Instance->CR |= DMA_SxCR_CT;

            // enable DMA
            // 使能DMA
            __HAL_DMA_ENABLE(this->Dt7_huart->hdmarx);

            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                this->DT7_DataProcess(0);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            // disable DMA
            // 失效DMA
            __HAL_DMA_DISABLE(this->Dt7_huart->hdmarx);

            // get receive data length, length = set_data_length - remain_length
            // 获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = Sbus_RX_Buffer_Num - this->Dt7_huart->hdmarx->Instance->NDTR;

            // reset set_data_lenght
            // 重新设定数据长度
            this->Dt7_huart->hdmarx->Instance->NDTR = Sbus_RX_Buffer_Num;

            // set memory buffer 0
            // 设定缓冲区0
            this->Dt7_huart->hdmarx->Instance->CR &= ~(DMA_SxCR_CT);

            // enable DMA
            // 使能DMA
            __HAL_DMA_ENABLE(this->Dt7_huart->hdmarx);

            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                // 处理遥控器数据
                this->DT7_DataProcess(1);
            }
        }
    }
}
/**
 * @brief 裁判系统数据串口中断服务函数
 */
void ECF_RC::REFFEREE_UART_Handler()
{
    uint16_t this_time_rx_len = 0;
    if (this->Referee_huart->Instance->SR & UART_FLAG_RXNE) // 接收到数据
                                                       // //SR寄存器是状态寄存器，若其与UART_FLAG_RXNE（00010100）与运算有1，则说明有接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(this->Referee_huart);
    }
    else if (this->Referee_huart->Instance->SR & UART_FLAG_IDLE) // 串口处于空闲状态  （UART_FLAG_IDLE =
                                                            // 0：未检测到空闲线路 1：检测到空闲线路）
    {                                                       // 在空闲中断里判断数据帧的传送是否正确
        // 当串口开始接收数据后，检测到1字节数据的时间内没有数据发送，则认为串口空闲了。

        __HAL_UART_CLEAR_PEFLAG(this->Referee_huart);

        if ((this->Referee_huart->hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            // disable DMA
            // 失效DMA
            __HAL_DMA_DISABLE(this->Referee_huart->hdmarx);

            // get receive data length, length = set_data_length - remain_length
            // 获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = REFEREE_RX_Buffer_Num - this->Referee_huart->hdmarx->Instance->NDTR;
            // reset set_data_lenght
            // 重新设定数据长度
            this->Referee_huart->hdmarx->Instance->NDTR = REFEREE_RX_Buffer_Num;

            // set memory buffer 1
            // 设定缓冲区1
            this->Referee_huart->hdmarx->Instance->CR |= DMA_SxCR_CT;

            // enable DMA
            // 使能DMA
            __HAL_DMA_ENABLE(this->Referee_huart->hdmarx);

            this->REFFEREE_DataProcess(this->Referee_RX_Buffer, 0, this_time_rx_len);
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            // disable DMA
            // 失效DMA
            __HAL_DMA_DISABLE(this->Referee_huart->hdmarx);

            // get receive data length, length = set_data_length - remain_length
            // 获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = REFEREE_RX_Buffer_Num - this->Referee_huart->hdmarx->Instance->NDTR;

            // reset set_data_lenght
            // 重新设定数据长度
            this->Referee_huart->hdmarx->Instance->NDTR = REFEREE_RX_Buffer_Num;

            // set memory buffer 0
            // 设定缓冲区0
            this->Referee_huart->hdmarx->Instance->CR &= ~(DMA_SxCR_CT);

            // enable DMA
            // 使能DMA
            __HAL_DMA_ENABLE(this->Referee_huart->hdmarx);

            // 处理图传链路遥控数据
            this->REFFEREE_DataProcess(this->Referee_RX_Buffer, 1, this_time_rx_len);
        }
    }
}
/**
 * @brief 图传数据串口中断服务函数
 */
void ECF_RC::PHOTO_UART_Handler()
{
    uint16_t this_time_rx_len = 0;
    if (this->Photo_huart->Instance->SR & UART_FLAG_RXNE) // 接收到数据
                                                       // //SR寄存器是状态寄存器，若其与UART_FLAG_RXNE（00010100）与运算有1，则说明有接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(this->Photo_huart);
    }
    else if (this->Photo_huart->Instance->SR & UART_FLAG_IDLE) // 串口处于空闲状态  （UART_FLAG_IDLE =
                                                            // 0：未检测到空闲线路 1：检测到空闲线路）
    {                                                       // 在空闲中断里判断数据帧的传送是否正确
        // 当串口开始接收数据后，检测到1字节数据的时间内没有数据发送，则认为串口空闲了。

        __HAL_UART_CLEAR_PEFLAG(this->Photo_huart);

        if ((this->Photo_huart->hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            // disable DMA
            // 失效DMA
            __HAL_DMA_DISABLE(this->Photo_huart->hdmarx);

            // get receive data length, length = set_data_length - remain_length
            // 获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = REFEREE_RX_Buffer_Num - this->Photo_huart->hdmarx->Instance->NDTR;
            // reset set_data_lenght
            // 重新设定数据长度
            this->Photo_huart->hdmarx->Instance->NDTR = REFEREE_RX_Buffer_Num;

            // set memory buffer 1
            // 设定缓冲区1
            this->Photo_huart->hdmarx->Instance->CR |= DMA_SxCR_CT;

            // enable DMA
            // 使能DMA
            __HAL_DMA_ENABLE(this->Photo_huart->hdmarx);

            this->REFFEREE_DataProcess(this->Photo_RX_Buffer, 0, this_time_rx_len);
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            // disable DMA
            // 失效DMA
            __HAL_DMA_DISABLE(this->Photo_huart->hdmarx);

            // get receive data length, length = set_data_length - remain_length
            // 获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = REFEREE_RX_Buffer_Num - this->Photo_huart->hdmarx->Instance->NDTR;

            // reset set_data_lenght
            // 重新设定数据长度
            this->Photo_huart->hdmarx->Instance->NDTR = REFEREE_RX_Buffer_Num;

            // set memory buffer 0
            // 设定缓冲区0
            this->Photo_huart->hdmarx->Instance->CR &= ~(DMA_SxCR_CT);

            // enable DMA
            // 使能DMA
            __HAL_DMA_ENABLE(this->Photo_huart->hdmarx);

            // 处理图传链路遥控数据
            this->REFFEREE_DataProcess(this->Photo_RX_Buffer, 1, this_time_rx_len);
        }
    }
}
/**
 * @brief 合并DT7与图传链路数据
 * @param Recive_forward 该数据来源为 其他板子 转发而来
 */
void ECF_RC::Updata_ctrl(bool Recive_forward)
{
    if (Recive_forward)
    {
        this->ctrl.kb.key_code = this->Dt7.kb.key_code | this->REFFEREE.PHOTO_ctrl.kb.key_code | Forward_ctrl.Struct.key_code;
        this->ctrl.mouse.x = this->Dt7.mouse.x | this->REFFEREE.PHOTO_ctrl.mouse.x | Forward_ctrl.Struct.mouseX;
        this->ctrl.mouse.y = this->Dt7.mouse.y | this->REFFEREE.PHOTO_ctrl.mouse.y | Forward_ctrl.Struct.mouseY;
        this->ctrl.mouse.z = this->Dt7.mouse.z | this->REFFEREE.PHOTO_ctrl.mouse.z;
        this->ctrl.mouse.press_l = this->Dt7.mouse.press_l | this->REFFEREE.PHOTO_ctrl.mouse.press_l | this->Forward_ctrl.Struct.mouseL_And_ch4_Set;
        this->ctrl.mouse.press_r = this->Dt7.mouse.press_r | this->REFFEREE.PHOTO_ctrl.mouse.press_r | this->Forward_ctrl.Struct.mouseR;
        this->ctrl.rc = this->Dt7.rc;
        if (this->Forward_ctrl.Struct.rs2 != RC_SW_ERROR)
        {
            this->ctrl.rc.ch[0] = this->Forward_ctrl.Struct.ch0 - 660;
            this->ctrl.rc.ch[1] = this->Forward_ctrl.Struct.ch1 - 660;
            this->ctrl.rc.s2 = this->Forward_ctrl.Struct.rs2;
        }
    }
    else
    {
        this->ctrl.kb.key_code = this->Dt7.kb.key_code | this->REFFEREE.PHOTO_ctrl.kb.key_code;
        this->ctrl.mouse.x = this->Dt7.mouse.x | this->REFFEREE.PHOTO_ctrl.mouse.x;
        this->ctrl.mouse.y = this->Dt7.mouse.y | this->REFFEREE.PHOTO_ctrl.mouse.y;
        this->ctrl.mouse.z = this->Dt7.mouse.z | this->REFFEREE.PHOTO_ctrl.mouse.z;
        this->ctrl.mouse.press_l = this->Dt7.mouse.press_l | this->REFFEREE.PHOTO_ctrl.mouse.press_l;
        this->ctrl.mouse.press_r = this->Dt7.mouse.press_r | this->REFFEREE.PHOTO_ctrl.mouse.press_r;
        this->ctrl.rc = this->Dt7.rc;
    }
}
// /**
//  * @brief 通过CAN转发数据
//  * @param From_TC 1：来自图传，不转发ch通道以及s2信息
//  * @note  阉割版转发数据
//  */

// void ECF_RC::Forward_by_Can(bool From_TC)
// {
//     Forward_ctrl_t Forward_ctrl;
//     Forward_ctrl.Struct.ch0 = this->ctrl.rc.ch[0] + 660;
//     Forward_ctrl.Struct.ch1 = this->ctrl.rc.ch[1] + 660;
//     Forward_ctrl.Struct.rs2 = this->ctrl.rc.s2;
//     if (From_TC)
//         Forward_ctrl.Struct.rs2 = RC_SW_ERROR;
//     // s2若为1，则代表忽略ch通道以及s2信息
//     Forward_ctrl.Struct.mouseX = this->ctrl.mouse.x;
//     Forward_ctrl.Struct.mouseY = this->ctrl.mouse.y;
//     Forward_ctrl.Struct.mouseR = this->ctrl.mouse.press_r;
//     Forward_ctrl.Struct.mouseL_And_ch4_Set = 0;
//     if (this->ctrl.rc.ch[4] >= 550)
//         Forward_ctrl.Struct.mouseL_And_ch4_Set = 1;
//     if (this->ctrl.mouse.press_l == 1)
//         Forward_ctrl.Struct.mouseL_And_ch4_Set = 1;
//     Forward_ctrl.Struct.key_code = this->ctrl.kb.key_code;

//     CAN_HandleTypeDef *hcan = &FORWAED_CAN;
//     CAN_TxHeaderTypeDef CAN_TxHeader = 
//     {
//         .StdId = FORWARD_CANID,
//         .IDE = CAN_ID_STD,
//         .RTR = CAN_RTR_DATA,
//         .DLC = 0x08,
//     };
//     uint32_t TxMailbox;
//     /*
//     HAL_CAN_AddTxMessage(hcan, &CAN_TxHeader,  Forward_ctrl.U8, &TxMailbox);
//     */
// }
void ECF_RC::Dt7_Clear(void)
{
    this->Dt7.kb.key_code = 0;
    this->Dt7.mouse = {0};
    memset(this->Dt7.rc.ch, 0, sizeof(int16_t) * 5);
}
void ECF_RC::TC_Clear(void)
{
    this->REFFEREE.PHOTO_ctrl.kb.key_code = 0;
    this->REFFEREE.PHOTO_ctrl.mouse = {0};
}
void ECF_RC::ECF_RC_Init()
{
    ;
}
ECF_RC *ECF_RC::Get_ECF_RC_Instance()
{
    return ECF_RC::ECF_RC_instance;
}

ECF_RC *ECF_RC::ECF_RC_instance = new ECF_RC;
ECF_RC *ECF_RC_instance = ECF_RC::Get_ECF_RC_Instance(); // 公共接口获取唯一实例

#ifdef RECIVE_DT7_CONTROL
static void DT7_disonline_clear(void)
{
    ECF_RC_instance->Dt7_Clear();
}

Safe_task_c DT7_Safe("DT7", 100, DT7_disonline_clear);

void ECF_DT7_UART_Handler()
{
    ECF_RC_instance->DT7_UART_Handler();
    DT7_Safe.Online();
}
#endif

#ifdef RECIVE_REFFEREE
void ECF_REFFEREE_UART_Handler()
{
    ECF_RC_instance->REFFEREE_UART_Handler();
}
#endif

#ifdef RECIVE_POHOTO_CONTROL
void ECF_REFFEREE_UART_Handler()
{
    ECF_RC_instance->PHOTO_UART_Handler();
}
static void TC_disonline_clear(void)
{
    ECF_RC_instance->TC_Clear();
}
Safe_task_c TC_Safe("TC", 100, TC_disonline_clear);
#endif

#ifdef RECIVE_FORWARD
static void Deal_Recive_Forward(CAN_RxHeaderTypeDef *Rxmessage, uint8_t *data)
{
    if (Rxmessage->StdId != FORWARD_CANID)
        return;
    memcpy(ECF_RC_instance->Forward_ctrl.U8, data, 8);
    ECF_RC_instance->Updata_ctrl(true);
}
#endif
void ECF_RC_Init()
{
#ifdef RECIVE_DT7_CONTROL
    DMA_Init(ECF_RC_instance->Dt7_huart, ECF_RC_instance->Sbus_RX_Buffer[0], ECF_RC_instance->Sbus_RX_Buffer[1], Sbus_RX_Buffer_Num);
#endif

#ifdef RECIVE_POHOTO_CONTROL
    DMA_Init(ECF_RC_instance->Photo_huart, ECF_RC_instance->Photo_RX_Buffer[0], ECF_RC_instance->Photo_RX_Buffer[1], REFEREE_RX_Buffer_Num);
#endif

#ifdef RECIVE_REFFEREE
    DMA_Init(ECF_RC_instance->Referee_huart, ECF_RC_instance->Referee_RX_Buffer[0], ECF_RC_instance->Referee_RX_Buffer[1], REFEREE_RX_Buffer_Num);
#endif

#ifdef RECIVE_FORWARD
// ECF_CAN_Rx_Callback_Register(FORWAED_BSPCAN, Bsp_Stdid, FORWARD_CANID, Deal_Recive_Forward);
#endif
}

/**
 * @brief 裁判系统数据段解析 与 整包CRC16校验
 * @param RefereeData 帧头解析出来的CMDID对应的细分数据结构体指针
 * @param frame_header 裁判系统此次通信的帧头指针
 * @param data_length  帧头解析出来的CMDID对应的细分数据结构体长度
 */
void ECF_RC::RefereeDataCRC16Deal(void *RefereeData, uint8_t*frame_header, uint8_t data_length)
{
    uint8_t *RefereeDataU8 = (uint8_t*)RefereeData;
    if (Verify_CRC16_Check_Sum(frame_header, HEADER_LEN + CMDID_LEN + data_length + CRC16_LEN) == 1)//整包CRC16校验
    {
        //校验通过，搬运到数据结构体内
        memcpy(RefereeData, &frame_header[HEADER_LEN + CMDID_LEN], data_length);
        //操作数据结构体内error，data_length不考虑数据结构体内error占用，因此指针移动后指向error所在字节
        memset(RefereeDataU8+data_length, 0, sizeof(uint8_t));
        this->Updata_ctrl(false);
        #ifdef SEND_FORWARD
        this->Forward_by_Can(true);
        #endif
    }
    else
        memset(RefereeDataU8+data_length, 1, sizeof(uint8_t));
}

const REFEREE_t* Get_REFFEREE(void){return &ECF_RC_instance->REFFEREE;}