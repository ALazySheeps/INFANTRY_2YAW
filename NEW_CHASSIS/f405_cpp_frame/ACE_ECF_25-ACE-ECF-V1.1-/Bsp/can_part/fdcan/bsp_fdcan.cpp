/*************************** Dongguan-University of Technology -ACE**************************
 * @file    bsp_fdcan.cpp
 * @author  KazuHa12441
 * @version V1.4
 * @date    2024/9/15
 * @brief
 *
 * @todo:
 *
 ********************************************************************************************
 * @verbatim
 *
 *
 *   https://blog.csdn.net/mengenqing/article/details/132583180
 ************************** Dongguan-University of Technology -ACE***************************/
#include "bsp_fdcan.hpp"
#include "memory.h"
#include "stdlib.h"

#include "bsp_dwt.hpp"

static void FDCANServiceInit()
{
    // 如果测试时，只需要一个CAN的话，在这里把其他fdcan的初始化注释掉
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0); // 启动中断
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0); // 启动中断
    HAL_FDCAN_Start(&hfdcan2);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0); // 启动中断
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0); // 启动中断
    HAL_FDCAN_Start(&hfdcan3);
    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0); // 启动中断
    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0); // 启动中断  
}

namespace BSP_CAN_Part_n
{
    FDCANInstance_c *FDCANInstance_c::fdcan_instance_[MX_REGISTER_CNT] = {nullptr};
    uint8_t CAN_Main_Class_c::idx_                                     = 0;

    // FDCAN结构体方式实例化构造函数
    FDCANInstance_c::FDCANInstance_c(FDCAN_Init_Config_s fdcan_config)
                : CAN_Main_Class_c(fdcan_config.tx_id, fdcan_config.rx_id),// 设置回调函数和接收发送id
                  fdcan_handle_(fdcan_config.fdcan_handle)
    {
        if(!idx_)
        {
            FDCANServiceInit();
        }
        if (fdcan_config.SAND_IDE == FDCAN_STANDARD_ID) // 标准帧
        {
            txconf.IdType     = FDCAN_STANDARD_ID;
            txconf.Identifier = this->tx_id_;
        } else // 拓展帧
        {
            txconf.IdType     = FDCAN_EXTENDED_ID;
            txconf.Identifier = this->tx_id_;
        }
        txconf.TxFrameType         = FDCAN_DATA_FRAME;   // 数据帧
        txconf.DataLength          = FDCAN_DLC_BYTES_8;  // 8位DLC
        txconf.ErrorStateIndicator = FDCAN_ESI_ACTIVE;   // 指定错误状态指示器（发送节点错误活跃
        txconf.BitRateSwitch       = FDCAN_BRS_OFF;      // 指定发送的T恤帧是带位率转换还是不带
        txconf.FDFormat            = FDCAN_CLASSIC_CAN;  // 指定发送帧是classic 还是fd
        txconf.TxEventFifoControl  = FDCAN_NO_TX_EVENTS; // 指定帧开始时捕获的时间戳计数器值传播（不存储tx事件
        txconf.MessageMarker       = 0;                  // 指定复制到Tx EventFIFO元素中的消息标记用于识别Tx信息状态
        fdcan_instance_[idx_++]    = this;
        if (idx_ >= MX_REGISTER_CNT) 
        {
            while (1) 
            {
                // 卡在这就是负载太多了
            }
        }
        Filter_Config(this);

    }

    /// @brief FDCAN实例化构造函数(继承后可以用这个)
    /// @param fdcan_handle fdcan句柄
    /// @param tx_id        发送id
    /// @param rx_id        接收id|电机id
    /// @param SAND_IDE     帧类型
    /// @param fdcan_module_callback 对反馈值进行处理的回调函数指针
    FDCANInstance_c::FDCANInstance_c(FDCAN_HandleTypeDef *fdcan_handle,
                                     uint32_t tx_id,
                                     uint32_t rx_id,
                                     uint32_t SAND_IDE)
        : CAN_Main_Class_c(tx_id, rx_id),
          fdcan_handle_(fdcan_handle)

    {
        if(!idx_)
        {
            FDCANServiceInit();
        }
        if (SAND_IDE == FDCAN_STANDARD_ID) // 标准帧
        {
            txconf.IdType     = FDCAN_STANDARD_ID;
            txconf.Identifier = tx_id_;
        } else // 拓展帧
        {
            txconf.IdType     = FDCAN_EXTENDED_ID;
            txconf.Identifier = tx_id_;
        }
        txconf.TxFrameType         = FDCAN_DATA_FRAME;   // 数据帧
        txconf.DataLength          = FDCAN_DLC_BYTES_8;  // 8位DLC
        txconf.ErrorStateIndicator = FDCAN_ESI_ACTIVE;   // 指定错误状态指示器（发送节点错误活跃
        txconf.BitRateSwitch       = FDCAN_BRS_OFF;      // 指定发送的T恤帧是带位率转换还是不带
        txconf.FDFormat            = FDCAN_CLASSIC_CAN;  // 指定发送帧是classic 还是fd
        txconf.TxEventFifoControl  = FDCAN_NO_TX_EVENTS; // 指定帧开始时捕获的时间戳计数器值传播（不存储tx事件
        txconf.MessageMarker       = 0;                  // 指定复制到Tx EventFIFO元素中的消息标记用于识别Tx信息状态
        fdcan_instance_[idx_++]    = this;
    }

    /// @brief 若直接调用调试可以用这个
    /// @param fdcan_handle
    /// @param tx_id
    /// @param rx_id
    /// @param SAND_IDE
    /// @param fdcan_module_callback
    FDCANInstance_c::FDCANInstance_c(
        FDCAN_HandleTypeDef *fdcan_handle,
        uint32_t tx_id,
        uint32_t rx_id,
        uint32_t SAND_IDE,
        void (*fdcan_module_callback)(FDCANInstance_c *register_instance))
        : CAN_Main_Class_c(tx_id, rx_id),
          fdcan_handle_(fdcan_handle),
          fdcan_module_callback(fdcan_module_callback)
    {
        // 初始化CAN外设
        if(!idx_)
        {
            FDCANServiceInit();
        }
        if (SAND_IDE == FDCAN_STANDARD_ID) // 标准帧
        {
            txconf.IdType     = FDCAN_STANDARD_ID;
            txconf.Identifier = tx_id_;
        } else // 拓展帧
        {
            txconf.IdType     = FDCAN_EXTENDED_ID;
            txconf.Identifier = tx_id_;
        }
        txconf.TxFrameType         = FDCAN_DATA_FRAME;   // 数据帧
        txconf.DataLength          = FDCAN_DLC_BYTES_8;  // 8位DLC
        txconf.ErrorStateIndicator = FDCAN_ESI_ACTIVE;   // 指定错误状态指示器（发送节点错误活跃
        txconf.BitRateSwitch       = FDCAN_BRS_OFF;      // 指定发送的T恤帧是带位率转换还是不带
        txconf.FDFormat            = FDCAN_CLASSIC_CAN;  // 指定发送帧是classic 还是fd
        txconf.TxEventFifoControl  = FDCAN_NO_TX_EVENTS; // 指定帧开始时捕获的时间戳计数器值传播（不存储tx事件
        txconf.MessageMarker       = 0;                  // 指定复制到Tx EventFIFO元素中的消息标记用于识别Tx信息状态
        fdcan_instance_[idx_++]    = this;

        if (idx_ >= MX_REGISTER_CNT) {
            while (1) 
            {
                // 卡在这就是负载太多了
            }
        }
        Filter_Config(this);
    }

    /// @brief 继承后用可以将回调函数编写好后，调用此函数进行回调
    /// @param fdcan_module_callback
    void FDCANInstance_c::ECF_SetRxCallBack(void (*fdcan_module_callback)(FDCANInstance_c *register_instance))
    {
        this->fdcan_module_callback = fdcan_module_callback;
    }

    FDCANInstance_c::FDCANInstance_c(
        uint32_t tx_id,
        uint32_t rx_id)
        : BSP_CAN_Part_n::CAN_Main_Class_c(tx_id, rx_id)// 设置回调函数和接收发送id
    {

    }

    // 仅用于DJIMotor层的9个FDCAN发送对象的构造
    void FDCANInstance_c::FDCAN_Motor_Init(FDCAN_HandleTypeDef *fdcan_handle,uint32_t SAND_IDE)
    {
        this->fdcan_handle_ = fdcan_handle;
        if(!idx_)
        {
            FDCANServiceInit();
        }
        if (SAND_IDE == FDCAN_STANDARD_ID) // 标准帧
        {
            txconf.IdType     = FDCAN_STANDARD_ID;
            txconf.Identifier = this->tx_id_;
        } else // 拓展帧
        {
            txconf.IdType     = FDCAN_EXTENDED_ID;
            txconf.Identifier = this->tx_id_;
        }
        txconf.TxFrameType         = FDCAN_DATA_FRAME;   // 数据帧
        txconf.DataLength          = FDCAN_DLC_BYTES_8;  // 8位DLC
        txconf.ErrorStateIndicator = FDCAN_ESI_ACTIVE;   // 指定错误状态指示器（发送节点错误活跃
        txconf.BitRateSwitch       = FDCAN_BRS_OFF;      // 指定发送的T恤帧是带位率转换还是不带
        txconf.FDFormat            = FDCAN_CLASSIC_CAN;  // 指定发送帧是classic 还是fd
        txconf.TxEventFifoControl  = FDCAN_NO_TX_EVENTS; // 指定帧开始时捕获的时间戳计数器值传播（不存储tx事件
        txconf.MessageMarker       = 0;                  // 指定复制到Tx EventFIFO元素中的消息标记用于识别Tx信息状态
        Filter_Config(this);
    }
    /// @brief 修改DLC
    /// @param length 数据长度
    void FDCANInstance_c::ECF_SetDLC(uint32_t length)
    {
        // 发送长度错误!检查调用参数是否出错,或出现野指针/越界访问
        if (length > 8 || length == 0) // 安全检查
        {
            while (1) 
            {
            }
        }
        txconf.DataLength = length;
    }
    FDCAN_HandleTypeDef* FDCANInstance_c::ECF_GetFDCanhandle()
    {
        return this->fdcan_handle_;
    }
    /// @brief FDCAN发送函数
    /// @param timeout 延时时间
    /// @return 返回发送状态
    CAN_State_e FDCANInstance_c::ECF_Transmit(float timeout)
    {
        BSP_DWT_n::BSP_DWT_c *dwt_time_fdcan = BSP_DWT_n::BSP_DWT_c::ECF_Get_DwtInstance(); // 获取DWT实例
        float dwt_start     = dwt_time_fdcan->ECF_DWT_GetTimeline_ms();
        dwt_time_fdcan->ECF_DWT_Delay_ms(timeout);
        while (HAL_FDCAN_GetTxFifoFreeLevel(this->fdcan_handle_) == 0); // 等待邮箱空闲
        tx_wait_time_ = dwt_time_fdcan->ECF_DWT_GetTimeline_ms() - dwt_start;
        if (HAL_FDCAN_AddMessageToTxFifoQ(fdcan_handle_, &txconf, tx_buff)) 
        {
            return CAN_ERROR;
        }
         return CAN_OK; // 发送成功
    }

    /// @brief 过滤器设置
    /// @param instance 实例指针
    void FDCANInstance_c::Filter_Config(FDCANInstance_c *instance)
    {
        FDCAN_FilterTypeDef filter;
        assert_param(instance->fdcan_handle_ != NULL);
        filter.IdType           = instance->txconf.IdType; // 32位工作
        filter.FilterType       = FDCAN_FILTER_RANGE;      // 范围过滤
        filter.FilterIndex      = 0;                       // bank
        filter.FilterID1        = 0;                       // 传统模式
        filter.FilterID2        = 0;
        filter.RxBufferIndex    = 0x0000;
        filter.IsCalibrationMsg = 0;
        filter.FilterConfig     = FDCAN_FILTER_TO_RXFIFO0;
        if (HAL_FDCAN_ConfigFilter(instance->fdcan_handle_, &filter) != HAL_OK) {
            Error_Handler();
        }
    }

    /// @brief 回调函数，在接收之后回调解析数据
    /// @param _hfdcan fdcan句柄
    /// @param fifox   哪个fifo
    void FDCANInstance_c::ECF_FIFOxCallback(FDCAN_HandleTypeDef *_hfdcan, uint32_t fifox)
    {
        static FDCAN_RxHeaderTypeDef rxconf; // 数据身份信息
        uint8_t fdcan_rx_data[8];
        while (HAL_FDCAN_GetRxFifoFillLevel(_hfdcan, fifox)) // FIFO不为空,有可能在其他中断时有多帧数据进入
        {
            HAL_FDCAN_GetRxMessage(_hfdcan, fifox, &rxconf, fdcan_rx_data); // 从FIFO中获取数据
            for (size_t i = 0; i < idx_; i++) {
                // 两者相等说明这是要找的实例
                if (_hfdcan == fdcan_instance_[i]->fdcan_handle_ && (((rxconf.IdType = FDCAN_STANDARD_ID && rxconf.Identifier == fdcan_instance_[i]->rx_id_) || // 标准帧需要检测ID是否匹配
                                                                      (rxconf.IdType = FDCAN_EXTENDED_ID))                                                      // 如果是拓展帧的话,则全部接受（DM电机用）
                                                                     )) {
                    if (fdcan_instance_[i]->fdcan_module_callback != NULL) // 回调函数不为空就调用
                    {
                        fdcan_instance_[i]->rx_len_ = rxconf.DataLength;                       // 保存接收到的数据长度
                        memcpy(fdcan_instance_[i]->rx_buff, fdcan_rx_data, rxconf.DataLength); // 消息拷贝到对应实例
                        fdcan_instance_[i]->fdcan_module_callback(fdcan_instance_[i]);         // 触发回调进行数据解析和处理
                    }
                    return;
                }
            }
        }
    }
}

/**
 * @brief rx fifo callback. Once FIFO_0 is full,this func would be called
 *
 * @param hcan CAN handle indicate which device the oddest mesg in FIFO_0 comes from
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    BSP_CAN_Part_n::FDCANInstance_c::ECF_FIFOxCallback(hfdcan, FDCAN_RX_FIFO0); // 调用我们自己写的函数来处理消息
}

/**
 * @brief rx fifo callback. Once FIFO_1 is full,this func would be called
 *
 * @param hcan CAN handle indicate which device the oddest mesg in FIFO_1 comes from
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    BSP_CAN_Part_n::FDCANInstance_c::ECF_FIFOxCallback(hfdcan, FDCAN_RX_FIFO1); // 调用我们自己写的函数来处理消息
}