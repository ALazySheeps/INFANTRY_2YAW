/*************************** Dongguan-University of Technology -ACE**************************
 * @file    bsp_can.cpp
 * @author  study-sheep & KazuHa12441
 * @version V1.1
 * @date    2024/9/14
 * @brief   CAN的BSP层文件
 ******************************************************************************
 * @verbatim
 *  支持CAN1和CAN2，支持标准帧和扩展帧
 *  使用方法：
 *      在目标文件里面定义一个CANInstance_c对象，并传入can句柄，发送id，接收id，标准帧/拓展帧，回调函数。
 *      然后就可以用该对象愉快地调用他的成员函数了。
 *  demo：
 *      // 定义一个CANInstance_c对象
 *      @param hcan1    hcan句柄
 *      @param 0x123         发送id
 *      @param 0x456         接收id
 *      @param CAN_ID_EXT  CAN报文类型 拓展帧
 *      @param dji_motor_callback 回调函数
 *      void dji_motor_callback(CANInstance_c* register_instance)
 *      {
 *
 *      }
 *      // can传输函数，发送数据帧到总线，用于驱动电机/CAN双板/多板通信
 *      @param  5 是邮箱堵塞的等待的最大时间,单位ms
        int16_t set = 1000;
        BSP_CAN_Part_n::CANInstance_c dji_motor2(&hcan1, 0x200, 0x201, CAN_ID_STD, dji_motor_callback);
        dji_motor2.tx_buff[0] = set >> 8;
        dji_motor2.tx_buff[1] = set;
        dji_motor2.tx_buff[2] = set >> 8;
        dji_motor2.tx_buff[3] = set;
        dji_motor2.tx_buff[4] = set >> 8;
        dji_motor2.tx_buff[5] = set;
        dji_motor2.tx_buff[6] = set >> 8;
        dji_motor2.tx_buff[7] = set;
        while (1)
        {
            dji_motor2.ECF_Transmit(1); // 发送数据   已测试，别喷我！！！
        }
 *      // 修改CAN发送报文的数据帧长度
 *      @param  7        数据长度
 *      dji_motor1.ECF_CAN_SetDLC(7); // 设置数据长度
 *
 * @attention
 *      在ECF_CANFIFOxCallback函数里面的条件判断(rxconf.IDE = CAN_ID_EXT) 目前拓展帧就用到DM电机，他的Extid是包含消息，是不断变换的。
 *      如果其他地方用到拓展帧，需要Extid完全匹配的（ID不会发生变化），可以考虑使用ID List Mode
 *
 *      目前把CAN过滤器设置为ID Mask Mode,过滤器不过滤，由后面CAN接收中断函数来判断接收的数据是不是要使用
 *      如果想要开启ID List Mode，在CANFilter_Config函数里面把部分代码注释取消，加上两行注释的代码即可，
 *      或者考虑是不是要CANInstance_c加上一个成员变量，来判断使用实例使用 ID Mask Mode  或者  ID List Mode
 * @version                                                  time
 * v1.0   基础版本                                           2024-9-11
 * v1.1   C++优化版本                                        2024-9-16
 * v1.2   修复了CAN外设初始化忘记把can1_is_置为OK的bug🤡      2024-9-16
 * v1.3   对宏定义进行修改，调用抽象类进行继承重写，优化文件结构 2024-9-20
 * v1.4   新增修改发送id的函数，用于发送修改后的扩展帧，新增对扩展帧接收更新 2024-12-1
 ************************** Dongguan-University of Technology -ACE***************************/
// 本文件头文件
#include "bsp_can.hpp"

#include "memory.h"
#include "stdlib.h"

#include "bsp_dwt.hpp"
/**
 * @brief 在第一个CAN实例初始化的时候会自动调用此函数,启动CAN服务
 *
 * @note 此函数会启动CAN1和CAN2,开启CAN1和CAN2的FIFO0 & FIFO1溢出通知
 *
 */
static void CANServiceInit()
{
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING); // 如果测试时，只需要一个CAN的话，在这里把另一个can的初始化注释掉
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING); // F4只有两个CAN  H7 老老实实 用FDCAN文件，兼容传统的CAN和FDCAN的设置
}

namespace BSP_CAN_Part_n
{
    // 初始化CAN实例指针数组
    CANInstance_c *CANInstance_c::can_instance_[MX_REGISTER_CNT] = {nullptr};
    // 初始化CAN实例指针数组下标
    uint8_t CAN_Main_Class_c::idx_ = 0;
    /**
     * @brief CANInstance_c构造函数
     *
     * @param can_handle    hcan句柄
     * @param tx_id         发送id
     * @param rx_id         接收id
     * @param SAND_CanIDE  CAN报文类型，标准帧/拓展帧
     * @param can_module_callback 回调函数
     **/
    // 用于调试的CAN构造函数
    CANInstance_c::CANInstance_c(
        CAN_HandleTypeDef *can_handle,
        uint32_t tx_id,
        uint32_t rx_id,
        uint32_t SAND_CanIDE,
        void (*can_module_callback)(CANInstance_c *register_instance))
        : CAN_Main_Class_c(tx_id, rx_id), // 设置回调函数和接收发送id
          can_handle_(can_handle),
          can_module_callback(can_module_callback)
    {
        // 初始化CAN外设
        if (!idx_) {
            CANServiceInit();
        }
        // 进行发送报文的配置
        if (SAND_CanIDE == CAN_ID_STD) {
            txconf.IDE   = CAN_ID_STD; // 使用标准id
            txconf.StdId = tx_id_;     // 发送id
        } else {
            txconf.IDE   = CAN_ID_EXT; // 使用扩展id
            txconf.ExtId = tx_id_;     // 发送id
        }
        txconf.RTR = CAN_RTR_DATA; // 发送数据帧(目前没有远程帧的需求)
        txconf.DLC = 0x08;         // 默认发送长度为8
        // 将当前实例加入指针数组中
        can_instance_[idx_++] = this;
        if (idx_ >= MX_REGISTER_CNT) {
            // 超过最大实例数，错误处理
            while (true) {
            }
        }
        Filter_Config(this); // 添加CAN过滤器规则
    }

    CANInstance_c::CANInstance_c(
        CAN_HandleTypeDef *can_handle,
        uint32_t tx_id,
        uint32_t rx_id,
        uint32_t SAND_CanIDE
        )
        : CAN_Main_Class_c(tx_id, rx_id), // 设置回调函数和接收发送id
          can_handle_(can_handle)
    {
        // 初始化CAN外设
        if (!idx_) {
            CANServiceInit();
        }
        // 进行发送报文的配置
        if (SAND_CanIDE == CAN_ID_STD) {
            txconf.IDE   = CAN_ID_STD; // 使用标准id
            txconf.StdId = tx_id_;     // 发送id
        } else {
            txconf.IDE   = CAN_ID_EXT; // 使用扩展id
            txconf.ExtId = tx_id_;     // 发送id
        }
        txconf.RTR = CAN_RTR_DATA; // 发送数据帧(目前没有远程帧的需求)
        txconf.DLC = 0x08;         // 默认发送长度为8
        // 将当前实例加入指针数组中
        can_instance_[idx_++] = this;
        if (idx_ >= MX_REGISTER_CNT) {
            // 超过最大实例数，错误处理
            while (true) {
            }
        }
        Filter_Config(this); // 添加CAN过滤器规则
    }

    CANInstance_c::CANInstance_c(
        uint32_t tx_id,
        uint32_t rx_id)
        : BSP_CAN_Part_n::CAN_Main_Class_c(tx_id, rx_id)
    {
    }
    // 仅用于DJIMotor层的6个CAN发送对象的构造
    void CANInstance_c::CAN_Motor_Init(CAN_HandleTypeDef *can_handle, uint32_t SAND_CanIDE)
    {
        this->can_handle_ = can_handle;
        if (!idx_) {
            CANServiceInit();
        }
        // 进行发送报文的配置
        if (SAND_CanIDE == CAN_ID_STD) {
            txconf.IDE   = CAN_ID_STD;   // 使用标准id
            txconf.StdId = this->tx_id_; // 发送id
        } else {
            txconf.IDE   = CAN_ID_EXT;   // 使用扩展id
            txconf.ExtId = this->tx_id_; // 发送id
        }
        txconf.RTR = CAN_RTR_DATA; // 发送数据帧(目前没有远程帧的需求)
        txconf.DLC = 0x08;         // 默认发送长度为8
        Filter_Config(this);       // 添加CAN过滤器规则
    }

    // 上层模块调用的CAN构造函数
    CANInstance_c::CANInstance_c(CAN_Init_Config_s can_config)
        : CAN_Main_Class_c(can_config.tx_id, can_config.rx_id), // 设置回调函数和接收发送id
          can_handle_(can_config.can_handle)
    {
        if (!idx_) {
            CANServiceInit();
        }
        // 进行发送报文的配置
        if (can_config.SAND_IDE == CAN_ID_STD) {
            txconf.IDE   = CAN_ID_STD;       // 使用标准id
            txconf.StdId = can_config.tx_id; // 发送id
        } else {
            txconf.IDE   = CAN_ID_EXT;       // 使用扩展id
            txconf.ExtId = can_config.tx_id; // 发送id
        }
        txconf.RTR = CAN_RTR_DATA; // 发送数据帧(目前没有远程帧的需求)
        txconf.DLC = 0x08;         // 默认发送长度为8
        // 将当前实例加入指针数组中
        can_instance_[idx_++]     = this;
        this->can_module_callback = NULL;
        if (idx_ >= MX_REGISTER_CNT) {
            // 超过最大实例数，错误处理
            while (true) {
            }
        }
        Filter_Config(this); // 添加CAN过滤器规则
    }

    void CANInstance_c::ECF_SetRxCallBack(void (*can_module_callback)(CANInstance_c *register_instance))
    {
        this->can_module_callback = can_module_callback;
    }

    /**
     * @brief 修改CAN发送报文的数据帧长度;注意最大长度为8,在没有进行修改的时候,默认长度为8
     *
     * @param length    设定长度
     */
    void CANInstance_c::ECF_SetDLC(uint32_t length)
    {
        // 发送长度错误!检查调用参数是否出错,或出现野指针/越界访问
        if (length > 8 || length == 0) // 安全检查
        {
            while (1) {
            }
        }
        this->txconf.DLC = length;
    }

    /**
     * @brief CAN传输函数，发送数据帧到总线，用于驱动电机
     * @param timeout 设定超时时间，单位ms
     * **/
    CAN_State_e CANInstance_c::ECF_Transmit(float timeout)
    {
        BSP_DWT_n::BSP_DWT_c *dwt_time_can = BSP_DWT_n::BSP_DWT_c::ECF_Get_DwtInstance();
        float dwt_start                    = dwt_time_can->ECF_DWT_GetTimeline_ms();
        //dwt_time_can->ECF_DWT_Delay_ms(timeout);
        while (HAL_CAN_GetTxMailboxesFreeLevel(can_handle_) == 0); // 等待邮箱空闲
        this->tx_wait_time_ = dwt_time_can->ECF_DWT_GetTimeline_ms() - dwt_start;
        // tx_mailbox会保存实际填入了这一帧消息的邮箱,但是知道是哪个邮箱发的似乎也没啥用
        //修改扩展帧发送ID
        if (txconf.IDE == CAN_ID_EXT) {
            this->txconf.ExtId = this->tx_id_;
        } 
        if (HAL_CAN_AddTxMessage(can_handle_, &txconf, tx_buff, &tx_mailbox)) {
            return CAN_ERROR;
        }
        return CAN_OK; // 发送成功
    }

    CAN_HandleTypeDef *CANInstance_c::ECF_GetCanhandle()
    {
        return this->can_handle_;
    }

    //用于扩展帧修改发送ID
    void CANInstance_c::Chance_Ext_Tx_ID(uint32_t ext_id)
    {
        this->tx_id_ = ext_id;
    }

    /**
     * @brief 添加过滤器以实现对特定id的报文的接收,会被CANInstance_c的构造函数调用
     *        给CAN添加过滤器后,BxCAN会根据接收到的报文的id进行消息过滤,符合规则的id会被填入FIFO触发中断
     *
     * @note f407的bxCAN有28个过滤器,这里将其配置为前14个过滤器给CAN1使用,后14个被CAN2使用
     *       初始化时,奇数id的模块会被分配到FIFO0,偶数id的模块会被分配到FIFO1
     *       注册到CAN1的模块使用过滤器0-13,CAN2使用过滤器14-27
     *
     * @attention 你不需要完全理解这个函数的作用,因为它主要是用于初始化,在开发过程中不需要关心底层的实现
     *            享受开发的乐趣吧!如果你真的想知道这个函数在干什么,请联系作者或自己查阅资料(请直接查阅官方的reference manual)
     *
     * @param _instance can instance owned by specific module
     */
    void CANInstance_c::Filter_Config(CANInstance_c *instance)
    {
        CAN_FilterTypeDef can_filter_init_structure;
        static uint8_t can1_filter_idx = 0, can2_filter_idx = 14; // 0-13给can1用,14-27给can2用
        // 检测关键传参
        assert_param(instance->can_handle != NULL);
        if (instance->txconf.IDE == CAN_ID_STD || instance->txconf.IDE == CAN_ID_EXT) {
            // 数据帧
            // 掩码后ID的高16bit
            can_filter_init_structure.FilterIdHigh = 0x00;
            // 掩码后ID的低16bit
            can_filter_init_structure.FilterIdLow = 0x00;
            // ID掩码值高16bit
            can_filter_init_structure.FilterMaskIdHigh = 0x00;
            // ID掩码值低16bit
            can_filter_init_structure.FilterMaskIdLow = 0x00;
        } else {
            // 遥控帧
            // 掩码后ID的高16bit
            can_filter_init_structure.FilterIdHigh = instance->txconf.StdId;
            // 掩码后ID的低16bit
            can_filter_init_structure.FilterIdLow = ((instance->txconf.IDE & 0x03) << 1);
            // ID掩码值高16bit
            can_filter_init_structure.FilterMaskIdHigh = 0;
            // ID掩码值低16bit
            can_filter_init_structure.FilterMaskIdLow = 0;
        }

        can_filter_init_structure.FilterMode  = CAN_FILTERMODE_IDMASK;
        can_filter_init_structure.FilterScale = CAN_FILTERSCALE_32BIT;

        // 滤波器序号, 0-27, 共28个滤波器, can1是0~13, can2是14~27
        can_filter_init_structure.FilterBank = instance->can_handle_ == &hcan1 ? (can1_filter_idx++) : (can2_filter_idx++); // 根据can_handle判断是CAN1还是CAN2,然后自增;
        // 滤波器绑定FIFOx, 只能绑定一个
        can_filter_init_structure.FilterFIFOAssignment = (instance->tx_id_ & 1) ? CAN_RX_FIFO0 : CAN_RX_FIFO1; // 奇数id的模块会被分配到FIFO0,偶数id的模块会被分配到FIFO1
        // 使能滤波器
        can_filter_init_structure.FilterActivation = CAN_FILTER_ENABLE;
        // 从机模式选择开始单元
        can_filter_init_structure.SlaveStartFilterBank = 14; // 从第14个过滤器开始配置从机过滤器(在STM32的BxCAN控制器中CAN2是CAN1的从机

        HAL_CAN_ConfigFilter(instance->can_handle_, &can_filter_init_structure);
    }

    /* -----------------------belows are callback definitions--------------------------*/
static CAN_RxHeaderTypeDef rxconf; // 数据身份信息
    /**
     * @brief 此函数会被下面两个函数调用,用于处理FIFO0和FIFO1溢出中断(说明收到了新的数据)
     *        所有的实例都会被遍历,找到can_handle和rx_id相等的实例时,调用该实例的回调函数
     *
     * @param _hcan
     * @param fifox passed to HAL_CAN_GetRxMessage() to get mesg from a specific fifo
     */
    void CANInstance_c::ECF_FIFOxCallback(CAN_HandleTypeDef *_hcan, uint32_t fifox)
    {
        
        uint8_t can_rx_data[8];
        while (HAL_CAN_GetRxFifoFillLevel(_hcan, fifox)) // FIFO不为空,有可能在其他中断时有多帧数据进入
        {
            HAL_CAN_GetRxMessage(_hcan, fifox, &rxconf, can_rx_data); // 从FIFO中获取数据
            for (size_t i = 0; i < idx_; ++i) {
                // 两者相等说明这是要找的实例
                if (_hcan == can_instance_[i]->can_handle_ && (((rxconf.IDE == CAN_ID_EXT)||(rxconf.IDE == CAN_ID_STD && rxconf.StdId == can_instance_[i]->rx_id_)  // 标准帧需要检测ID是否匹配
                                                                )                                               // 如果是拓展帧的话,则全部接受（DM电机用）
                                                               )) {
                    if (can_instance_[i]->can_module_callback != NULL) // 回调函数不为空就调用
                    {
                        can_instance_[i]->rx_id_ = (rxconf.IDE == CAN_ID_EXT)? rxconf.ExtId : rxconf.StdId; // 保存接收到的数据ID
                        can_instance_[i]->rx_len_ = rxconf.DLC;                     // 保存接收到的数据长度
                        memcpy(can_instance_[i]->rx_buff, can_rx_data, rxconf.DLC); // 消息拷贝到对应实例
                        can_instance_[i]->can_module_callback(can_instance_[i]);    // 触发回调进行数据解析和处理
                    }
                    memset(&rxconf, 0, sizeof(rxconf));
                    return;
                }
            }
        }
    }
}

/**
 * @brief 注意,STM32的两个CAN设备共享两个FIFO
 * 下面两个函数是HAL库中的回调函数,他们被HAL声明为__weak,这里对他们进行重载(重写)
 * 当FIFO0或FIFO1溢出时会调用这两个函数
 */
// 下面的函数会调用CANFIFOxCallback()来进一步处理来自特定CAN设备的消息

/**
 * @brief rx fifo callback. Once FIFO_0 is full,this func would be called
 *
 * @param hcan CAN handle indicate which device the oddest mesg in FIFO_0 comes from
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    BSP_CAN_Part_n::CANInstance_c::ECF_FIFOxCallback(hcan, CAN_RX_FIFO0); // 调用我们自己写的函数来处理消息
}

/**
 * @brief rx fifo callback. Once FIFO_1 is full,this func would be called
 *
 * @param hcan CAN handle indicate which device the oddest mesg in FIFO_1 comes from
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    BSP_CAN_Part_n::CANInstance_c::ECF_FIFOxCallback(hcan, CAN_RX_FIFO1); // 调用我们自己写的函数来处理消息
}