/**
*****************************东莞理工学院ACE实验室*****************************
* @file 		bsp_referee.c
*
* @brief 		包括裁判系统初始化，裁判系统数据获取，裁判系统通讯协议的解析
* @author   叶彦均
* @note  		
* @history  全新升级，支持hal库
* Date       Version Author Description
*              1.0   叶彦均 全新升级，支持hal库
* 2024-3-13    1.1   郑楠   协议更新
* 2025-1-13    1.2   study-sheep H7芯片裁判系统适配
@verbatim
==============================================================================

ECF_referee_uart_init()丢到初始化
用Get_referee_Address()获取裁判系统数据
==============================================================================
@endverbatim
*****************************东莞理工学院ACE实验室******************************/


#include "bsp_referee.hpp"

extern UART_HandleTypeDef huart6;
// 缓冲区
uint8_t rx_1[256] = {0};
REFEREE_t referee;///裁判系统数据
void RefereeDataDeal(REFEREE_t *referee);
// 接收回调函数,防止调用出现野指针
void tx_call()
{
}
void rx_call(uint8_t *buff, uint16_t size)
{
	memcpy(referee.RefereeData, buff, size);
	referee.DataLen = size;
	RefereeDataDeal(&referee);
}

/*裁判数据接收数据处理*/
void RefereeDataDeal(REFEREE_t *referee);
/*比赛状态*/
static void GAME_STATUS(REFEREE_t *referee, unsigned char k);
/*比赛结果数据*/
static void GAME_RESULT(REFEREE_t *referee, unsigned char k);
/*机器人血量状态数据*/
static void ROBOT_HP(REFEREE_t *referee, unsigned char k);
/*补给站动作标识*/
static void SUPPLY_PROJECTILE_ACTION(REFEREE_t *referee, unsigned char k);
/*裁判警告数据*/
static void REFEREE_WARNING(REFEREE_t *referee, unsigned char k);
/*飞镖发射口倒计时*/
static void DART_REAMAINING_TIME(REFEREE_t *referee, unsigned char k);
/*机器人状态*/
static void ROBOT_STATUS(REFEREE_t *referee, unsigned char k);
/*功率热量*/
static void POWER_HEAT(REFEREE_t *referee, unsigned char k);
/*机器人位置数据*/
static void ROBOT_POSITION(REFEREE_t *referee, unsigned char k);
/*机器人增益*/
static void ROBOT_BUFF(REFEREE_t *referee, unsigned char k);
/*场地事件数据*/
static void EVENT_DATA(REFEREE_t *referee, unsigned char k);
/*无人机能量状态数据*/
static void UAV_ENERGY_TIME(REFEREE_t *referee, unsigned char k);
/*伤害状态数据*/
static void HURT_DATA(REFEREE_t *referee, unsigned char k);
/*实时射击数据*/
static void SHOOT_DATA(REFEREE_t *referee, unsigned char k);
/*剩余弹丸和金币数据*/
static void BULLET_DATA(REFEREE_t *referee, unsigned char k);
/*机器人RFID状态*/
static void RFID_STATUS(REFEREE_t *referee, unsigned char k);
/*飞镖机器人客户端指令*/
static void DART_CLIENT(REFEREE_t *referee, unsigned char k);
/*交互数据接收信息*/
static void INTERACT_HEADER(REFEREE_t *referee, unsigned char k);
/*客户端下发信息*/
static void CLIENT_DATA(REFEREE_t *referee, unsigned char k);
/*客户端接受信息*/
static void CLIENT_MAP_DATA(REFEREE_t *referee, unsigned char k);
/*哨兵接收地面机器人位置信息*/
static void GROUND_ROBOT_POSITION(REFEREE_t *referee, unsigned char k);
/*雷达接收标记进度信息*/
static void RADAR_MARK(REFEREE_t *referee, unsigned char k);
/*哨兵机器人兑换相关信息*/
static void SENTRY_INFO(REFEREE_t *referee, unsigned char k);
/*雷达双倍易伤相关数据*/
static void RADAR_INFO(REFEREE_t *referee, unsigned char k);
/*哨兵自主决策指令信息*/
static void SENTRY_CMD(REFEREE_t *referee, unsigned char k);
/*雷达自主决策指令信息*/
static void RADAR_CMD(REFEREE_t *referee, unsigned char k);
/*图传链路键鼠信息*/
static void PICTURE_TRANSMISSION(REFEREE_t *referee, unsigned char k);

USART_N::usart_c* referee_CC;
REFEREE_t *Get_referee_Address(void)
{
	return &referee;
}
void ECF_referee_uart_init()
{
    USART_N::usart_init_t usart_con =
    {
    .usart_handle_ = &huart6, //hal库usart句柄
    .rxbuf_size_ = 255,        //接收区缓冲大小
    .rx_type_ = USART_N::USART_RX_DMA_IDLE, //接收类型 USART_RX_DMA_IDLE
    .tx_type_ = USART_N::USART_TX_DMA,      //发送类型
    .usart_tx_callback_ptr_ = tx_call,
    .usart_rx_callback_ptr_ = rx_call,  //接收回调函数指针
    .rx_buff_ptr_ = rx_1                 //发送区指针
    };
		referee_CC = new USART_N::usart_c(usart_con);
    referee_CC->USART_rx_start();
}

/*裁判数据接收数据处理*/
void RefereeDataDeal(REFEREE_t *referee)
{
	uint8_t i;
	for (i = 0; i < referee->DataLen; i++)
	{
		if (referee->RefereeData[i] == 0xA5) //帧头
		{
			if (Verify_CRC8_Check_Sum(referee->RefereeData, HEADER_LEN) == 1) //CRC8校验
			{
				referee->RealLen = ((referee->RefereeData[i + 1]) | (referee->RefereeData[i + 2] << 8));					  //数据长度
				referee->Cmd_ID = ((referee->RefereeData[i + HEADER_LEN]) | (referee->RefereeData[i + HEADER_LEN + 1] << 8)); //命令码ID

				switch (referee->Cmd_ID)
				{
				case ID_STATE:
					GAME_STATUS(referee, i); 
					i = i + (DATA_STATUS_LEN + 9) + 9 - 1;
					break;
				
//				case ID_RESULT:
//					GAME_RESULT(referee, i);         
//					/*根据数据协议，下一次通讯帧头位于本次帧头 +5Byte（frame_header）+2Byte（cmd_id) +data长度 +2Byte(16位CRC)*/
//					/*即本次帧头 +9Byte*/
//					/* -1 抵消for执行完的自增*/
//					i = i + (DATA_RESULT_LEN + 9) - 1;
//					break;
//				
				case ID_ROBOT_HP:					
					ROBOT_HP(referee, i);       
					i = i + (DATA_ROBOT_HP_LEN + 9) - 1;				
					break;
				
//				case ID_EVENT_DATA:
//					EVENT_DATA(referee, i);             //无RFID，待测试
//					i = i + (DATA_EVENT_DATA_LEN + 9) - 1;
//					break;
				
//				case ID_SUPPLY_PROJECTILE_ACTION:
//					SUPPLY_PROJECTILE_ACTION(referee, i);    
//					i = i + (DATA_SUPPLY_PROJECTILE_ACTION_LEN + 9) - 1;
//					break;
//				case ID_REFEREE_WARNING:
//					REFEREE_WARNING(referee, i);       
//					i = i + (DATA_REFEREE_WARNING_LEN + 9) - 1;
//					break;
//				case ID_DART_REMAINING_TIME:
//					DART_REAMAINING_TIME(referee, i);       
//					i = i + (DATA_DART_REMAINING_TIME_LEN + 9) - 1;
//					break;
				case ID_ROBOT_STATE:
					ROBOT_STATUS(referee, i);            
					i = i + (DATA_ROBOT_STATUS_LEN + 9) - 1;
					break;
				case ID_POWER_HEAT_DATA:
					POWER_HEAT(referee, i);   
					i = i + (DATA_POWER_HEAT_DATA_LEN + 9) - 1;
					break;
//				case ID_ROBOT_POS:
//					ROBOT_POSITION(referee, i);   
//					i = i + (DATA_ROBOT_POS_LEN + 9) - 1;
//					break;
//				case ID_BUFF:
//					ROBOT_BUFF(referee, i);   //无RFID，待测试
//					i = i + (DATA_BUFF_LEN + 9) - 1;
//					break;
//				case ID_AERIAL_ROBOT_ENERGY:
//					UAV_ENERGY_TIME(referee, i);   //联盟赛无无人机
//					i = i + (DATA_AERIAL_ROBOT_ENERGY_LEN + 9) - 1;
//					break;
				case ID_ROBOT_HURT:
					HURT_DATA(referee, i);          
					i = i + (DATA_ROBOT_HURT_LEN + 9) - 1;
					break;
				case ID_SHOOT_DATA:
					SHOOT_DATA(referee, i);                   
					i = i + (DATA_SHOOT_DATA_LEN + 9) - 1;
					break;
				case ID_BULLET_REMAINING:
					BULLET_DATA(referee, i);                 
					i = i + (DATA_BULLET_REMAINING_LEN + 9) - 1;
					break;
//				case ID_RFID_STATUS:
//					RFID_STATUS(referee, i);           //无RFID，待测试
//					i = i + (DATA_RFID_STATUS_LEN + 9) - 1;
//					break;
//				case ID_DART_CLIENT_CMD:
//				 	DART_CLIENT(referee, i);  
//			  		i = i + (DATA_DART_CLIENT_CMD_LEN + 9) -1;
//					break;
//				case ID_GROUND_ROBOT_POSITION_CMD:
//					GROUND_ROBOT_POSITION(referee, i);//哨兵接收地面机器人位置信息
//					i = i + (DATA_GROUND_ROBOT_POSITION_CMD_LEN + 9) -1;
//					break; 
//				case ID_RADAR_MARK_CMD:	
//					RADAR_MARK(referee, i);//敌方机器人被标记进度
//					i = i + (DATA_RADAR_MARK_CMD_LEN + 9) -1;
//					break; 
//			 	case ID_SENTRY_INFO_CMD:
//					SENTRY_INFO(referee, i);//哨兵机器人兑换相关数据
//					i = i + (DATA_SENTRY_INFO_CMD_LEN + 9) -1;
//					break; 
//				case ID_RADAR_INFO_CMD:
//					RADAR_INFO(referee, i);//雷达双倍易伤相关数据
//					i = i + (DATA_RADAR_INFO_CMD_LEN + 9) -1;
//					break; 
				case ID_ROBOT_INTERACTION_DATA:
					/*自定义子内容，解析后续更新*/
					i = i + (DATA_ROBOT_INTERACTION_DATA_LEN + 9) -1;
					break; 
//				case ID_SENTRY_CMD:
//					SENTRY_CMD(referee, i);
//					i = i + (DATA_SENTRY_CMD_LEN + 9) -1;
//					break; 
//				case ID_RADAR_CMD:
//					RADAR_CMD(referee, i);
//					i = i + (DATA_RADAR_CMD_LEN + 9) -1;
//					break; 
					/*
				case ID_DIY_CONTROLLER:
					INTERACT_HEADER(referee, i);  
					i = i + (DATA_DIY_CONTROLLER + 9) -1;
					break;*/
				case ID_CLIENT_DOWMLOAD:
					CLIENT_DATA(referee, i);  
					i = i + (DATA_CLIENT_DOWMLOAD_LEN + 9) -1;
					break;
//				case ID_PICTURE_TRANSMISSION:
//					PICTURE_TRANSMISSION(referee, i);  
//					i = i + (DATA_PICTURE_TRANSMISSION_LEN + 9) -1;
//					break;
//				case ID_CLIENT_RECEIVE:
//					CLIENT_MAP_DATA(referee, i);
//					i = i + (DATA_CLIENT_RECEIVE_LEN  + 9) -1;
//					break;
				default:
					break;
				}
			}
		}
	}
}

/*比赛状态数据*/
static void GAME_STATUS(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_STATUS_LEN + 9); //数据转移

	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_STATUS_LEN + 9) == 1) //CRC16校验
	{
		memcpy(&referee->Game_Status, referee->RealData + 7, DATA_STATUS_LEN);
		referee->Game_Status.error = 0;
	}
	else
	{
		referee->Game_Status.error = 1;
	}
}

/*比赛结果数据*/
static void GAME_RESULT(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_RESULT_LEN + 9); //数据转移

	if (Verify_CRC16_Check_Sum(referee->RealData,  DATA_RESULT_LEN + 9) == 1) //CRC16校验
	{
		memcpy(&referee->Game_Result, referee->RealData + 7, DATA_RESULT_LEN);
		referee->Game_Result.error = 0;
	}
	else
	{
		referee->Game_Result.error = 1;
	}
}

/*机器人血量状态数据*/
static void ROBOT_HP(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_ROBOT_HP_LEN + 9); //数据转移

	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_ROBOT_HP_LEN + 9) == 1) //CRC16校验
	{
		memcpy(&referee->Robot_HP, referee->RealData + 7,DATA_ROBOT_HP_LEN);
		referee->Robot_HP.error = 0;
	}
	else
	{
		referee->Robot_HP.error = 1;
	}
}

/*场地事件数据*/
static void EVENT_DATA(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_EVENT_DATA_LEN + 9); //数据转移
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_EVENT_DATA_LEN + 9) == 1)
	{
		memcpy(&referee->Event_Data, referee->RealData + 7, DATA_EVENT_DATA_LEN);
		referee->Event_Data.error = 0;
	}
	else
	{
		referee->Event_Data.error = 1;
	}
}

/*补给站动作标识*/
static void SUPPLY_PROJECTILE_ACTION(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_SUPPLY_PROJECTILE_ACTION_LEN + 9); //数据转移
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_SUPPLY_PROJECTILE_ACTION_LEN + 9) == 1)
	{
		memcpy(&referee->Supply_Action, referee->RealData + 7, DATA_SUPPLY_PROJECTILE_ACTION_LEN);
		referee->Supply_Action.error = 0;
	}
	else
	{
		referee->Supply_Action.error = 1;
	}
}

/*裁判警告数据*/
static void REFEREE_WARNING(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_REFEREE_WARNING_LEN + 9); //数据转移
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_REFEREE_WARNING_LEN + 9) == 1)  //CRC16校验
	{
		memcpy(&referee->Referee_Warning, referee->RealData + 7, DATA_REFEREE_WARNING_LEN);
		referee->Referee_Warning.error = 0;
	}
	else
	{
		referee->Referee_Warning.error = 1;
	}
}

/*飞镖发射口倒计时*/
static void DART_REAMAINING_TIME(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_DART_REMAINING_TIME_LEN + 9); //数据转移
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_DART_REMAINING_TIME_LEN + 9) == 1)  //CRC16校验
	{
		memcpy(&referee->Dart_Remaining_Time, referee->RealData + 7, DATA_DART_REMAINING_TIME_LEN);
		referee->Dart_Remaining_Time.error = 0;
	}
	else
	{
		referee->Dart_Remaining_Time.error = 1;
	}
}

/*机器人状态数据*/
static void ROBOT_STATUS(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_ROBOT_STATUS_LEN + 9); //数据转移
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_ROBOT_STATUS_LEN + 9) == 1)  //CRC16校验
	{
		memcpy(&referee->Robot_Status, referee->RealData + 7, DATA_ROBOT_STATUS_LEN);
		referee->Robot_Status.error = 0;
	}
	else
	{
		referee->Robot_Status.error = 1;
	}
}

/*热量功率数据*/
static void POWER_HEAT(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_POWER_HEAT_DATA_LEN + 9); //数据转移
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_POWER_HEAT_DATA_LEN + 9) == 1)  //CRC16校验
	{
		memcpy(&referee->Power_Heat, referee->RealData + 7, DATA_POWER_HEAT_DATA_LEN);
		referee->Power_Heat.error = 0;
	}
	else
	{
		referee->Power_Heat.error = 1;
	}
}

/*机器人位置数据*/
static void ROBOT_POSITION(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_ROBOT_POS_LEN + 9); //数据转移
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_ROBOT_POS_LEN + 9) == 1)  //CRC16校验
	{
		memcpy(&referee->Robot_Position, referee->RealData + 7, DATA_ROBOT_POS_LEN);
		referee->Robot_Position.error = 0;
	}
	else
	{
		referee->Robot_Position.error = 1;
	}
}

/*机器人增益*/
static void ROBOT_BUFF(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_BUFF_LEN + 9); //数据转移
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_BUFF_LEN + 9) == 1)  //CRC16校验
	{
		memcpy(&referee->Buff, referee->RealData + 7, DATA_BUFF_LEN);
		referee->Buff.error = 0;
	}
	else
	{
		referee->Buff.error = 1;
	}
}

/*无人机能量状态数据*/
static void UAV_ENERGY_TIME(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_AERIAL_ROBOT_ENERGY_LEN + 9); //数据转移
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_AERIAL_ROBOT_ENERGY_LEN + 9) == 1)
	{
		memcpy(&referee->Aerial_Energy, referee->RealData + 7, DATA_AERIAL_ROBOT_ENERGY_LEN);
		referee->Aerial_Energy.error = 0;
	}
	else
	{
		referee->Aerial_Energy.error = 1;
	}
}

/*伤害状态数据*/
static void HURT_DATA(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_ROBOT_HURT_LEN + 9); //数据转移
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_ROBOT_HURT_LEN + 9) == 1)
	{
		memcpy(&referee->Robot_Hurt, referee->RealData + 7, DATA_ROBOT_HURT_LEN);
		referee->Robot_Hurt.error = 0;
	}
	else
	{
		referee->Robot_Hurt.error = 1;
	}
}

/*子弹剩余数据*/
static void BULLET_DATA(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_BULLET_REMAINING_LEN + 9); //数据转移
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_BULLET_REMAINING_LEN + 9) == 1)
	{
		memcpy(&referee->Bullet_Num, referee->RealData + 7, DATA_BULLET_REMAINING_LEN);
		referee->Bullet_Num.error = 0;
	}
	else
	{
		referee->Bullet_Num.error = 1;
	}
}

/*实时射击数据*/
static void SHOOT_DATA(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_SHOOT_DATA_LEN + 9); //数据转移
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_SHOOT_DATA_LEN + 9) == 1)
	{
		memcpy(&referee->Shoot_Data, referee->RealData + 7, DATA_SHOOT_DATA_LEN);
		referee->Shoot_Data.error = 0;
	}
	else
	{
		referee->Shoot_Data.error = 1;
	}
}

/*机器人RFID状态*/
static void RFID_STATUS(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_RFID_STATUS_LEN + 9); //数据转移
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_RFID_STATUS_LEN + 9) == 1)
	{
		memcpy(&referee->RFID_Status, referee->RealData + 7, DATA_RFID_STATUS_LEN);
		referee->RFID_Status.error = 0;
	}
	else
	{
		referee->RFID_Status.error = 1;
	}
}

/*飞镖机器人客户端指令*/
static void DART_CLIENT(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_DART_CLIENT_CMD_LEN +9); //数据转移
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_DART_CLIENT_CMD_LEN + 9) == 1)
	{
		memcpy(&referee->Dart_Client, referee->RealData + 7, DATA_DART_CLIENT_CMD_LEN);
		referee->Dart_Client.error = 0;
	}
	else
	{
		referee->Dart_Client.error = 1;
	}
}

/*哨兵接收地面机器人位置信息*/
static void GROUND_ROBOT_POSITION(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_GROUND_ROBOT_POSITION_CMD_LEN +9); //数据转移
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_GROUND_ROBOT_POSITION_CMD_LEN + 9) == 1)
	{
		memcpy(&referee->Ground_robot_position, referee->RealData + 7, DATA_GROUND_ROBOT_POSITION_CMD_LEN);
		referee->Ground_robot_position.error = 0;
	}
	else
	{
		referee->Ground_robot_position.error = 1;
	}
}
/*雷达接收标记进度信息*/
static void RADAR_MARK(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_RADAR_MARK_CMD_LEN +9); //数据转移
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_RADAR_MARK_CMD_LEN + 9) == 1)
	{
		memcpy(&referee->Radar_mark, referee->RealData + 7, DATA_RADAR_MARK_CMD_LEN);
		referee->Radar_mark.error = 0;
	}
	else
	{
		referee->Radar_mark.error = 1;
	}
}
/*哨兵机器人兑换相关信息*/
static void SENTRY_INFO(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_SENTRY_INFO_CMD_LEN +9); //数据转移
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_SENTRY_INFO_CMD_LEN + 9) == 1)
	{
		memcpy(&referee->Sentry_info, referee->RealData + 7, DATA_SENTRY_INFO_CMD_LEN);
		referee->Sentry_info.error = 0;
	}
	else
	{
		referee->Sentry_info.error = 1;
	}
}
/*雷达双倍易伤相关数据*/
static void RADAR_INFO(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_RADAR_INFO_CMD_LEN + 9); //数据转移
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_RADAR_INFO_CMD_LEN + 9) == 1)
	{
		memcpy(&referee->Radar_info, referee->RealData + 7, DATA_RADAR_INFO_CMD_LEN);
		referee->Radar_info.error = 0;
	}
	else
	{
		referee->Radar_info.error = 1;
	}
}
/*哨兵自主决策指令信息*/
static void SENTRY_CMD(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_SENTRY_CMD_LEN + 9); //数据转移
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_SENTRY_CMD_LEN + 9) == 1)
	{
		memcpy(&referee->Sentry_Cmd, referee->RealData + 7, DATA_SENTRY_CMD_LEN);
		referee->Sentry_Cmd.error = 0;
	}
	else
	{
		referee->Sentry_Cmd.error = 1;
	}
}
/*雷达自主决策指令信息*/
static void RADAR_CMD(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_RADAR_CMD_LEN + 9); //数据转移
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_RADAR_CMD_LEN + 9) == 1)
	{
		memcpy(&referee->Radar_Cmd, referee->RealData + 7, DATA_RADAR_CMD_LEN);
		referee->Radar_Cmd.error = 0;
	}
	else
	{
		referee->Radar_Cmd.error = 1;
	}
}
/*图传链路键鼠信息*/
static void PICTURE_TRANSMISSION(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_PICTURE_TRANSMISSION_LEN + 9); //数据转移
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_PICTURE_TRANSMISSION_LEN + 9) == 1)
	{
		memcpy(&referee->Remote_control, referee->RealData + 7, DATA_PICTURE_TRANSMISSION_LEN);
		referee->Remote_control.error = 0;
	}
	else
	{
		referee->Remote_control.error = 1;
	}
}
/*交互数据接收信息*/
static void INTERACT_HEADER(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_DIY_CONTROLLER + 9); //数据转移
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_DIY_CONTROLLER + 9) == 1)
	{
		memcpy(&referee->Interact_Header, referee->RealData + 7, DATA_DIY_CONTROLLER);
		referee->Interact_Header.error = 0;
	}
	else
	{
		referee->Interact_Header.error = 1;
	}
}

/*客户端下发信息*/
static void CLIENT_DATA(REFEREE_t *referee, unsigned char k)
{
		memcpy(referee->RealData, referee->RefereeData + k, DATA_CLIENT_DOWMLOAD_LEN + 9); //数据转移
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_CLIENT_DOWMLOAD_LEN + 9) == 1)
	{
		memcpy(&referee->Client_Data, referee->RealData + 7, DATA_CLIENT_DOWMLOAD_LEN);
		referee->Client_Data.error = 0;
	}
	else
	{
		referee->Client_Data.error = 1;
	}
}

/*客户端接受信息*/
static void CLIENT_MAP_DATA(REFEREE_t *referee, unsigned char k)
{
	memcpy(referee->RealData, referee->RefereeData + k, DATA_CLIENT_RECEIVE_LEN + 9); //数据转移
	if (Verify_CRC16_Check_Sum(referee->RealData, DATA_CLIENT_RECEIVE_LEN + 9) == 1)
	{
		memcpy(&referee->ClientMapData, referee->RealData + 7, DATA_CLIENT_RECEIVE_LEN);
		referee->ClientMapData.error = 0;
	}
	else
	{
		referee->ClientMapData.error = 1;
	}
}