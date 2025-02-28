//视觉任务，现在无需电控下位机进行弹道解算，保证通讯稳定不单开任务
#include "Vision.h"
#include "fifo.h"
    
fifo_s_t* usb_info = NULL;
Visual_Rx_t Visual_Rx;
Visual_Tx_t Visual_Tx;
float Send_Pitch_Compeny = 0.0f;  //pitch补偿

void Virtual_Init()
{
    memset(&Visual_Rx, 0, sizeof(Visual_Rx_t));
    memset(&Visual_Tx, 0, sizeof(Visual_Tx_t));
    Visual_Rx.distance = -1.0f;
    usb_info = fifo_s_create(96);
}

void Virtual_recive(void)
{
	if(fifo_s_isempty(usb_info) != 1)
	{
		static uint8_t read_buff[VIRTUAL_DATA_LEN];
		fifo_s_gets(usb_info, read_buff, VIRTUAL_DATA_LEN);
		if(read_buff[0] == 0xFF && read_buff[15] == 0xFE)
        {
            memcpy(&Visual_Rx.fire_flag,&read_buff[1],1);//允许开火标志位
			memcpy(&Visual_Rx.pitch,&read_buff[2],4);
			memcpy(&Visual_Rx.yaw,&read_buff[6],4);
			memcpy(&Visual_Rx.distance,&read_buff[10],4);
			memcpy(&Visual_Rx.armor_nums,&read_buff[14],1);
        }
	}
}

uint64_t send_time = 0;

void Virtual_send(float Pitch, float Yaw, float Roll, float _bullet_speed, uint8_t _reset_tracker, uint8_t _now_mode)
{
    send_time++;

    static uint8_t send_buff[33];
    Visual_Tx.header = 0xFF;
    Visual_Tx.trailer = 0xFE;
    memcpy(&Visual_Tx.aim_x, 0, 12);
    Visual_Tx.Roll = Roll;
    Visual_Tx.Pitch = Pitch + Send_Pitch_Compeny;
    Visual_Tx.Yaw = Yaw;
    Visual_Tx.detect_color = 1;
    Visual_Tx.reset_tracker = _reset_tracker;
    Visual_Tx.now_mode = _now_mode;
    Visual_Tx.bullet_speed = _bullet_speed;
    memcpy(&send_buff, &Visual_Tx.header, 33);
    CDC_Transmit_FS(send_buff, sizeof(send_buff));
}

Visual_Rx_t* Get_virtual_recive_ptr()
{
    return &Visual_Rx;
}


