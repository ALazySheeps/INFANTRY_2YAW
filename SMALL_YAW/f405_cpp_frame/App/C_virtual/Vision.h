#ifndef VISION_H
#define VISION_H

#include "visual_config.h"
#include "fifo.h"

#define VIRTUAL_DATA_LEN 16

void Virtual_Init(void);
void Virtual_recive(void);
void Virtual_send(float Pitch, float Yaw, float Roll, float _bullet_speed, uint8_t _reset_tracker, uint8_t _now_mode);
Visual_Rx_t* Get_virtual_recive_ptr();

#endif