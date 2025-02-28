#ifndef __DR16_HPP
#define __DR16_HPP


#ifdef STM32H723xx
#include "bsp_usart_h7.hpp"
#endif

#ifdef STM32F405xx
#include "bsp_usart_f4.hpp"
#endif


#ifdef __cplusplus
extern "C" {
#endif

#include "dr16.h"

#ifdef __cplusplus
}
#endif

/* 遥控文档内容-BEGIN */
/* ----------------------- RC Channel Definition---------------------------- */
#define RC_CH_VALUE_MIN    ((uint16_t)364)  // 通道最小值
#define RC_CH_VALUE_OFFSET ((uint16_t)1024) // 通道中间值
#define RC_CH_VALUE_MAX    ((uint16_t)1684) // 通道最大值
/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_ERROR ((uint16_t)0) // 出现严重错误
#define RC_SW_UP    ((uint16_t)1)
#define RC_SW_MID   ((uint16_t)3)
#define RC_SW_DOWN  ((uint16_t)2)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W     ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S     ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A     ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D     ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL  ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q     ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E     ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R     ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F     ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G     ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z     ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X     ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C     ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V     ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B     ((uint16_t)1 << 15)

// 参数
#define DR16_RX_BUFFER_NUM     18
#define RC_FRAME_LENGTH        18
#define RC_CHANNAL_ERROR_VALUE 700 // 遥控出错上限
#define DEADLINE               100
#define LOSTTIMEMAX            1000

namespace DR16_n
{
    typedef enum {
        LOST,
        NORMAL
    } DT7_State_e;

    typedef struct {
        struct {
            int16_t ch[5];
            uint8_t s1;
            uint8_t s2;
        } rc;
        struct {
            int16_t x = 0;
            int16_t y = 0;
            int16_t z = 0;
            uint8_t press_l = 0;
            uint8_t press_r = 0;
        } mouse;
        /* keyboard key information */
        union {
            uint16_t key_code;
            struct
            {
                uint16_t W : 1;
                uint16_t S : 1;
                uint16_t A : 1;
                uint16_t D : 1;
                uint16_t SHIFT : 1;
                uint16_t CTRL : 1;
                uint16_t Q : 1;
                uint16_t E : 1;
                uint16_t R : 1;
                uint16_t F : 1;
                uint16_t G : 1;
                uint16_t M : 1; // uint16_t Z : 1;
                uint16_t N : 1; // uint16_t X : 1;
                uint16_t C : 1;
                uint16_t V : 1;
                uint16_t B : 1;
            } bit;
        } kb;
    } RC_ctrl_t;

    class DR16_c
    {
       private:
        void LostTimeSetZero(); 
        void rc_deadline_limit(int16_t *input);
        
       protected:
       
        int16_t dead_line_;
        uint32_t lost_time_;
        USART_N::usart_init_t *UARTParam;
       

       public: 
        DT7_State_e dt7_state_ = NORMAL;
        uint8_t double_buffer_[2][DR16_RX_BUFFER_NUM]; // 双缓冲区
        USART_N::usart_c *dr16_rece_;
        static DR16_c *dr16_;
        RC_ctrl_t RC_HandleData;
        DR16_c(int16_t dead_line_);
        void RC_Data_init_Clear();

        DR16_c *GetClassPtr();
        RC_ctrl_t *GetDataStructPtr();
         
        DT7_State_e GetDR16State();
        uint32_t *GetLostTimePtr();
        void DataHandle(volatile const uint8_t *pData);
    };
    void LostHandle();
    void callback(uint8_t *pdata, uint16_t psize);
}

#endif /*__DR16_HPP*/