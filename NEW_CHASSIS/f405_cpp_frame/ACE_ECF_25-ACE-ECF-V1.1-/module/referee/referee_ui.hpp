#ifndef _REFEREE_UI_HPP
#define _REFEREE_UI_HPP

#ifdef STM32H723xx
#include "bsp_usart_h7.hpp"
#endif

#ifdef STM32F405xx
#include "bsp_usart_f4.hpp"
#endif

#ifdef __cplusplus
extern "C"
{
#endif

#define UI_RX_MAX_NUM 18
#define UI_RX_DATA_LENGTH 18
#include "main.h"
#ifdef __cplusplus
}
#endif
/*****************************************UI****************************************************/
// 要进行修改请参照裁判系统通信协议
#pragma pack(1) // 按1字节对齐
//以下为绘制ui的通信格式
/************************帧头+CMDid+数据头+数据+帧尾校验**************************/
// 帧头：开始标准+数据长度+序号+crc8校验
// CMDid：看手册-->UI_CMD_Robo_Exchange
// 数据头：内容id+接收者id+发送者id
// 数据： 删除/ 图形 / 字符
// 校验
/****************************开始标志*********************/
#define UI_SOF 0xA5
/****************************UI数据长度********************/
#define CRC8_LEN 4u // 帧头CRC8校验长度=帧头+数据长+包序号

#define FRAMEHEAD_LEN 5u       // 帧头总长度
#define CMD_ID_LEN 2u          // CMDid长度
#define UI_DATA_HEAD_LEN 6u    // 数据包头及id长度
#define UI_Operate_Deal_LEN 2u // 删除操作数据长度
#define UI_GRAPH_DATA_LEN 15u  // 绘制图形操作数据长度
#define TAIL_CRC16_LEN 2u      // 帧尾crc校验长度
#define UI_DELETE_LEN 17u      // 删除操作 帧总长度:帧头+CMDid+数据头+数据+帧尾校验；若有改动需重新计算
#define UI_STRING_DATA_LEN 45u // 字符串数据长度
// #define ALL_GRAPH_LEN 30u  //没有用到
#define ALL_STRING_LEN 60u // 45+15 生成字符串操作总长度
/****************************CMD_ID数据********************/
#define UI_CMD_Robo_Exchange 0x0301 // 机器人交互数据 命令id--通过此命令绘制ui
/*****************************帧头*****************************/
typedef struct
{
    uint8_t SOF;          // 起始字节,固定0xA5
    uint16_t Data_Length; // 帧数据长度
    uint8_t Seq;          // 包序号
    uint8_t CRC8;         // CRC8校验值
} UI_Packhead_t;          // 帧头
// 该结构体需要后接数据/操作结构体
/*****************************数据头部id****************************/
typedef struct
{
    uint16_t Data_ID;     // 内容ID 根据内容id不同完成不同功能
    uint16_t Sender_ID;   // 发送者ID
    uint16_t Receiver_ID; // 接收者ID
} UI_Data_Operate_t;      // 操作定义帧：和0x301同为一帧，只是格式内容和id不同

/****************************内容ID数据********************/
/* 交互数据ID */
typedef enum
{
    UI_Data_ID_Del = 0x100,   // 删除操作
    UI_Data_ID_Draw1 = 0x101, // 绘制1个图形
    UI_Data_ID_Draw2 = 0x102, // 绘制2个图形
    UI_Data_ID_Draw5 = 0x103,
    UI_Data_ID_Draw7 = 0x104,
    UI_Data_ID_DrawChar = 0x110, // 绘制一个字符

    /* 自定义交互数据部分 */
    Communicate_Data_ID = 0x0200,

} Interactive_Data_ID_e;
/****************************红方机器人ID********************/
/* 机器人id / 发送者id */
typedef enum
{
    // 红方机器人ID
    RobotID_RHero = 1,
    RobotID_REngineer = 2,
    RobotID_RStandard1 = 3,
    RobotID_RStandard2 = 4,
    RobotID_RStandard3 = 5,
    RobotID_RAerial = 6,
    RobotID_RSentry = 7,
    RobotID_RDart = 8,
    RobotID_RRadar = 9,
    RobotID_ROutpost = 10,
    RobotID_RBaseland = 11,
    // 蓝方机器人ID
    RobotID_BHero = 101,
    RobotID_BEngineer = 102,
    RobotID_BStandard1 = 103,
    RobotID_BStandard2 = 104,
    RobotID_BStandard3 = 105,
    RobotID_BAerial = 106,
    RobotID_BSentry = 107,
    RobotID_BDart = 108,
    RobotID_BRadar = 109,
    RobotID_BOutpost = 110,
    RobotID_BBaseland = 111,
} Robot_ID_e;
/**************************红方操作手ID / 接收者id************************/
#define UI_Data_CilentID_RHero 0x0101
#define UI_Data_CilentID_REngineer 0x0102
#define UI_Data_CilentID_RStandard1 0x0103
#define UI_Data_CilentID_RStandard2 0x0104
#define UI_Data_CilentID_RStandard3 0x0105
#define UI_Data_CilentID_RAerial 0x0106 // 红方无人机
/***************************蓝方操作手ID / 接收者id***********************/
#define UI_Data_CilentID_BHero 0x0165
#define UI_Data_CilentID_BEngineer 0x0166
#define UI_Data_CilentID_BStandard1 0x0167
#define UI_Data_CilentID_BStandard2 0x0168
#define UI_Data_CilentID_BStandard3 0x0169
#define UI_Data_CilentID_BAerial 0x016A // 蓝方无人机


/***************************删除操作***************************/
/* 删除操作 */
typedef enum
{
    UI_Data_Del_NoOperate = 0, // 空操作
    UI_Data_Del_Layer = 1,     // 删除图层
    UI_Data_Del_ALL = 2,       // 删除全部图层
} UI_Delete_Operate_e;
/***************************图形配置参数__图形操作********************/
/* 图形配置参数__图形操作 */
typedef enum
{
    UI_Graph_ADD = 1,
    UI_Graph_Change = 2,
    UI_Graph_Del = 3,
} UI_Graph_Operate_e;
/***************************图形配置参数__图形类型********************/
/* 图形配置参数__图形类型 */
typedef enum
{
    UI_Graph_Line = 0,      // 直线
    UI_Graph_Rectangle = 1, // 矩形
    UI_Graph_Circle = 2,    // 整圆
    UI_Graph_Ellipse = 3,   // 椭圆
    UI_Graph_Arc = 4,       // 圆弧
    UI_Graph_Float = 5,     // 浮点型
    UI_Graph_Int = 6,       // 整形
    UI_Graph_Char = 7,      // 字符型

} UI_Graph_Type_e;
/***************************图形配置参数__图形颜色********************/
/* 图形配置参数__图形颜色 */
typedef enum
{
    UI_Color_Main = 0, // 红蓝主色
    UI_Color_Yellow = 1,
    UI_Color_Green = 2,
    UI_Color_Orange = 3,
    UI_Color_Purplish_red = 4, // 紫红色
    UI_Color_Pink = 5,
    UI_Color_Cyan = 6, // 青色
    UI_Color_Black = 7,
    UI_Color_White = 8,

} UI_Graph_Color_e;


/**子id：0x0100  功能:选手端删除图层  lens：2*/
// 只需将其接到UI_Data_Operate（操作定义帧）后即可
typedef struct
{
    uint8_t Delete_Operate; // 删除操作
    uint8_t Layer;          // 删除图层
} UI_Data_Delete_t;         // 删除图层帧

/**子id：0x0101  功能:选手端发送浮点数  lens：15*/
// 具体看裁判系统通信协议-图形细节参数说明
typedef struct
{
    uint8_t graphic_name[3];
    uint32_t operate_tpye : 3;
    uint32_t graphic_tpye : 3;
    uint32_t layer : 4;
    uint32_t color : 4;
    uint32_t start_angle : 9;
    uint32_t end_angle : 9;
    uint32_t width : 10;
    uint32_t start_x : 11;
    uint32_t start_y : 11;
    int graph_Float; // 浮点数据
} Float_Data_t;

/**子id：0x0101  功能:选手端绘制一个图形  lens：15*/
// 具体看裁判系统通信协议-图形细节参数说明
typedef struct
{
    uint8_t graphic_name[3];   // 图形名，作为索引
    uint32_t operate_tpye : 3; // 图形配置1：图形操作
    uint32_t graphic_tpye : 3; // 图形配置1：图形类型
    uint32_t layer : 4;        // 图形配置1：图形层数
    uint32_t color : 4;        // 图形配置1：图形颜色
    uint32_t start_angle : 9;  // 图形配置1：根据图形不同而不同，这里设置为起始角度 /字体大小
    uint32_t end_angle : 9;    // 图形配置1：根据图形不同而不同，这里设置为起始角度 /字体大小
    uint32_t width : 10;       // 图形配置2：线宽，建议字体大小：线宽==10：1
    uint32_t start_x : 11;     // 图形配置2：起点/圆心x坐标
    uint32_t start_y : 11;     // 图形配置2：起点/圆心y坐标
    uint32_t radius : 10;      // 图形配置3：根据图形不同而不同，这里设置为半径
    uint32_t end_x : 11;       // 图形配置3：根据图形不同而不同，这里设置为终点 x 坐标
    uint32_t end_y : 11;       // 图形配置3：根据图形不同而不同，这里设置为终点 y 坐标
    //  uint32_t details_c : 10;
    // uint32_t details_d : 11;
    // uint32_t details_e : 11; // 图形数据
} Graph_Data_t; // 图像数据
// 一般一次（最多）绘制7个图形

/**子id：0x0110  功能:选手端绘制字符  lens：45*/
// 只需将其接到UI_Data_Operate（操作定义帧）后即可
typedef struct
{
    Graph_Data_t Graph_Control;
    char show_Data[30]; // 字符
} String_Data_t;        // 打印字符串数据

/*****将需要发送的帧节进行拼接成一个结构体，可关注以下三个即可*******/
// 将三个结构体拼接而成的完整操作结构体
typedef struct
{
    UI_Packhead_t FrameHeader;      // 帧头
    uint16_t CMD_ID;                // 命令ID
    UI_Data_Operate_t datahead;     // 具体类型及id
    UI_Data_Delete_t Delete_detail; // 集体操作
    uint16_t frametail;             // 帧尾校验
} UI_delete_t;

typedef struct
{
    UI_Packhead_t FrameHeader;  // 帧头
    uint16_t CmdID;             // 命令ID
    UI_Data_Operate_t datahead; // 具体类型及id
    uint16_t frametail;         // 帧尾校验
} UI_GraphReFresh_t;
typedef struct
{
    UI_Packhead_t FrameHeader;  // 帧头
    uint16_t CmdID;             // 命令ID
    UI_Data_Operate_t datahead; // 具体类型及id
    String_Data_t String_Data;
    uint16_t frametail; // 帧尾校验
} UI_CharReFresh_t;     // 打印字符串数据
namespace referee_ui_n
{
    class referee_ui_c
    {

    public:
        referee_ui_c(/* args */);
        ~referee_ui_c();
        referee_ui_c(uint16_t Robot_ID, uint16_t Cilent_ID);
        uint16_t UI_Robot_ID;  // 需要设置的robotid
        uint16_t UI_Cilent_ID; // 需要设置的操作手id
        void UI_Send(uint8_t *send_data, uint16_t data_lens);
        void UI_Delete(UI_Delete_Operate_e Del_Operate, uint8_t Del_Layer);
        void UI_Line_Draw(Graph_Data_t *image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
                          uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y);
        void Rectangle_Draw(Graph_Data_t *image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer,
                            uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y);
        void Circle_Draw(Graph_Data_t *image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width,
                         uint32_t Start_x, uint32_t Start_y, uint32_t Graph_Radius);
        void Arc_Draw(Graph_Data_t *image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_StartAngle,
                      uint32_t Graph_EndAngle, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t x_Length, uint32_t y_Length);
        void UIFloatDraw(Float_Data_t *image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Size,
                         uint32_t Graph_Digit, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, float Graph_Float);
        void Char_Draw(String_Data_t *image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Size,
                       uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, const char *Char_Data);
        void UI_Graph_Refresh(Graph_Data_t *Graph, int cnt...);
        void UI_Char_Refresh(String_Data_t string);
        USART_N::usart_c *UI_uart;

    protected:
    private:
        /* data */
    };

}

#endif /*_REFEREE_UI_HPP*/
