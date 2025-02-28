
/*************************** Dongguan-University of Technology -ACE**************************
 * @file    referee_ui.cpp
 * @author  laniceee
 * @version V1.0
 * @date    2024/10/25
 * @brief
 *   向裁判系统发送设定ui
 * @todo 考虑优化结构体，放到构造函数，减少重复赋值
 *       和裁判系统合并
 *       测试可行性
 ********************************************************************************************
 * @verbatim
 * 1--初始化时，将robot_id 和cilnent_id 传入类中
 * 2-- 使用函数进行绘制
 *
 ************************** Dongguan-University of Technology -ACE***************************/

#include "referee_ui.hpp"
#include "crc_ref.h"
#include "stdarg.h"
#include "string.h"

// 走图传链路：自定义控制器/机器人交互数据（包括ui）/键鼠遥控数据
// 走常规链路：

namespace referee_ui_n
{
    unsigned char UI_Seq = 0; // 包序号

    // 目前只需发送无需接收
    // uint8_t ui_double_buffer_[2][UI_RX_MAX_NUM * 2]; // 双缓冲区
    USART_N::usart_init_t UI_uart_param =
        {
            //.usart_handle_ = &huart2,
            // .rxbuf_size_ = UI_RX_MAX_NUM * 2,         // 接收区大小
            .rx_type_ = USART_N::USART_RX_DMA_IDLE_D, // 接收类型
            .tx_type_ = USART_N::USART_TX_DMA,        // 发送类型
                                                      //.usart_rx_callback_ptr_ = &callback,
            .usart_data_length_ = UI_RX_DATA_LENGTH,
            //  .rx_buff_ptr_ = ui_double_buffer_[0],
            //.secondebuf_ptr_ = ui_double_buffer_[1],
    };

    referee_ui_c::referee_ui_c(uint16_t Robot_ID, uint16_t Cilent_ID) : UI_Robot_ID(Robot_ID), UI_Cilent_ID(Cilent_ID)
    {

        UI_uart = new USART_N::usart_c(UI_uart_param);
    }
    referee_ui_c::referee_ui_c(/* args */)
    {
        UI_uart = new USART_N::usart_c(UI_uart_param);
    }
    referee_ui_c::~referee_ui_c()
    {
        delete UI_uart;
    }

    // 发送数据封装，其实可以不封装
    void referee_ui_c::UI_Send(uint8_t *send_data, uint16_t data_lens)
    {
        UI_uart->USART_send(send_data, data_lens, 0);
    }

    /************************************************删除图层*************************************************
    **参数: uint8_t Del_Operate  删除的操作，具体看枚举
            uint8_t Del_Layer : 删除的图层--0~9
    **********************************************************************************************************/
    void referee_ui_c::UI_Delete(UI_Delete_Operate_e Del_Operate, uint8_t Del_Layer)
    {

        // 以下结构体构成发送包
        static UI_delete_t UI_delete_data;

        UI_delete_data.FrameHeader.SOF = UI_SOF;                                         // 填充包头数据
        UI_delete_data.FrameHeader.Data_Length = UI_DATA_HEAD_LEN + UI_Operate_Deal_LEN; // 数据包长度
        UI_delete_data.FrameHeader.Seq = UI_Seq;
        UI_delete_data.FrameHeader.CRC8 = Get_CRC8_Check_Sum((uint8_t *)&UI_delete_data, CRC8_LEN, 0xFF);

        UI_delete_data.CMD_ID = UI_CMD_Robo_Exchange;

        UI_delete_data.datahead.Data_ID = UI_Data_ID_Del;
        UI_delete_data.datahead.Receiver_ID = UI_Cilent_ID; // 填充操作数据
        UI_delete_data.datahead.Sender_ID = UI_Robot_ID;

        UI_delete_data.Delete_detail.Delete_Operate = Del_Operate;
        UI_delete_data.Delete_detail.Layer = Del_Layer;
        UI_delete_data.frametail = Get_CRC16_Check_Sum((uint8_t *)&UI_delete_data, UI_DELETE_LEN - TAIL_CRC16_LEN, 0xFFFF);

        UI_Send((uint8_t *)&UI_delete_data, UI_DELETE_LEN);

        UI_Seq++; // 包序号+1
    }
    /************************************************绘制直线*************************************************
    **参数：*image Graph_Data类型变量指针，用于存放图形数据
            imagename[3]   图片名称，用于标识更改
            Graph_Operate  图片操作，见头文件
            Graph_Layer    图层0-9
            Graph_Color    图形颜色
            Graph_Width    图形线宽
            Start_x、Start_x    开始坐标
            End_x、End_y   结束坐标
    **********************************************************************************************************/
    void referee_ui_c::UI_Line_Draw(
        Graph_Data_t *image,
        char imagename[3],
        uint32_t Graph_Operate,
        uint32_t Graph_Layer,
        uint32_t Graph_Color,
        uint32_t Graph_Width,
        uint32_t Start_x,
        uint32_t Start_y,
        uint32_t End_x,
        uint32_t End_y)
    {
        int i;

        for (i = 0; i < 3 && imagename[i] != '\0'; i++) // 填充至‘0’为止
            image->graphic_name[2 - i] = imagename[i];  // 按内存地址增大方向填充，所以会有i与2-i

        image->operate_tpye = Graph_Operate;
        image->graphic_tpye = UI_Graph_Line;
        image->layer = Graph_Layer;
        image->color = Graph_Color;

        image->start_angle = 0;
        image->end_angle = 0;
        image->width = Graph_Width;
        image->start_x = Start_x;
        image->start_y = Start_y;
        image->radius = 0;
        image->end_x = End_x;
        image->end_y = End_y;
    }

    /************************************************绘制矩形*************************************************
    **参数：*image Graph_Data类型变量指针，用于存放图形数据
            imagename[3]   图片名称，用于标识更改
            Graph_Operate   图片操作，见头文件
            Graph_Layer    图层0-9
            Graph_Color    图形颜色
            Graph_Width    图形线宽
            Start_x、Start_x    开始坐标
            End_x、End_y   结束坐标（对顶角坐标）
    **********************************************************************************************************/
    void referee_ui_c::Rectangle_Draw(
        Graph_Data_t *image,
        char imagename[3],
        uint32_t Graph_Operate,
        uint32_t Graph_Layer,
        uint32_t Graph_Color,
        uint32_t Graph_Width,
        uint32_t Start_x,
        uint32_t Start_y,
        uint32_t End_x,
        uint32_t End_y)
    {
        int i;

        for (i = 0; i < 3 && imagename[i] != '\0'; i++)
            image->graphic_name[2 - i] = imagename[i];

        image->graphic_tpye = UI_Graph_Rectangle;
        image->operate_tpye = Graph_Operate;
        image->layer = Graph_Layer;
        image->color = Graph_Color;
        image->start_angle = 0;
        image->end_angle = 0;
        image->width = Graph_Width;
        image->start_x = Start_x;
        image->start_y = Start_y;
        image->radius = 0;
        image->end_x = End_x;
        image->end_y = End_y;
    }

    /************************************************绘制整圆*************************************************
    **参数：*image Graph_Data类型变量指针，用于存放图形数据
            imagename[3]   图片名称，用于标识更改
            Graph_Operate   图片操作，见头文件
            Graph_Layer    图层0-9
            Graph_Color    图形颜色
            Graph_Width    图形线宽
            Start_x、Start_x    圆心坐标
            Graph_Radius  图形半径
    **********************************************************************************************************/
    void referee_ui_c::Circle_Draw(
        Graph_Data_t *image,
        char imagename[3],
        uint32_t Graph_Operate,
        uint32_t Graph_Layer,
        uint32_t Graph_Color,
        uint32_t Graph_Width,
        uint32_t Start_x,
        uint32_t Start_y,
        uint32_t Graph_Radius)
    {
        int i;

        for (i = 0; i < 3 && imagename[i] != '\0'; i++)
            image->graphic_name[2 - i] = imagename[i];

        image->graphic_tpye = UI_Graph_Circle;
        image->operate_tpye = Graph_Operate;
        image->layer = Graph_Layer;
        image->color = Graph_Color;
        image->start_angle = 0;
        image->end_angle = 0;
        image->width = Graph_Width;
        image->start_x = Start_x;
        image->start_y = Start_y;
        image->radius = Graph_Radius;
        image->end_x = 0;
        image->end_y = 0;
    }

    /************************************************绘制圆弧*************************************************
    **参数：*image Graph_Data类型变量指针，用于存放图形数据
            imagename[3]   图片名称，用于标识更改
            Graph_Operate   图片操作，见头文件
            Graph_Layer    图层0-9
            Graph_Color    图形颜色
            Graph_Width    图形线宽
            Graph_StartAngle,Graph_EndAngle    开始，终止角度
            Start_y,Start_y    圆心坐标
            x_Length,y_Length   x,y方向上轴长，参考椭圆
    **********************************************************************************************************/
    void referee_ui_c::Arc_Draw(
        Graph_Data_t *image,
        char imagename[3],
        uint32_t Graph_Operate,
        uint32_t Graph_Layer,
        uint32_t Graph_Color,
        uint32_t Graph_StartAngle,
        uint32_t Graph_EndAngle,
        uint32_t Graph_Width,
        uint32_t Start_x,
        uint32_t Start_y,
        uint32_t x_Length,
        uint32_t y_Length)
    {
        int i;

        for (i = 0; i < 3 && imagename[i] != '\0'; i++)
            image->graphic_name[2 - i] = imagename[i];

        image->graphic_tpye = UI_Graph_Arc;
        image->operate_tpye = Graph_Operate;
        image->layer = Graph_Layer;
        image->color = Graph_Color;
        image->width = Graph_Width;
        image->start_x = Start_x;
        image->start_y = Start_y;
        image->start_angle = Graph_StartAngle;
        image->end_angle = Graph_EndAngle;
        image->end_x = x_Length;
        image->end_y = y_Length;
    }

    /************************************************绘制浮点型数据*************************************************
    **参数：*image Graph_Data类型变量指针，用于存放图形数据
            imagename[3]   图片名称，用于标识更改
            Graph_Operate   图片操作，见头文件
            Graph_Layer    图层0-9
            Graph_Color    图形颜色
            Graph_Width    图形线宽
            Graph_Size     字号
            Graph_Digit    小数位数
            Start_x、Start_x    开始坐标
            Graph_Float   要显示的变量
    **********************************************************************************************************/
    void referee_ui_c::UIFloatDraw(
        Float_Data_t *image,
        char imagename[3],
        uint32_t Graph_Operate,
        uint32_t Graph_Layer,
        uint32_t Graph_Color,
        uint32_t Graph_Size,
        uint32_t Graph_Digit,
        uint32_t Graph_Width,
        uint32_t Start_x,
        uint32_t Start_y,
        float Graph_Float)
    {
        int i;

        for (i = 0; i < 3 && imagename[i] != '\0'; i++)
            image->graphic_name[2 - i] = imagename[i];

        image->graphic_tpye = UI_Graph_Float;
        image->operate_tpye = Graph_Operate;
        image->layer = Graph_Layer;
        image->color = Graph_Color;
        image->width = Graph_Width;
        image->start_x = Start_x;
        image->start_y = Start_y;
        image->start_angle = Graph_Size;
        image->end_angle = Graph_Digit;
        image->graph_Float = Graph_Float;
    }

    /************************************************绘制字符型数据*************************************************
    **参数：*image Graph_Data类型变量指针，用于存放图形数据
            imagename[3]   图片名称，用于标识更改
            Graph_Operate   图片操作，见头文件
            Graph_Layer    图层0-9
            Graph_Color    图形颜色
            Graph_Width    图形线宽
            Graph_Size     字号
            Start_x、Start_y    开始坐标
            *Char_Data          待发送字符串开始地址
    **********************************************************************************************************/
    void referee_ui_c::Char_Draw(
        String_Data_t *image,
        char imagename[3],
        uint32_t Graph_Operate,
        uint32_t Graph_Layer,
        uint32_t Graph_Color,
        uint32_t Graph_Size,
        uint32_t Graph_Width,
        uint32_t Start_x,
        uint32_t Start_y,
        const char *Char_Data)
    {
        int i;

        for (i = 0; i < 3 && imagename[i] != '\0'; i++)
            image->Graph_Control.graphic_name[2 - i] = imagename[i];

        image->Graph_Control.graphic_tpye = UI_Graph_Char;
        image->Graph_Control.operate_tpye = Graph_Operate;
        image->Graph_Control.layer = Graph_Layer;
        image->Graph_Control.color = Graph_Color;
        image->Graph_Control.width = Graph_Width;
        image->Graph_Control.start_x = Start_x;
        image->Graph_Control.start_y = Start_y;
        image->Graph_Control.start_angle = Graph_Size;
        image->Graph_Control.end_angle = strlen(Char_Data);

        strcpy(image->show_Data, Char_Data);
    }

    /************************************************更新图像*************************************************
    **参数:  Graph_Data_t *Graph 图像指针
             int cnt  图像个数
             ...  接下来传多个图像的指针
    **********************************************************************************************************/
    void referee_ui_c::UI_Graph_Refresh(Graph_Data_t *Graph, int cnt...) // 由于不知道参数个数，所以采用vs_list 接收
    {

        UI_GraphReFresh_t UI_GraphReFresh_data;
        Graph_Data_t graphData;

        uint8_t temp_datalength = FRAMEHEAD_LEN + CMD_ID_LEN + UI_DATA_HEAD_LEN + UI_GRAPH_DATA_LEN * cnt + TAIL_CRC16_LEN; // 计算交互数据长度

        static uint8_t buffer[512]; // 交互数据缓存

        va_list ap;        // 创建一个 va_list 类型变量
        va_start(ap, cnt); // 初始化 va_list 变量为一个参数列表

        UI_GraphReFresh_data.FrameHeader.SOF = UI_SOF;
        UI_GraphReFresh_data.FrameHeader.Data_Length = UI_DATA_HEAD_LEN + cnt * UI_GRAPH_DATA_LEN;
        UI_GraphReFresh_data.FrameHeader.Seq = UI_Seq;
        UI_GraphReFresh_data.FrameHeader.CRC8 = Get_CRC8_Check_Sum((uint8_t *)&UI_GraphReFresh_data, CRC8_LEN, 0xFF);

        UI_GraphReFresh_data.CmdID = UI_CMD_Robo_Exchange;

        switch (cnt)
        {
        case 1:
            UI_GraphReFresh_data.datahead.Data_ID = UI_Data_ID_Draw1;
            break;
        case 2:
            UI_GraphReFresh_data.datahead.Data_ID = UI_Data_ID_Draw2;
            break;
        case 5:
            UI_GraphReFresh_data.datahead.Data_ID = UI_Data_ID_Draw5;
            break;
        case 7:
            UI_GraphReFresh_data.datahead.Data_ID = UI_Data_ID_Draw7;
            break;
        default:
            break;
        }

        UI_GraphReFresh_data.datahead.Receiver_ID = UI_Cilent_ID;
        UI_GraphReFresh_data.datahead.Sender_ID = UI_Robot_ID;
        memcpy(buffer, (uint8_t *)&UI_GraphReFresh_data, FRAMEHEAD_LEN + CMD_ID_LEN + UI_DATA_HEAD_LEN); // 将帧头、命令码、交互数据帧头三部分复制到缓存中

        for (uint8_t i = 0; i < cnt; i++) // 发送交互数据的数据帧，并计算CRC16校验值
        {
            graphData = va_arg(ap, Graph_Data_t); // 访问参数列表中的每个项,第二个参数是你要返回的参数的类型,在取值时需要将其强制转化为指定类型的变量
            // 将数据部分提取出来
            memcpy(buffer + (FRAMEHEAD_LEN + CMD_ID_LEN + UI_DATA_HEAD_LEN + UI_GRAPH_DATA_LEN * i), (uint8_t *)&graphData, UI_GRAPH_DATA_LEN);
        }
        Append_CRC16_Check_Sum(buffer, temp_datalength);
        UI_Send(buffer, temp_datalength);

        va_end(ap); // 结束可变参数的获取
    }

    /************************************************更新字符型数据*************************************************
    **参数:  String_Data_t string 传输字符串定义结构体
    **********************************************************************************************************/
    void referee_ui_c::UI_Char_Refresh(String_Data_t string)
    {

        static UI_CharReFresh_t UI_CharReFresh_data;
        uint8_t temp_datalength = UI_DATA_HEAD_LEN + UI_STRING_DATA_LEN;

        UI_CharReFresh_data.FrameHeader.SOF = UI_SOF;
        UI_CharReFresh_data.FrameHeader.Data_Length = temp_datalength;
        UI_CharReFresh_data.FrameHeader.Seq = UI_Seq;
        UI_CharReFresh_data.FrameHeader.CRC8 = Get_CRC8_Check_Sum((uint8_t *)&UI_CharReFresh_data, CRC8_LEN, 0xff);
        UI_CharReFresh_data.CmdID = UI_CMD_Robo_Exchange;

        UI_CharReFresh_data.datahead.Data_ID = UI_Data_ID_DrawChar;
        UI_CharReFresh_data.datahead.Sender_ID = UI_Robot_ID;
        UI_CharReFresh_data.datahead.Receiver_ID = UI_Cilent_ID; //

        string.Graph_Control.end_angle = strlen(string.show_Data);
        UI_CharReFresh_data.String_Data = string;

        UI_CharReFresh_data.frametail = Get_CRC16_Check_Sum((uint8_t *)&UI_CharReFresh_data, UI_DATA_HEAD_LEN + CMD_ID_LEN + temp_datalength, 0xFFFF);

        UI_Send((uint8_t *)&UI_CharReFresh_data, ALL_STRING_LEN);
        UI_Seq++;
    }
}