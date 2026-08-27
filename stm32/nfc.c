#include "nfc.h"
#include "delay.h"

//唤醒命令
const u8 NFC_WakeUp[] = {0x55,0x55,0,0,0,0,0,0,0,0,0,0,0,0,0xFF,0x03,0xFD,0xD4,0x14,0x01,0x17,0x00};
//寻卡命令
const u8 NFC_SearchCard[] = {0x00,0x00,0xFF,0x04,0xFC,0xD4,0x4A,0x01,0x00,0xE1,0x00};

UART_FRAME_TypeDef UART2Frame;

u8 NFC_WakeUp_Ok = 0;         //NFC唤醒标志
u8 NFC_find_Card = 0;         //NFC找到一张卡
u8 NFC_sendcmd_find = 1;      //NFC收到卡帧头等待
u8 NFC_wait_Card = 0;
u8 NFC_read_id_flag=0;
u8 NFC_DataBlock[16];         //存储一个BLOCK的数据
u8 USART2_RX_BUF[USART2_REC_LEN];   //接收缓冲,最大USART_REC_LEN个字节.
u16 USART2_RX_STA=0;          //接收状态标记
u16 slen;                     //缓冲数组长度
u8 Sys_Stat;                  //nfc id卡状态
u8 Sum = 0;                   //校验和
u8 REC_LEN=0;
u8 led_flag=0;

//===== 刷卡流水灯（非阻塞状态机）=====
u8 led_flow_active = 0;       //流水灯是否在运行
u8 led_flow_pos   = 1;        //当前亮点位置(1~6)
u8 led_flow_count = 0;        //已推进帧数

/**
 * @brief 流水灯推进一帧：亮点在6个灯上依次流动(亮-灭交替)
 *        每调用一次移动一格，跑满3圈后自动熄灭
 */
void FlowLight_Update(void)
{
    u8 i;
    if(!led_flow_active) return;

    //点亮当前位，其余熄灭
    for(i=1;i<=6;i++)
    {
        if(i==led_flow_pos)
            R_ws2812_rgb(i, WS_GREEN);
        else
            R_ws2812_rgb(i, WS_DARK);
    }
    R_ws2812_refresh(led_num);

    //移到下一个灯
    led_flow_pos++;
    if(led_flow_pos>6) led_flow_pos=1;

    //跑满3圈后停止并全部熄灭
    led_flow_count++;
    if(led_flow_count >= 6*3)
    {
        led_flow_active = 0;
        led_flow_count  = 0;
        for(i=1;i<=6;i++) R_ws2812_rgb(i, WS_DARK);
        R_ws2812_refresh(led_num);
    }
}

/**
 * @brief USART2发送多字节
 */
void UART2SendFrame(u8 *buf,u16 len)
{
    u16 i;
    for(i=0;i<len;i++)
    {
        while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
        USART_SendData(USART2,buf[i]);
    }
    while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);
}

/**
 * @brief 串口输出十六进制
 */
void put_HEX(USART_TypeDef* USARTx,u8 *buf,u16 len)
{
    u16 i;
    for(i=0;i<len;i++)
    {
        USART_SendData(USARTx, buf[i]>>4 >= 0xA ? (buf[i]>>4)-0xA+'A' : (buf[i]>>4)+'0');
        while(USART_GetFlagStatus(USARTx,USART_FLAG_TXE)==RESET);

        USART_SendData(USARTx, (buf[i]&0x0F)>=0xA ? (buf[i]&0x0F)-0xA+'A' : (buf[i]&0x0F)+'0');
        while(USART_GetFlagStatus(USARTx,USART_FLAG_TXE)==RESET);
        USART_SendData(USARTx,' ');
        while(USART_GetFlagStatus(USARTx,USART_FLAG_TXE)==RESET);
    }
    USART_SendData(USARTx,'\r');
    while(USART_GetFlagStatus(USARTx,USART_FLAG_TXE)==RESET);
    USART_SendData(USARTx,'\n');
    while(USART_GetFlagStatus(USARTx,USART_FLAG_TXE)==RESET);
}

/**
 * @brief NFC轮询，放在main while(1)
 *        唤醒应答与寻卡帧解析在 stm32f10x_it.c 的 USART2_IRQHandler 中完成
 */
void NFC_Handler(void)
{
    static u8 wakeup_tick = 0;  //唤醒命令发送节拍(每100ms+1)

    if(led_flow_active)          //流水灯运行中，推进一帧
    {
        FlowLight_Update();
        return;
    }

    if(NFC_WakeUp_Ok == 0)       //未唤醒：周期性发送唤醒命令
    {
        if(++wakeup_tick >= 5)   //每5*100ms=500ms发一次
        {
            wakeup_tick = 0;
            UART2Frame.RxCounter = 0;
            UART2SendFrame((u8*)NFC_WakeUp, sizeof(NFC_WakeUp));//发送唤醒指令
        }
        return;
    }

    if(NFC_find_Card==1)         //刷到卡
    {
        FoundCard_Handler();
    }
    else if(NFC_find_Card==0 && NFC_sendcmd_find==1)
    {
        UART2Frame.RxCounter=0;
        UART2SendFrame((u8*)NFC_SearchCard, sizeof(NFC_SearchCard));//发送寻卡指令
        NFC_sendcmd_find=0;
        delay_ms(200);
    }
}

/**
 * @brief 刷卡成功：启动流水灯
 */
void FoundCard_Handler(void)
{
    NFC_find_Card=0;    //清除标识
    NFC_sendcmd_find=1; //允许继续发寻卡指令

    //启动流水灯
    led_flow_active=1;
    led_flow_pos=1;
    led_flow_count=0;
}
