#ifndef __NFC_H
#define __NFC_H

#include "stm32f10x.h"
#include "sys.h"
#include "colorful_led.h"   // 提供 R_led_mode/R_led_CLC 等函数声明
#include <string.h>         // 提供 memcpy/memset（USART2 中断中使用）

#define USART2_REC_LEN  200

/* USART2 接收帧结构 */
typedef struct
{
    u8  RxBuffer[200];
    u16 RxCounter;
}UART_FRAME_TypeDef;

extern UART_FRAME_TypeDef UART2Frame;

/* PN532 指令帧 */
extern const u8 NFC_WakeUp[];       // 唤醒命令
extern const u8 NFC_SearchCard[];   // 寻卡命令

/* NFC 状态与数据 */
extern u8 NFC_WakeUp_Ok;            // NFC 唤醒标志
extern u8 NFC_find_Card;            // NFC 找到一张卡
extern u8 NFC_sendcmd_find;         // 是否可发送寻卡指令
extern u8 NFC_wait_Card;
extern u8 NFC_read_id_flag;
extern u8 NFC_DataBlock[16];        // 卡号（UID）
extern u8 USART2_RX_BUF[USART2_REC_LEN];
extern u16 USART2_RX_STA;
extern u16 slen;
extern u8 Sys_Stat;
extern u8 Sum;
extern u8 REC_LEN;
extern u8 led_flag;                 // LED 开关状态标志

/* 函数声明 */
void NFC_Handler(void);                     // NFC 轮询，main while(1) 调用
void FoundCard_Handler(void);               // 刷卡成功回调
void UART2SendFrame(u8 *buf,u16 len);       // USART2 发送多字节
void put_HEX(USART_TypeDef* USARTx,u8 *buf,u16 len); // 串口输出十六进制

#endif
