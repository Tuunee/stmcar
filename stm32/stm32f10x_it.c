/**
  ******************************************************************************
  * @file    GPIO/IOToggle/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h" 
#include "nfc.h"
#include "usart.h"


 
void NMI_Handler(void)
{
}
 
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}
 
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

 
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}
 
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}
 
void SVC_Handler(void)
{
}
 
void DebugMon_Handler(void)
{
}
 
void PendSV_Handler(void)
{
}
 
/* SysTick_Handler 已由 control_system.c 定义，此处注释掉，避免重复定义 L6200E */
/*
void SysTick_Handler(void)
{
}
*/

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

void USART2_IRQHandler(void)
{
    static u8 ack_buf[8];       //滑动窗口，检测唤醒应答
    u8 ch;
    u8 i;

    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        ch = USART_ReceiveData(USART2);

        if(NFC_WakeUp_Ok==0)            //未唤醒：等待唤醒应答
        {
            //移位滑动窗口
            for(i=0;i<6;i++) ack_buf[i] = ack_buf[i+1];
            ack_buf[6] = ch;

            //唤醒成功条件1：收到PN532 ACK帧  00 00 FF 00 FF 00
            if(ack_buf[0]==0x00 && ack_buf[1]==0x00 && ack_buf[2]==0xFF &&
               ack_buf[3]==0x00 && ack_buf[4]==0xFF && ack_buf[5]==0x00)
            {
                NFC_WakeUp_Ok = 1;      //唤醒成功
            }
            //唤醒成功条件2：收到唤醒应答帧 00 00 FF xx xx D5 14 ...
            else if(ack_buf[0]==0x00 && ack_buf[1]==0x00 && ack_buf[2]==0xFF &&
                    ack_buf[5]==0xD5 && ack_buf[6]==0x14)
            {
                NFC_WakeUp_Ok = 1;      //唤醒成功
            }
        }
        else                            //唤醒成功,进入寻卡流程
        {
            UART2Frame.RxBuffer[UART2Frame.RxCounter] = ch;
            UART2Frame.RxCounter++;
            if(UART2Frame.RxCounter==25)
            {
                memcpy(USART2_RX_BUF, (uint8_t*)UART2Frame.RxBuffer,25);
                put_HEX(USART1,USART2_RX_BUF,25);

                //识别三张已知卡（UID位于19~22）
                if(
                ((0x25==USART2_RX_BUF[19])&&(0xF7==USART2_RX_BUF[20])&&(0x48==USART2_RX_BUF[21])&&(0x06==USART2_RX_BUF[22]))
                ||((0x50==USART2_RX_BUF[19])&&(0x84==USART2_RX_BUF[20])&&(0xFC==USART2_RX_BUF[21])&&(0x23==USART2_RX_BUF[22]))
                ||((0x40==USART2_RX_BUF[19])&&(0x74==USART2_RX_BUF[20])&&(0x80==USART2_RX_BUF[21])&&(0x23==USART2_RX_BUF[22]))
                )
                {
                    NFC_find_Card = 1;  //刷到卡
                }

                //保存卡号（UID）
                NFC_DataBlock[0] = USART2_RX_BUF[19];
                NFC_DataBlock[1] = USART2_RX_BUF[20];
                NFC_DataBlock[2] = USART2_RX_BUF[21];
                NFC_DataBlock[3] = USART2_RX_BUF[22];

                memset((uint8_t*)UART2Frame.RxBuffer,0,50);
                memset((uint8_t*)USART2_RX_BUF,0,50);
                UART2Frame.RxCounter=0;
            }
        }
    }
}
