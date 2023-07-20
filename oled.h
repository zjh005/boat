//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
//����Ӳ������Ƭ��STM32F103RCT6,����72M  ��Ƭ��������ѹ3.3V��5V
//QDtech-OLEDҺ������ for STM32
//xiao��@ShenZhen QDtech co.,LTD
//��˾��վ:www.qdtech.net
//�Ա���վ��http://qdtech.taobao.com
//��˾�ṩ����֧�֣��κμ������⻶ӭ��ʱ����ѧϰ 
//����:QDtech2008@gmail.com 
//Skype:QDtech2008
//��������QQȺ:324828016
//��������:2018/6/6
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������ȫ�����Ӽ������޹�˾ 2009-2019
//All rights reserved
/****************************************************************************************************
//=========================================��Դ����================================================//
//      5V  ��DC 5V��Դ
//     GND  �ӵ�
//======================================OLED�������߽���==========================================//
//��ģ��������������ΪIIC
//     SCL  ��PB13    // IICʱ���ź�
//     SDA  ��PB14    // IIC�����ź�
//======================================OLED�������߽���==========================================//
//��ģ��������������ΪIIC������Ҫ�ӿ����ź���    
//=========================================����������=========================================//
//��ģ�鱾����������������Ҫ�Ӵ�������
//============================================================================================//
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, QD electronic SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
**************************************************************************************************/
#ifndef __OLED_H
#define __OLED_H			  	 
#include "main.h"
#include "stdlib.h"	    	
#include "gpio.h"
#define OLED_MODE 0
#define SIZE 8
#define XLevelL		0x00
#define XLevelH		0x10
#define Max_Column	128
#define Max_Row		64
#define	Brightness	0xFF 
#define X_WIDTH 	128
#define Y_WIDTH 	64	    						  
//-----------------OLED IIC�˿ڶ���----------------  					   

#define OLED_SCLK_Clr() HAL_GPIO_WritePin(SCLK_GPIO_Port,SCLK_Pin,0)//CLK
#define OLED_SCLK_Set() HAL_GPIO_WritePin(SCLK_GPIO_Port,SCLK_Pin,1)

#define OLED_SDIN_Clr() HAL_GPIO_WritePin(SDIN_GPIO_Port,SDIN_Pin,0)//DIN
#define OLED_SDIN_Set() HAL_GPIO_WritePin(SDIN_GPIO_Port,SDIN_Pin,1)
 		     
#define OLED_CMD  0	//д����
#define OLED_DATA 1	//д����

#define IIC_SLAVE_ADDR 0x78  //IIC slave device address

//OLED�����ú���
void OLED_WR_Byte(unsigned dat,unsigned cmd);  
void OLED_Display_On(void);
void OLED_Display_Off(void);	   							   		    
void OLED_Init(void);
void OLED_Clear(unsigned dat); 
void OLED_DrawPoint(uint8_t x,uint8_t y,uint8_t t);
void OLED_Fill(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2,uint8_t dot);
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t Char_Size);
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size);
void OLED_ShowString(uint8_t x,uint8_t y, uint8_t *p,uint8_t Char_Size);	 
void OLED_Set_Pos(unsigned char x, unsigned char y);
void OLED_ShowCHinese(uint8_t x,uint8_t y,uint8_t no);
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[]);
void Delay_50ms(unsigned int Del_50ms);
void Delay_1ms(unsigned int Del_1ms);
void fill_picture(unsigned char fill_Data);
void Picture(void);
void IIC_Start(void);
void IIC_Stop(void);
void Write_IIC_Command(unsigned char IIC_Command);
void Write_IIC_Data(unsigned char IIC_Data);
void Write_IIC_Byte(unsigned char IIC_Byte);

void IIC_Wait_Ack(void);
#endif