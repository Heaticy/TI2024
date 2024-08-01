#ifndef _MYIIC_H
#define _MYIIC_H
#include "stdint.h"
#include "stm32h7xx.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	
//IO方向设置
#define SDA_IN()  {GPIOC->MODER&=~(uint32_t)(3U<<(15*2));GPIOC->MODER|=(uint32_t)(0U<<15*2);}	//PC15输入模式
#define SDA_OUT() {GPIOC->MODER&=~(uint32_t)(3U<<(15*2));GPIOC->MODER|=(uint32_t)(1U<<15*2);} //PC15输出模式


#define IIC_SCL_0  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_RESET) //SCL
#define IIC_SCL_1  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_SET) //SCL

#define IIC_SDA_0  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_RESET) //SDA
#define IIC_SDA_1  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_SET) //SDA

#define READ_SDA  HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_15)  //输入SDA
//IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口				 
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
void IIC_Send_Byte(uint8_t txd);			//IIC发送一个字节
uint8_t IIC_Read_Byte(uint8_t ack);//IIC读取一个字节
uint8_t IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号

void IIC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
uint8_t IIC_Read_One_Byte(uint8_t daddr,uint8_t addr);	 
#endif

