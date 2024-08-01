#ifndef _CH455_H
#define _CH455_H
#include "stdint.h"



//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	
//IO方向设置

/* ********************************************************************************************* */
/* 常量定义 */

/* CH450的常用命令码，如果考虑与I2C兼容，那么高8位应该右移1位 */
#define CH450_SYSOFF    0x0400					// 关闭显示
#define CH450_SYSON1    0x0401					// 开启显示
#define CH450_SYSON2    0x0403					// 开启显示,键盘
#define CH450_GET_KEY	0x0700					// 获取按键返回值
#define CH450_DIG2      0x1200		            // 数码管位2显示,需另加8位数据
#define CH450_DIG3      0x1300		            // 数码管位3显示,需另加8位数据
#define CH450_DIG4      0x1400		            // 数码管位4显示,需另加8位数据
#define CH450_DIG5      0x1500					// 数码管位5显示,需另加8位数据
#define CH450_DIG6      0x1600					// 数码管位6显示,需另加8位数据
#define CH450_DIG7      0x1700		            // 数码管位7显示,需另加8位数据

// CH450接口定义
#define		CH450_I2C_ADDR1		0x40			// CH450的地址
#define		CH450_I2C_MASK		0x3E			// CH450的高字节命令掩码

//有效按键代码
#define		CH450_KEY_MIN		0x42
#define		CH450_KEY_MAX		0x7F





void CH450_Write(uint16_t cmd);	// 向CH450发出操作命令
void	CH450_buf_write( uint16_t cmd );
void	CH450_buf_index( uint8_t index, uint8_t dat );
void CH450_set_bit(uint8_t bit_addr);
void CH450_clr_bit(uint8_t bit_addr);
// 对外子程序
extern void CH455_init(void);
extern uint8_t CH455_Key_Read(void);				// 向CH450发出按键读操作命令
extern void CH455_Display(uint8_t digital,uint8_t num);
#endif



