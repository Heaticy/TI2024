#include "ch455.h"
#include "myiic.h"
#include "stm32h7xx_hal.h"
#include "delay.h"

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
// STM32F429开发板
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

uint8_t CH450_buf[6];                 //6位数码管
const uint8_t BCD_decode_tab[0x10] = { 0X3F, 0X06, 0X5B, 0X4F, 0X66, 0X6D, 0X7D, 0X07, 0X7F, 0X6F, 0X77, 0X7C, 0X39, 0X5E, 0X79, 0X71 };
uint16_t CH455_KEY_RX_FLAG=0;       //键盘接收状态标记	
uint8_t CH455_KEY_NUM=0;					//按下键盘的值

void CH450_Write(uint16_t cmd)	//写命令
{
		IIC_Start();               /*启动总线*/
   	IIC_Send_Byte(((uint8_t)(cmd>>7)&CH450_I2C_MASK)|CH450_I2C_ADDR1);
		IIC_Wait_Ack(); 
   	IIC_Send_Byte((uint8_t)cmd);               /*发送数据*/
		IIC_Wait_Ack(); 
  	IIC_Stop();                 /*结束总线*/ 
}

void	CH450_buf_write( uint16_t cmd )  // 向CH450输出数据或者操作命令,自动建立数据映象
{
	if ( cmd & 0x1000 ) 
		{   // 加载数据的命令,需要备份数据到映象缓冲区
		CH450_buf[ (uint8_t)( ( cmd >> 8 ) - 2 ) & 0x07 ] = (uint8_t)( cmd & 0xFF );	// 备份数据到相应的映象单元
	}
	CH450_Write( cmd );	// 发出
}


void	CH450_buf_index( uint8_t index, uint8_t dat )  // 向CH450指定的数码管输出数据,自动建立数据映象
// index 为数码管序号,有效值为0到5,分别对应DIG2到DIG7
{
	uint16_t cmd;
	CH450_buf[ index ] = dat;	// 备份数据到相应的映象单元
	cmd = ( CH450_DIG2 + ( (uint16_t)index << 8 ) ) | dat ;	// 生成操作命令
	CH450_Write( cmd );	// 发出
}

void CH450_set_bit(uint8_t bit_addr)     //段位点亮
{
	uint8_t byte_addr;
	byte_addr=(bit_addr&0x38)>>3;
	if ( byte_addr < 6 ) CH450_buf_index( byte_addr, CH450_buf[byte_addr] | (1<<(bit_addr&0x07)) );
}

void CH450_clr_bit(uint8_t bit_addr)     //段位熄灭
{
	uint8_t byte_addr;
	byte_addr=(bit_addr&0x38)>>3;
	if ( byte_addr < 6 ) CH450_buf_index( byte_addr, CH450_buf[byte_addr] & ~(1<<(bit_addr&0x07)) );
}
//CH455初始化
void CH455_init(void)
{
	uint8_t i;
	for ( i = 0; i < 6; i ++ ) CH450_buf_index( i, 0 ); // 因为CH450复位时不清空显示内容，所以刚开电后必须人为清空，再开显示
	CH450_buf_write(CH450_SYSON2);	// 开启显示及键盘
// 如果需要定期刷新显示内容，那么只要执行7个命令，包括6个数据加载命令，以及1个开启显示命令
	CH455_Display(1,1);			// 显示BCD码1
	CH455_Display(2,2);
	CH455_Display(3,3);
	CH455_Display(4,4);
}

//数码管显示
void CH455_Display(uint8_t digital,uint8_t num)		
{
	if(digital == 1){
		CH450_buf_write(CH450_DIG4 | BCD_decode_tab[num]);  //第1位数码管显示
	}
	else if(digital == 2){
		CH450_buf_write(CH450_DIG5 | BCD_decode_tab[num]);  //第2位数码管显示
	}
	else if(digital == 3){
		CH450_buf_write(CH450_DIG6 | BCD_decode_tab[num]);  //第3位数码管显示
	}
	else if(digital == 4){
		CH450_buf_write(CH450_DIG7 | BCD_decode_tab[num]);  //第4位数码管显示
	}
}


uint8_t CH455_Key_Read(void)		// 向CH450发出按键读操作命令
{
		uint8_t keycode;
		uint8_t ch455_key_num=0;
	
   	IIC_Start();               
   	IIC_Send_Byte(((uint8_t)(CH450_GET_KEY>>7)&CH450_I2C_MASK)|0x01|CH450_I2C_ADDR1);
		IIC_Wait_Ack();
   	keycode=IIC_Read_Byte(0);  
		IIC_Stop();        

		//将码值转换为key1-key16
		switch(keycode)
		{
			case 0x44:
				ch455_key_num = 0;   //对应按键的编号key
				break;			
			case 0x45:
				ch455_key_num = 1;
				break;
			case 0x46:
				ch455_key_num = 2;
				break;
			case 0x47:
				ch455_key_num = 3;
				break;
			case 0x4C:
				ch455_key_num = 4;
				break;
			case 0x4D:
				ch455_key_num = 5;
				break;
			case 0x4E:
				ch455_key_num = 6;
				break;
			case 0x4F:
				ch455_key_num = 7;
				break;
			case 0x54:
				ch455_key_num = 8;
				break;
			case 0x55:
				ch455_key_num = 9;
				break;
			case 0x56:
				ch455_key_num = 10;
				break;
			case 0x57:
				ch455_key_num = 11;
				break;
			case 0x5C:
				ch455_key_num = 12;
				break;
			case 0x5D:
				ch455_key_num = 13;
				break;
			case 0x5E:
				ch455_key_num = 14;
				break;
			case 0x5F:
				ch455_key_num = 15;
				break;
									
			default:
				ch455_key_num = 0;
				break;
		}
		
		return(ch455_key_num);
}

// 按键中断回调函数，在中断环境中执行
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//	if (GPIO_Pin == GPIO_PIN_13)
//		{			
//			// 判断中断来自于PC13管脚
//				CH455_KEY_RX_FLAG = 1;
//			
//				CH455_KEY_NUM = CH455_Key_Read();
//				__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_13);
//			
//		}			
//}

