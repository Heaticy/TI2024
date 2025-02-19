/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dac.h"
#include "lptim.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AD9959.h"
#include "HMC472.h"
#include "debug.h"
#include "ch455.h"
#include "delay.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define PI 3.1415926535

// CW用下面这个数组
uint32_t vamp_L[11][10] = { {91,182,273,364,455,550,637,728,818,906}, //30MHz,vamp_L_f_i = 0
													 {90,182,272,363,454,545,634,724,813,903}, //31MHz,vamp_L_f_i = 1
												   {90,180,272,363,453,541,632,720,808,897}, //32MHz,vamp_L_f_i = 2
												   {90,180,272,360,451,532,625,716,806,896}, //33MHz,vamp_L_f_i = 3
													 {90,180,270,360,451,533,622,715,805,893}, //34MHz,vamp_L_f_i = 4
													 {90,180,270,360,449,533,622,712,802,892}, //35MHz,vamp_L_f_i = 5
													 {90,180,270,360,450,534,621,714,801,892}, //36MHz,vamp_L_f_i = 6
													 {90,181,271,361,451,534,621,712,801,892}, //37MHz,vamp_L_f_i = 7
													 {91,182,273,360,451,533,621,713,801,892}, //38MHz,vamp_L_f_i = 8
													 {91,183,275,360,450,533,621,713,801,892}, //39MHz,vamp_L_f_i = 9
													 {92,185,277,360,450,533,621,713,801,892}, //40MHz,vamp_L_f_i = 10
};
// 除了CW，都用下面这个数组
uint32_t vamp_L_hmc[11][10] = { {91,182,273,364,455,536,625,714,750,800}, //30MHz,vamp_L_f_i = 0
													 {90,182,272,362,454,534,622,710,750,800}, //31MHz,vamp_L_f_i = 1
												   {90,180,270,361,452,532,620,708,750,800}, //32MHz,vamp_L_f_i = 2
												   {90,180,269,360,451,530,619,706,750,800}, //33MHz,vamp_L_f_i = 3
													 {90,180,270,360,450,530,618,705,750,800}, //34MHz,vamp_L_f_i = 4
													 {90,180,270,360,450,530,618,705,750,800}, //35MHz,vamp_L_f_i = 5
													 {90,180,270,361,451,531,619,707,750,800}, //36MHz,vamp_L_f_i = 6
													 {90,181,271,362,453,533,621,710,750,800}, //37MHz,vamp_L_f_i = 7
													 {91,182,273,364,455,536,625,713,750,800}, //38MHz,vamp_L_f_i = 8
													 {91,183,275,367,458,540,630,719,750,800}, //39MHz,vamp_L_f_i = 9
													 {92,185,277,370,463,545,635,726,750,800}, //40MHz,vamp_L_f_i = 10
};
uint8_t vamp_L_f_i = 0; // 默认是30MHz
uint8_t vamp_L_i = 0;


uint32_t f_L = 30000000;

uint8_t deg[7] = {40,70,101,131,162,193,225}; // 40
//uint8_t deg[7] = {0,30,60,90,120,150,180}; // 
uint8_t deg_i = 0;
uint8_t init_flag = 0;
uint8_t init_flag_delay = 0;

uint16_t attenuation = 0;
uint16_t attenuation_hmc= 0;
uint8_t  attenuation_real[11] = {0,2,4,6,8,10,12,14,16,18,20}; // 对应显示0db-20db衰减
uint8_t  attenuation_real_maxvamp[11] = {0,1,3,5,7,9,11,13,15,17,19}; // 最大有效值对应的数组
// 调试范围是0-31

uint8_t time_delay[7] = {0,78,111,139,167,195,222}; // ns级单位
uint8_t time_delay_i = 0;

//float adjust[7] = {0.45f,0.5f,0.6f,0.7f,0.85f,0.95f,1.1f};
float adjust[7] = {0.4f,0.45f,0.55f,0.65f,0.8f,0.9f,1.05f};
uint8_t adjust_i = 0;

float vp_B = 0.27f; // 峰值
#define VP_B 0.27f
#define DAC_VOLTAGE_DEFAULT 0.9f // vp_B / 0.3, max
uint16_t vamp_A_1 = 955.6667f * VP_B; // output1
uint16_t vamp_A_2 = 963.3333f * VP_B; //  output3
float dac_voltage = DAC_VOLTAGE_DEFAULT;
uint32_t dac_value = (uint32_t)(DAC_VOLTAGE_DEFAULT/3.3f*4095.0f); //dac code

float vamp_L_2_scale = 1.1667f;
uint32_t vamp_L_2 = 114;

float w_s_ns = 0; // 调制信号频率2MHz
float w_c_ns = 0;
uint32_t delay_phi_s = 0; // 以度为单位
uint32_t delay_phi_c = 0;

extern uint16_t CH455_KEY_RX_FLAG; //键盘接收状态标记	
extern uint8_t CH455_KEY_NUM;			//按下键盘的值

uint8_t mode = 1; // 1是加法模式，0是减法模式，2是CW测量模式


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint32_t normalize_angle(uint32_t angle) {
    // 将角度限制在0到360之间
    angle = angle % 360;
    // 如果角度为负值，加360使其为正
    return angle;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    
		//注意要调整中断优先级使SYSTICK的高于按键中断 
		HAL_Delay(50); //延时实现消抖，但在复杂程序不建议在中断中使用延时函数
		
		if (GPIO_Pin == GPIO_PIN_12){	// 判断中断来自于PC12管脚
				//判断PC12被按下
				if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12)){
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
					mode = 0;
					printf("t12.txt=\"- mode \"\xff\xff\xff");
				}
		}
		else if (GPIO_Pin == GPIO_PIN_3){	// 判断中断来自于PE3管脚
				if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3)){
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
					mode = 1;
					printf("t12.txt=\"+ mode \"\xff\xff\xff");
				}
		}
		else if (GPIO_Pin == GPIO_PIN_13){
			
			CH455_KEY_RX_FLAG = 1;
			
			CH455_KEY_NUM = CH455_Key_Read();

		
			if (CH455_KEY_NUM == 0){ // 调节RMS
				if (mode == 0){ // 减法模式
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
					HAL_Delay(100);
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);
					if(vamp_L_i - 1 > 0){
						vamp_L_i = vamp_L_i - 1;
					}
					else if(vamp_L_i == 0){
						vamp_L_i = 9;
					}
					else{
						vamp_L_i = 0;
					}
				}
				if (mode == 1 || mode == 2){ // 加法模式
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
					HAL_Delay(100);
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
					if(vamp_L_i + 1 < 9){
						vamp_L_i = vamp_L_i + 1;
					}
					else if(vamp_L_i == 9){
						vamp_L_i = 0;
					}
					else{
						vamp_L_i = 9;
					}
				}
				if (mode != 2){
					vamp_L_2 = vamp_L_hmc[vamp_L_f_i][vamp_L_i] * vamp_L_2_scale;
					if (vamp_L_2 > 1022) {AD9959_SetAmp4Channel(vamp_L_hmc[vamp_L_f_i][vamp_L_i],vamp_A_1,1023,vamp_A_2);}
					else {AD9959_SetAmp4Channel(vamp_L_hmc[vamp_L_f_i][vamp_L_i],vamp_A_1,vamp_L_2,vamp_A_2);}
				}
				else{
					vamp_L_2 = vamp_L[vamp_L_f_i][vamp_L_i] * vamp_L_2_scale;
					if (vamp_L_2 > 1022) {AD9959_SetAmp4Channel(vamp_L[vamp_L_f_i][vamp_L_i],vamp_A_1,1023,vamp_A_2);}
					else {AD9959_SetAmp4Channel(vamp_L[vamp_L_f_i][vamp_L_i],vamp_A_1,vamp_L_2,vamp_A_2);}
				}
				printf("t6.txt=\"%d mV\"\xff\xff\xff",(vamp_L_i+1)*100);
			}
			if (CH455_KEY_NUM == 1){ // 调节调制，最好：500mV，30MHz
				if (mode == 0){ // 减法模式
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
					HAL_Delay(100);
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);
					if(adjust_i - 1 > 0){
						adjust_i = adjust_i - 1;
					}
					else if(adjust_i == 0){
						adjust_i = 6;
					}
					else{
						adjust_i = 0;
					}
				}
				if (mode == 1){ // 加法模式
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
					HAL_Delay(100);
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
					if(adjust_i + 1 < 6){
						adjust_i = adjust_i + 1;
					}
					else if(adjust_i == 6){
						adjust_i = 0;
					}
					else{
						adjust_i = 6;
					}
				}
				dac_voltage = vp_B / adjust[adjust_i];
				if (dac_voltage > 3.3f) {dac_voltage = 3.3f;}
				dac_value = (uint32_t)(dac_voltage/3.3f*4095.0f);
				HAL_DAC_SetValue(&hdac1,DAC_CHANNEL_1,DAC_ALIGN_12B_R,dac_value);//channel 1 output
				
				printf("t7.txt=\"%d %% \"\xff\xff\xff",(adjust_i+3)*10);
			}
			if (CH455_KEY_NUM == 2){ // 调节频率
				if (mode == 0){ // 减法模式
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
					HAL_Delay(100);
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);
					if(f_L - 1000000 > 30000000){
						f_L = f_L - 1000000;
					}
					else if(f_L == 30000000){
						f_L = 40000000;
					}
					else{
						f_L = 30000000;
					}
				}
				if (mode == 1 || mode == 2){ // 加法模式
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
					HAL_Delay(100);
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
					if(f_L + 1000000 < 40000000){
						f_L = f_L + 1000000;
					}
					else if(f_L == 40000000){
						f_L = 30000000;
					}
					else{
						f_L = 40000000;
					}
				}
				
				AD9959_SetFrequency4Channel(f_L,2000000,f_L,2000000);
				vamp_L_f_i = f_L/1000000 - 30;
				if (mode != 2){
					vamp_L_2 = vamp_L_hmc[vamp_L_f_i][vamp_L_i] * vamp_L_2_scale;
					if (vamp_L_2 > 1022) {AD9959_SetAmp4Channel(vamp_L_hmc[vamp_L_f_i][vamp_L_i],vamp_A_1,1023,vamp_A_2);}
					else {AD9959_SetAmp4Channel(vamp_L_hmc[vamp_L_f_i][vamp_L_i],vamp_A_1,vamp_L_2,vamp_A_2);}
				}
				else{
					vamp_L_2 = vamp_L[vamp_L_f_i][vamp_L_i] * vamp_L_2_scale;
					if (vamp_L_2 > 1022) {AD9959_SetAmp4Channel(vamp_L[vamp_L_f_i][vamp_L_i],vamp_A_1,1023,vamp_A_2);}
					else {AD9959_SetAmp4Channel(vamp_L[vamp_L_f_i][vamp_L_i],vamp_A_1,vamp_L_2,vamp_A_2);}
				}
				
				printf("t8.txt=\"%d MHz  \"\xff\xff\xff",f_L/1000000);
			}
		
			if (CH455_KEY_NUM == 3){ // 调节衰减db
				if (mode == 0){ // 减法模式
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
					HAL_Delay(100);
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);
					if(attenuation - 2 > 0){
						attenuation = attenuation - 2;
					}
					else if(attenuation == 0){
						attenuation = 20;
					}
					else{
						attenuation = 0;
					}
				}
				if (mode == 1){ // 加法模式
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
					HAL_Delay(100);
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
					if(attenuation + 2 < 20){
						attenuation = attenuation + 2;
					}
					else if(attenuation == 20){
						attenuation = 0;
					}
					else{
						attenuation = 20;
					}
				}
				
				attenuation_hmc = attenuation_real[attenuation / 2] * 2;
				if(vamp_L_i == 9){attenuation_hmc = attenuation_real_maxvamp[attenuation / 2] * 2;}
				HMC472set(attenuation_hmc);
				
				printf("t9.txt=\"%d dB\"\xff\xff\xff",attenuation);
			}
			
			if (CH455_KEY_NUM == 4){ // 调节延迟
				if (mode == 0){ // 减法模式
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
					HAL_Delay(100);
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);
					if(time_delay_i > 0){
						time_delay_i = time_delay_i - 1;
					}
					else{
						time_delay_i = 6;
					}
				}
				if (mode == 1){ // 加法模式
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
					HAL_Delay(100);
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
					if(time_delay_i < 6){
						time_delay_i = time_delay_i + 1;
					}
					else{
						time_delay_i = 0;
					}
				}
				w_s_ns = 2*2*0.001f; // 调制信号频率2MHz
				w_c_ns = 2*(f_L/1000000)*0.001f;
				delay_phi_s = w_s_ns * time_delay[time_delay_i] * 180; // 以度为单位
				delay_phi_c = w_c_ns * time_delay[time_delay_i] * 180;
				
				if (init_flag_delay){Init_AD9959();}
				
				delay_phi_s = normalize_angle(delay_phi_s);
				delay_phi_c = normalize_angle(delay_phi_c);
				
				AD9959_SetPhase4Channel(0,0,360-delay_phi_c,360-delay_phi_s);
				AD9959_SetFrequency4Channel(f_L,2000000,f_L,2000000);
				vamp_L_2 = vamp_L_hmc[vamp_L_f_i][vamp_L_i] * vamp_L_2_scale;
				if (vamp_L_2 > 1022) {AD9959_SetAmp4Channel(vamp_L_hmc[vamp_L_f_i][vamp_L_i],vamp_A_1,1023,vamp_A_2);}
				else {AD9959_SetAmp4Channel(vamp_L_hmc[vamp_L_f_i][vamp_L_i],vamp_A_1,vamp_L_2,vamp_A_2);}
	
				init_flag = 0;
				if (time_delay_i == 6){init_flag = 1;}
				
				if (time_delay_i == 0){printf("t10.txt=\"%d ns \"\xff\xff\xff",0);}
				else {printf("t10.txt=\"%d ns \"\xff\xff\xff",50+(time_delay_i-1)*30);}
				
			}
			
			if (CH455_KEY_NUM == 5){ // 调节相位
				if (mode == 0){ // 减法模式
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
					HAL_Delay(100);
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);
					if(deg_i - 1 > 0){
						deg_i = deg_i - 1;
					}
					else if(deg_i == 0){
						deg_i = 6;
					}
					else{
						deg_i = 0;
					}
				}
				if (mode == 1){ // 加法模式
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
					HAL_Delay(100);
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
					if(deg_i + 1 < 6){
						deg_i = deg_i + 1;
					}
					else if(deg_i == 6){
						deg_i = 0;
					}
					else{
						deg_i = 6;
					}
				}
				
				if (init_flag){Init_AD9959();}
				
				AD9959_SetPhase4Channel(0,0,deg[deg_i],0);
				AD9959_SetFrequency4Channel(f_L,2000000,f_L,2000000);
				vamp_L_2 = vamp_L_hmc[vamp_L_f_i][vamp_L_i] * vamp_L_2_scale;
				if (vamp_L_2 > 1022) {AD9959_SetAmp4Channel(vamp_L_hmc[vamp_L_f_i][vamp_L_i],vamp_A_1,1023,vamp_A_2);}
				else {AD9959_SetAmp4Channel(vamp_L_hmc[vamp_L_f_i][vamp_L_i],vamp_A_1,vamp_L_2,vamp_A_2);}
				
				
				init_flag = 0;
				if (deg_i == 6){init_flag = 1;}
				
				printf("t11.txt=\"%d deg \"\xff\xff\xff",deg_i*30);
			}
			
			if (CH455_KEY_NUM == 15){ // 开启CW测量模式，直流偏置为2V
				uint8_t i;
				mode = 2;
				for (i = 0; i < 3; i++){
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
					HAL_Delay(100);
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
					HAL_Delay(100);
				}
				dac_voltage = 2.0f;
				dac_value = (uint32_t)(dac_voltage/3.3f*4095.0f);
				HAL_DAC_SetValue(&hdac1,DAC_CHANNEL_1,DAC_ALIGN_12B_R,dac_value);//channel 1 output
				
				vamp_A_1 = 0;
				AD9959_SetFrequency4Channel(f_L,2000000,f_L,2000000);
				vamp_L_f_i = f_L/1000000 - 30;
				vamp_L_2 = vamp_L[vamp_L_f_i][vamp_L_i] * vamp_L_2_scale;
				if (vamp_L_2 > 1022) {AD9959_SetAmp4Channel(vamp_L[vamp_L_f_i][vamp_L_i],vamp_A_1,1023,vamp_A_2);}
				else {AD9959_SetAmp4Channel(vamp_L[vamp_L_f_i][vamp_L_i],vamp_A_1,vamp_L_2,vamp_A_2);}
				
				printf("t12.txt=\"CW(+) mode \"\xff\xff\xff");
				printf("t6.txt=\"%d mV\"\xff\xff\xff",(vamp_L_i+1)*100);
				printf("t8.txt=\"%d MHz  \"\xff\xff\xff",f_L/1000000);
			}
//			__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_13);
		}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */
	uint8_t i = 0;
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_LPTIM1_Init();
  MX_LPTIM2_Init();
  MX_TIM4_Init();
  MX_DAC1_Init();
	HMC472_Init();
  /* USER CODE BEGIN 2 */
		for (int i = 0; i < 11; ++i) {
        for (int j = 0; j < 10; ++j) {
            vamp_L[i][j] = vamp_L[i][j] * 1.05f;
        }
    }
	HAL_DAC_Start(&hdac1,DAC_CHANNEL_1);//dac1 open
	HAL_DAC_SetValue(&hdac1,DAC_CHANNEL_1,DAC_ALIGN_12B_R,dac_value);//channel 1 output
	
	Init_AD9959();
	AD9959_SetPhase4Channel(0,0,deg[deg_i],0);
	AD9959_SetFrequency4Channel(f_L,2000000,f_L,2000000);
	vamp_L_2 = vamp_L_hmc[vamp_L_f_i][vamp_L_i] * vamp_L_2_scale;
	if (vamp_L_2 > 1022) {AD9959_SetAmp4Channel(vamp_L_hmc[vamp_L_f_i][vamp_L_i],vamp_A_1,1023,vamp_A_2);}
	else {AD9959_SetAmp4Channel(vamp_L_hmc[vamp_L_f_i][vamp_L_i],vamp_A_1,vamp_L_2,vamp_A_2);}
	
	
	if(vamp_L_i == 9){attenuation_hmc = attenuation_real_maxvamp[attenuation / 2] * 2;}
	attenuation_hmc = attenuation_real[attenuation / 2] * 2;
	HMC472set(attenuation_hmc);
	
	printf("t12.txt=\"+ mode \"\xff\xff\xff");
	printf("t6.txt=\"%d mV\"\xff\xff\xff",(vamp_L_i+1)*100);
	printf("t7.txt=\"%d %% \"\xff\xff\xff",(adjust_i+3)*10);
	printf("t8.txt=\"%d MHz  \"\xff\xff\xff",f_L/1000000);
	printf("t11.txt=\"%d deg \"\xff\xff\xff",deg_i*30);
	printf("t9.txt=\"%d dB\"\xff\xff\xff",attenuation);
	printf("t10.txt=\"%d ns \"\xff\xff\xff",0);
	
  for (i = 0; i < 3; i = i + 1){
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);
		HAL_Delay(200);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
		HAL_Delay(200);
	}
	
	delay_init(480);//延时函数初始化
	delay_ms(100);//100ms必须加，ch455上电后需要一段时间才能启动
	CH455_init();//ch455 初始化
	
	
	CH455_Display(1,1);//数码管1显示1
	CH455_Display(2,2);//数码管2显示2
	CH455_Display(3,3);//数码管3显示3
	CH455_Display(4,4);//数码管4显示4
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
	
	while (1)
  {
		if(CH455_KEY_RX_FLAG == 1)	//键盘按下，在终端中读取CH455寄存器完成后
		{
			CH455_Display(1,CH455_KEY_NUM);//数码管显示按键值
			CH455_Display(2,CH455_KEY_NUM);
			CH455_Display(3,CH455_KEY_NUM);
			CH455_Display(4,CH455_KEY_NUM);
			CH455_KEY_RX_FLAG = 0;
		}
		HAL_Delay(100);		//延时
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_CSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  RCC_OscInitStruct.CSICalibrationValue = RCC_CSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_CSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
