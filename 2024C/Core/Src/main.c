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

#define VAMP_MIN 116 // 有效值是100mV
#define VAMP_A_MIN 845

uint16_t vamp_L = VAMP_MIN;
uint16_t vamp_step = VAMP_MIN;
uint16_t vamp_min = VAMP_MIN;
uint16_t vamp_A = VAMP_A_MIN;

uint32_t f_L = 35000000;

uint16_t deg = 0;

uint16_t attenuation = 0;

double adjust = 0.3;

float dac_voltage = 3.0f;//set dac output to 3V

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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    
		//注意要调整中断优先级使SYSTICK的高于按键中断 
		HAL_Delay(50); //延时实现消抖，但在复杂程序不建议在中断中使用延时函数
		
		if (GPIO_Pin == GPIO_PIN_12){	// 判断中断来自于PC12管脚
				//判断PC12被按下
				if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12)){
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
					mode = 0;
					printf("t12.txt=\"- mode \"\xff\xff\xff");
				}
		}
		else if (GPIO_Pin == GPIO_PIN_3){	// 判断中断来自于PE3管脚
				if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3)){
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
					mode = 1;
					printf("t12.txt=\"+ mode \"\xff\xff\xff");
				}
		}
		else if (GPIO_Pin == GPIO_PIN_13){
			
			CH455_KEY_RX_FLAG = 1;
			
			CH455_KEY_NUM = CH455_Key_Read();

		
			if (CH455_KEY_NUM == 0){ // 调节RMS
				if (mode == 0){ // 减法模式
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
					HAL_Delay(500);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
					if(vamp_L - vamp_step > 0){
						vamp_L = vamp_L - vamp_step;
					}
					else if(vamp_L == 0){
						vamp_L = 1023;
					}
					else{
						vamp_L = 0;
					}
				}
				if (mode == 1){ // 加法模式
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
					HAL_Delay(500);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
					if(vamp_L + vamp_step < 1024){
						vamp_L = vamp_L + vamp_step;
					}
					else if(vamp_L == 1023){
						vamp_L = vamp_min;
					}
					else{
						vamp_L = 1023;
					}
				}
				AD9959_SetAmp4Channel(vamp_L,vamp_A,vamp_L,vamp_A);
				
				printf("t6.txt=\"%d \"\xff\xff\xff",vamp_L);
			}
			if (CH455_KEY_NUM == 1){ // 调节调制
				if (mode == 0){ // 减法模式
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
					HAL_Delay(500);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
					if(adjust - 0.1 > 0.22){
						adjust = adjust - 0.1;
					}
					else if((adjust - 0.3 < 0.05) | (0.3 - adjust < 0.05)){
						adjust = 0.9;
					}
					else{
						adjust = 0.3;
					}
				}
				if (mode == 1){ // 加法模式
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
					HAL_Delay(500);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
					if(adjust + 0.1 < 0.98){
						adjust = adjust + 0.1;
					}
					else if((adjust - 0.9 < 0.05) | (0.9 - adjust < 0.05)){
						adjust = 0.3;
					}
					else{
						adjust = 0.9;
					}
				}
				dac_voltage = 0.9 / adjust;
				if (dac_voltage > 3.25f){dac_voltage = 3.3f;}
				
				printf("t13.txt=\"%d %% \"\xff\xff\xff",(int)(dac_voltage));
				printf("t7.txt=\"%d %% \"\xff\xff\xff",(int)(adjust*100.0+0.01));
			}
			if (CH455_KEY_NUM == 2){ // 调节频率
				if (mode == 0){ // 减法模式
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
					HAL_Delay(100);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
					if(f_L - 1000000 > 30000000){
						f_L = f_L - 1000000;
					}
					else if(f_L == 30000000){
						f_L = 40000000;
					}
					else{
						adjust = 30000000;
					}
				}
				if (mode == 1){ // 加法模式
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
					HAL_Delay(100);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
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
				
				printf("t8.txt=\"%d  \"\xff\xff\xff",f_L);
			}
		
			if (CH455_KEY_NUM == 3){
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
				HAL_Delay(100);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
			}
			
			if (CH455_KEY_NUM == 4){
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
				HAL_Delay(100);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
			}
			
			if (CH455_KEY_NUM == 5){
				if (mode == 0){ // 减法模式
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
					HAL_Delay(100);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
					if(deg - 30 > 0){
						deg = deg - 30;
					}
					else if(deg == 0){
						deg = 180;
					}
					else{
						deg = 0;
					}
				}
				if (mode == 1){ // 加法模式
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
					HAL_Delay(100);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
					if(deg + 30 < 180){
						deg = deg + 30;
					}
					else if(deg == 180){
						deg = 0;
					}
					else{
						deg = 180;
					}
				}
				AD9959_SetPhase4Channel(0,0,0,deg);
				
				printf("t11.txt=\"%d deg \"\xff\xff\xff",deg);
			}
			
			if (CH455_KEY_NUM == 6){ // 调节衰减，需要等代码调试
				if (mode == 0){ // 减法模式
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
					HAL_Delay(500);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
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
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
					HAL_Delay(500);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
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

				
				printf("t9.txt=\"%d dB\"\xff\xff\xff",attenuation);
			}
			
			if (CH455_KEY_NUM == 7){ // 开启CW测量模式，直流偏置为1V
				uint8_t i;
				for (i = 0; i < 3; i++){
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
					HAL_Delay(100);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
					HAL_Delay(100);
				}
				mode = 2;
				dac_voltage = 1.0;
				
				printf("t12.txt=\"CW test \"\xff\xff\xff");
				printf("t13.txt=\"%d %% \"\xff\xff\xff",(int)(dac_voltage));
			}
			__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_13);
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
	uint32_t i = 0;
	uint32_t dac_value = 0; //dac code
  /* USER CODE END 1 */

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
	dac_value = (uint32_t)(dac_voltage/3.3f*4095.0f);//vref=3.3v
	HAL_DAC_Start(&hdac1,DAC_CHANNEL_1);//dac1 open
	HAL_DAC_SetValue(&hdac1,DAC_CHANNEL_1,DAC_ALIGN_12B_R,dac_value);//channel 1 output
	Init_AD9959();
	AD9959_SetFrequency4Channel(f_L,2000000,f_L,2000000);
	AD9959_SetAmp4Channel(vamp_L,vamp_A,vamp_L,vamp_A);
	AD9959_SetPhase4Channel(0,0,0,deg);
	
	printf("t12.txt=\"+ mode \"\xff\xff\xff");
	printf("t6.txt=\"%d \"\xff\xff\xff",vamp_L);
	printf("t7.txt=\"%d %% \"\xff\xff\xff",(int)(adjust*100.0+0.01));
	printf("t8.txt=\"%d  \"\xff\xff\xff",f_L);
	printf("t11.txt=\"%d deg \"\xff\xff\xff",deg);
	printf("t13.txt=\"%d %% \"\xff\xff\xff",(int)(dac_voltage));
	
  for (i = 0; i < 3; i = i + 1){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
		HAL_Delay(200);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
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
