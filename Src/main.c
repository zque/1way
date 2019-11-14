/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//USE_HAL_DRIVER, STM32H743xx,ARM_MATH_CM7,__CC_ARM,ARM_MATH_MATRIX_CHECK,ARM_MATH_ROUNDING  加入此行宏定义启用FFT算法
//#include "printf.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "sys.h"
#include "delay.h"
#include "arm_math.h"
#define FFT_LENGTH 		1024
#define adc2i					670							//adc转化成电流i的比值
#define bee				HAL_GPIO_To

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int i=0;							//for循环变量
int j=0;							//for循环变量
int k=0;							//for循环变量
int limit =200  ;						//设置报警突变电流
int sw=0;							//切换前后波形数据控制变量		
int count=0;
float amp_value=0.0;				//限制漏电电流转换为幅值有效值
int zero = 33220;
int reset4G =0;  //4g数据 接收超时重置标志
int connect_confirm =0; //定时发送消息确保连接
int alarm_message = 0;
int time_out=0;
int filter_len=5;
int limit_A =300;
int filter_index = 0;
int beep_flag=0;
int led_flag=0;
float adc1_error[1024];
float adc0_error[1024];
int har_error=0;
float cos_error=0;
float I1_error=0;
float I0_error=0;
int printf_flag=1;
uint8_t input_read ;

//struct {
//	int K1_Pin:1;
//	
//}sw_input;


uint8_t aRxBuffer2;			//接收中断缓冲
uint8_t Uart2_RxBuff[256];		//接收缓冲
uint8_t Uart2_Rx_Cnt = 0;		//接收缓冲计数

uint8_t aRxBuffer1;			//接收中断缓冲
uint8_t Uart3_RxBuff[256];		//接收缓冲
uint8_t Uart3_Rx_Cnt = 0;		//接收缓冲计数
uint8_t	cAlmStr[] = "数据溢出(大于256)\r\n";
								

int tim_count=0;
int start=0;						//第一次执行main函数 只采集了一个波形不做比较，用于第一次跳过比较
arm_cfft_radix4_instance_f32 scfft;

int filter1[5],filter0[5];
int adc1[1030],adc0[1030];
float avg1[1024],avg0[1024];
float cut1[400],cut0[400];
int p1=0,p0=0;
float A=0;
float B=0;
float AB=0;

float input1[2*FFT_LENGTH],input0[2*FFT_LENGTH];
float output1[FFT_LENGTH],output0[FFT_LENGTH];
float max1=0,max0=0;
float Imax1=0;
float Imax0=0;
float cos1=0;
float cos0=0;
int har=0;
int flag=0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_TIM1_Init(void);
static void MX_GPIO_Init(void);
static void MX_ADC3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void SET4G(void);
void sort(int* a,int len);
int get_ADC(ADC_HandleTypeDef adc);
void filter_A(int* a);
int abs(int a);
int filter_M(int *filter,int len);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_TIM1_Init();
  MX_GPIO_Init();
  MX_ADC3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  //HAL_ADCEx_Calibration_Start(&hadc1,ADC_CALIB_OFFSET,ADC_DIFFERENTIAL_ENDED);
  
 // HAL_TIM_Base_Start_IT(&htim1);
  delay_init(480);
  //HAL_GPIO_WritePin(RE_GPIO_Port,RE_Pin,GPIO_PIN_SET);
  HAL_UART_Receive_IT(&huart2, (uint8_t *)&aRxBuffer2,1);
  HAL_UART_Receive_IT(&huart1, (uint8_t *)&aRxBuffer1, 1);
  HAL_TIM_Base_Start_IT(&htim1);
  arm_cfft_radix4_init_f32(&scfft,FFT_LENGTH,0,1);
  delay_ms(1000);
	filter_index = filter_len/2;
  amp_value=(adc2i*limit);			//限制漏电电流转换为波形有效幅值
									//频谱幅值与波形有效值关系： 		波形有效值=频谱幅值*2/FFT_LEANGTH
									//波形有效值与漏电电流关系：		ADC测得电压=参考电压(3.3)/(ADC分辨率/2)*波形有效值
									//								互感器电流=ADC测得电压/负载电阻(内部负载电阻为70欧）
									//								漏电电流=互感器电流*1000(1000为互感器的线圈比1000：1）
									
			

	input_read = (GPIOH->IDR-->2)&0x0F;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	
    /* USER CODE BEGIN 3 */
		sw=!sw;
		
	




//**************************************ADC采样***************************************************//		
		if(sw){	for(i=0;i<1030;i++){for(k=0;k<filter_len;k++)filter1[k]=get_ADC(hadc3);
																adc1[i]=filter_M(filter1,filter_len);
																delay_us(20);}
						filter_A(adc1);
																		
		}else{for(i=0;i<1030;i++){for(k=0;k<filter_len;k++)filter1[k]=get_ADC(hadc3);
																adc1[i]=filter_M(filter1,filter_len);
																delay_us(20);}
						filter_A(adc1);
		}
		
		
	//*************************************滑动平均、cut、input***********************************************//
		if(sw){	for(i=0;i<1024;i++){input1[2*i]=avg1[i]=(adc1[i]+adc1[i+1]+adc1[i+2]+adc1[i+3]+adc1[i+4])/5.0;	input1[2*i+1]=0;}
						max1=0;
						for(i=0;i<200;i++){	if(avg1[i]>max1)max1=avg1[i];};	
						for(i=0;i<200;i++){	if(avg1[i]==max1)p1=i;}	
						for(i=0;i<400;i++){	cut1[i]=avg1[i+p1];}}
		else{for(i=0;i<1024;i++){	input0[2*i]=avg0[i]=(adc0[i]+adc0[i+1]+adc0[i+2]+adc0[i+3]+adc0[i+4])/5.0;	input0[2*i+1]=0;}
			
						max0=0;
						for(i=0;i<200;i++){	if(avg0[i]>max0)max0=avg0[i];};	
						for(i=0;i<200;i++){	if(avg0[i]==max0)p0=i;}
						for(i=0;i<400;i++){	cut0[i]=avg0[i+p0];}}
		
						
						
	//***************************************相似度计算*********************************************************//
												
		if(start){A=B=AB=0;
							for(i=0;i<400;i++){A+=(cut0[i]-zero)*(cut0[i]-zero);B+=(cut1[i]-zero)*(cut1[i]-zero);AB+=(cut0[i]-zero)*(cut1[i]-zero);}
							cos1=AB*AB/A/B;}
		
							
							
	//***************************************FFT计算************************************************************//
		if(sw){	arm_cfft_radix4_f32(&scfft,input1);
						arm_cmplx_mag_f32(input1,output1,FFT_LENGTH);
						Imax1=0;
						for(i=1;i<100;i++){	if(output1[i]>Imax1)Imax1=output1[i];}}
		else if(start){ 	
				arm_cfft_radix4_f32(&scfft,input0);
				arm_cmplx_mag_f32(input0,output0,FFT_LENGTH);
				Imax0=0;
				for(i=1;i<100;i++){	if(output0[i]>Imax0)Imax0=output0[i];}}
		
				
				
	//*****************************************************前后波形振幅比较******************//				
		if(start){if(sw){if((Imax1-Imax0)>amp_value)har=(int)(Imax1-Imax0);}
							else{	if((Imax0-Imax1)>amp_value)har=(int)(Imax0-Imax1);}}
		
							
							
							
	//***************************************************报警****************************************//
		if((cos1<0.95f)&&har){	flag=1;
						memcpy(adc0,avg0,sizeof(adc0));
						memcpy(adc1,avg1,sizeof(adc0));
						har_error=har;
						cos_error=cos1;
						I1_error=Imax1;
						I0_error=Imax0;
						printf("AT+CIPSEND=1,52,\"219.128.73.196\",20030\r\n");
						delay_ms(100);
						printf("漏电电流11：%05i\t漏电电流10：%05i\t突变电流1：%05i",(int)Imax1/adc2i,(int)Imax0/adc2i,(har/adc2i));
								har=0;}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Supply configuration update enable 
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source 
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 32;
  PeriphClkInitStruct.PLL2.PLL2N = 150;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Common config 
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc3.Init.Resolution = ADC_RESOLUTION_16B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 480;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED2_Pin|LED3_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SIM_RST_Pin|RE_Pin|BEEP_Pin|KM1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, STOP_Pin|FIN_Pin|TKOUT_Pin|M1_Pin 
                          |ON_Pin|OFF_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED2_Pin LED3_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|LED3_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : SIM_RST_Pin RE_Pin BEEP_Pin KM1_Pin */
  GPIO_InitStruct.Pin = SIM_RST_Pin|RE_Pin|BEEP_Pin|KM1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : STOP_Pin FIN_Pin TKOUT_Pin M1_Pin 
                           ON_Pin OFF_Pin */
  GPIO_InitStruct.Pin = STOP_Pin|FIN_Pin|TKOUT_Pin|M1_Pin 
                          |ON_Pin|OFF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_UART_TxCpltCallback could be implemented in the user file
   */
	if(huart->Instance == USART2){	if(Uart2_Rx_Cnt >= 255){
										Uart2_Rx_Cnt = 0;
										memset(Uart2_RxBuff,0x00,sizeof(Uart2_RxBuff));
										HAL_UART_Transmit(&huart2, (uint8_t *)&cAlmStr, sizeof(cAlmStr),0xFFFF);}
									else{
										Uart2_RxBuff[Uart2_Rx_Cnt++] = aRxBuffer2;   //接收数据转存
										if(aRxBuffer2==0x06)	time_out=0;
										if((Uart2_RxBuff[Uart2_Rx_Cnt-1] == 0x0A)&&(Uart2_RxBuff[Uart2_Rx_Cnt-2] == 0x0D)) //判断结束位
										{	
//											HAL_GPIO_WritePin(RE_GPIO_Port,RE_Pin,GPIO_PIN_SET);
//											HAL_UART_Transmit(&huart3, (uint8_t *)&Uart2_RxBuff, Uart2_Rx_Cnt,0xFFFF);//将串口2收到的信息发送到485串口
//											HAL_GPIO_WritePin(RE_GPIO_Port,RE_Pin,GPIO_PIN_RESET);
											if((Uart2_RxBuff[0] == 0x50)||(Uart2_RxBuff[1] == 0x42))SET4G();// 判断接收的消息为 PB DONE 时初始化4G模块
											Uart2_Rx_Cnt = 0;
											memset(Uart2_RxBuff,0x00,sizeof(Uart2_RxBuff)); //清空数组
										}
									}

									HAL_UART_Receive_IT(&huart2, (uint8_t *)&aRxBuffer2, 1);}   //再开启接收中断
	if(huart->Instance == USART3){if(Uart3_Rx_Cnt >= 255)  //溢出判断
									{
										Uart3_Rx_Cnt = 0;
										memset(Uart3_RxBuff,0x00,sizeof(Uart3_RxBuff));
										HAL_UART_Transmit(&huart2, (uint8_t *)&cAlmStr, sizeof(cAlmStr),0xFFFF);	
									}
									else
									{
										Uart3_RxBuff[Uart3_Rx_Cnt++] = aRxBuffer1;   //接收数据转存
											//HAL_GPIO_TogglePin(BEE_GPIO_Port,BEE_Pin);
										if(aRxBuffer1==0x06){	printf_flag=0;
																					HAL_GPIO_WritePin(RE_GPIO_Port,RE_Pin,GPIO_PIN_SET);
																					for (i=0;i<1024;i++){printf("%f\t%f\r\n",adc1_error[i],adc0_error[i]);}
																					printf("相似度：%f\t漏电值1：%f\t漏电值0：%f\t突变值：%i\r\n",cos0,I1_error/adc2i,I0_error/adc2i,har/adc2i);
																					HAL_GPIO_WritePin(RE_GPIO_Port,RE_Pin,GPIO_PIN_RESET);
																					printf_flag=1;
																					}
										if((Uart3_RxBuff[Uart3_Rx_Cnt-1] == 0x0A)&&(Uart3_RxBuff[Uart3_Rx_Cnt-2] == 0x0D)) //判断结束位
										{
//											HAL_UART_Transmit(&huart2, (uint8_t *)&Uart3_RxBuff, Uart3_Rx_Cnt,0xFFFF);//将485串口收到的信息发送到串口2
											Uart3_Rx_Cnt = 0;
											memset(Uart3_RxBuff,0x00,sizeof(Uart3_RxBuff)); //清空数组
										}
									}
									HAL_UART_Receive_IT(&huart1, (uint8_t *)&aRxBuffer1, 1);
									HAL_UART_Receive_IT(&huart2, (uint8_t *)&aRxBuffer2, 1);}
		
		
							
}
//**************************************printf*****************************//
int fputc(int ch, FILE *f)
{ 	if(printf_flag){
			while((USART2->ISR&0X40)==0);  
			USART2->TDR=(uint8_t)ch; } 
		else{while((USART3->ISR&0X40)==0);
				USART1->TDR=(uint8_t)ch;}
	return ch;
}

int abs(int a){if(a>=0)return a;else return -a;}
//********************************************限幅***************************************//
void filter_A(int * a){for(i=1;i<1020;i++){
			if(((a[i]-a[i-1])>limit_A && (a[i]-a[i+1])>limit_A)||((a[i-1]-a[i])>limit_A && (a[i+1]-a[i])>limit_A)) a[i]=a[i-1];
			if(((a[i]-a[i-1])>limit_A && (a[i+1]-a[i+2])>limit_A)||((a[i-1]-a[i])>limit_A && (a[i+2]-a[i+1])>limit_A)) a[i]=a[i+1]=a[i-1];
			if(((a[i]-a[i-1])>limit_A && (a[i+3]-a[i+2])>limit_A)||((a[i-1]-a[i])>limit_A && (a[i+2]-a[i+3])>limit_A)) a[i+2]=a[i]=a[i+1]=a[i-1];
}
		


}




//****************************************get  ADC***************************************//
int get_ADC(ADC_HandleTypeDef adc){
			HAL_ADC_Start(&adc);
			HAL_ADC_PollForConversion(&adc,0xffff);
			return HAL_ADC_GetValue(&adc);
}


//*********************************************中值**************************************//
int filter_M(int *filter,int len){
	sort(filter,len);
	return filter[len/2];
}
 
//******************************************排序************************************//
void sort(int* a,int len)
{
    int begin = 1;
    int i = 0;
    while(begin < len)
    {
        int key = a[begin];
        for(i = begin-1;i>=0;i--)
        {
            if(a[i]<=key)    
            {
                a[i+1] = key;
                break;
            }
            a[i+1] = a[i];
        }
        if(i<0)
            a[0] = key;
        begin++;
    }
}


void SET4G(void){
	delay_ms(300);
	printf("AT+CGSOCKCONT=1,\"IP\",\"CMNET\"\r\n");
	delay_ms(300);
	printf("AT+CSOCKSETPN=1\r\n");
	delay_ms(300);
	printf("AT+NETOPEN\r\n");
	delay_ms(300);
	printf("AT+CIPOPEN=1,\"UDP\",,,20030\r\n");
	delay_ms(300);
	printf("AT+CIPSEND=1,18,\"219.128.73.196\",20030\r\n");
	delay_ms(300);
	printf("4G模块初始化完成\r\n");
//	HAL_GPIO_TogglePin(BEE_GPIO_Port,BEE_Pin);
//	delay_ms(1000);
//	HAL_GPIO_TogglePin(BEE_GPIO_Port,BEE_Pin);
	
}
 
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	
	tim_count++;
	if(tim_count==1000){HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);tim_count=0;
						time_out++;
						connect_confirm++;
						led_flag=!led_flag;
						if(flag){	HAL_GPIO_WritePin(KM1_GPIO_Port,KM1_Pin,GPIO_PIN_SET);
											beep_flag=1;}
						
					
									
									}
						
						if(led_flag){if(flag)HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_SET);}
						else{ if(flag)HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_RESET);}			
						if(beep_flag)HAL_GPIO_TogglePin(BEEP_GPIO_Port,BEEP_Pin);
						
	
	if(!HAL_GPIO_ReadPin(M1_GPIO_Port,M1_Pin)){flag=0;HAL_GPIO_WritePin(KM1_GPIO_Port,KM1_Pin,GPIO_PIN_RESET);
																						HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_RESET);
																						beep_flag=0;HAL_GPIO_WritePin(BEEP_GPIO_Port,BEEP_Pin,GPIO_PIN_RESET);}			
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
