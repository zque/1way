///* USER CODE BEGIN Header */
///**
//  ******************************************************************************
//  * @file           : main.c
//  * @brief          : Main program body
//  ******************************************************************************
//  * @attention
//  *
//  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
//  * All rights reserved.</center></h2>
//  *
//  * This software component is licensed by ST under BSD 3-Clause license,
//  * the "License"; You may not use this file except in compliance with the
//  * License. You may obtain a copy of the License at:
//  *                        opensource.org/licenses/BSD-3-Clause
//  *
//  ******************************************************************************
//  */
///* USER CODE END Header */

///* Includes ------------------------------------------------------------------*/
//#include "main.h"

///* Private includes ----------------------------------------------------------*/
///* USER CODE BEGIN Includes */
////USE_HAL_DRIVER, STM32H743xx,ARM_MATH_CM7,__CC_ARM,ARM_MATH_MATRIX_CHECK,ARM_MATH_ROUNDING  加入此行宏定义启用FFT算法
////#include "printf.h"
//#include <stdio.h>
//#include <string.h>
//#include <math.h>
//#include "sys.h"
//#include "delay.h"
//#include "arm_math.h"
//#define FFT_LENGTH 		1024
//#define adc2i					670							//adc转化成电流i的比值
//#define bee				HAL_GPIO_To
///* USER CODE END Includes */

///* Private typedef -----------------------------------------------------------*/
///* USER CODE BEGIN PTD */

///* USER CODE END PTD */

///* Private define ------------------------------------------------------------*/
///* USER CODE BEGIN PD */
///* USER CODE END PD */

///* Private macro -------------------------------------------------------------*/
///* USER CODE BEGIN PM */

///* USER CODE END PM */

///* Private variables ---------------------------------------------------------*/
//ADC_HandleTypeDef hadc1;
//ADC_HandleTypeDef hadc2;
//ADC_HandleTypeDef hadc3;

//TIM_HandleTypeDef htim1;

//UART_HandleTypeDef huart2;
//UART_HandleTypeDef huart3;

///* USER CODE BEGIN PV */
///* Private variables ---------------------------------------------------------*/
//int i=0;							//for循环变量
//int j=0;							//for循环变量
//int k=0;							//for循环变量
//int limit =200  ;						//设置报警突变电流
//int sw=0;							//切换前后波形数据控制变量		
//int count=0;
//float amp_value=0.0;				//限制漏电电流转换为幅值有效值
//float A=0.0;
//float B=0.0;
//float AB=0.0;
//int zero = 33220;
//int reset4G =0;  //4g数据 接收超时重置标志
//int connect_confirm =0; //定时发送消息确保连接
//int alarm_message = 0;
//int time_out=0;
//int filter_len=5;
//int limit_A =300;
//int filter_index = 0;
//int beep_flag=0;
//int led_flag=0;
//float adc1[1024];
//float adc0[1024];
//int har=0;
//float cos0=0;
//float I1=0;
//float I0=0;
//int printf_flag=1;
//uint8_t input_read ;

////struct {
////	int K1_Pin:1;
////	
////}sw_input;


//uint8_t aRxBuffer2;			//接收中断缓冲
//uint8_t Uart2_RxBuff[256];		//接收缓冲
//uint8_t Uart2_Rx_Cnt = 0;		//接收缓冲计数

//uint8_t aRxBuffer3;			//接收中断缓冲
//uint8_t Uart3_RxBuff[256];		//接收缓冲
//uint8_t Uart3_Rx_Cnt = 0;		//接收缓冲计数
//uint8_t	cAlmStr[] = "数据溢出(大于256)\r\n";
//								

//int tim_count=0;
//int start=0;						//第一次执行main函数 只采集了一个波形不做比较，用于第一次跳过比较
//arm_cfft_radix4_instance_f32 scfft;
////**********************************************ADC**********************//
//int filter11[5],filter21[5],filter31[5],filter41[5],filter51[5],filter61[5],filter71[5],filter81[5];
//int filter10[5],filter20[5],filter30[5],filter40[5],filter50[5],filter60[5],filter70[5],filter80[5];

//int adc11[1030];//原始波形		ADC:1	通道2
//int adc21[1030];//原始波形		ADC:1	通道3
//int adc31[1030];//原始波形		ADC:1	通道4
//int adc41[1030];//原始波形		ADC:1	通道5
//int adc51[1030];//原始波形		ADC:2	通道2
//int adc61[1030];//原始波形		ADC:3	通道1
//int adc71[1030];//原始波形		ADC:3	通道2
//int adc81[1030];//原始波形		ADC:3	通道3



//int adc10[1030];//原始波形		ADC:1	通道2
//int adc20[1030];//原始波形		ADC:1	通道3
//int adc30[1030];//原始波形		ADC:1	通道4
//int adc40[1030];//原始波形		ADC:1	通道5
//int adc50[1030];//原始波形		ADC:2	通道2
//int adc60[1030];//原始波形		ADC:3	通道1
//int adc70[1030];//原始波形		ADC:3	通道2
//int adc80[1030];//原始波形		ADC:3	通道3



////********************************************滑动平均*******************//
//float avg10[1024];
//float avg20[1024];
//float avg30[1024];
//float avg40[1024];
//float avg50[1024];
//float avg60[1024];
//float avg70[1024];
//float avg80[1024];

//float avg11[1024];
//float avg21[1024];
//float avg31[1024];
//float avg41[1024];
//float avg51[1024];
//float avg61[1024];
//float avg71[1024];
//float avg81[1024];


////********************************************报警标志har****************//
//int har1=0;
//int har2=0;
//int har3=0;
//int har4=0;
//int har5=0;
//int har6=0;
//int har7=0;
//int har8=0;
//int har9=0;

////******************************************FFT输入、输出input，output******************//
//float input10[FFT_LENGTH*2];
//float input20[FFT_LENGTH*2];
//float input30[FFT_LENGTH*2];
//float input40[FFT_LENGTH*2];
//float input50[FFT_LENGTH*2];
//float input60[FFT_LENGTH*2];
//float input70[FFT_LENGTH*2];
//float input80[FFT_LENGTH*2];

//float input11[FFT_LENGTH*2];
//float input21[FFT_LENGTH*2];
//float input31[FFT_LENGTH*2];
//float input41[FFT_LENGTH*2];
//float input51[FFT_LENGTH*2];
//float input61[FFT_LENGTH*2];
//float input71[FFT_LENGTH*2];
//float input81[FFT_LENGTH*2];



//float output10[FFT_LENGTH];
//float output20[FFT_LENGTH];
//float output30[FFT_LENGTH];
//float output40[FFT_LENGTH];
//float output50[FFT_LENGTH];
//float output60[FFT_LENGTH];
//float output70[FFT_LENGTH];
//float output80[FFT_LENGTH];

//float output11[FFT_LENGTH];
//float output21[FFT_LENGTH];
//float output31[FFT_LENGTH];
//float output41[FFT_LENGTH];
//float output51[FFT_LENGTH];
//float output61[FFT_LENGTH];
//float output71[FFT_LENGTH];
//float output81[FFT_LENGTH];
////********************************************频谱振幅最大值*******************************//
//float max11=0;
//float max21=0;
//float max31=0;
//float max41=0;
//float max51=0;
//float max61=0;
//float max71=0;
//float max81=0;

//float max10=0;
//float max20=0;
//float max30=0;
//float max40=0;
//float max50=0;
//float max60=0;
//float max70=0;
//float max80=0;

//float Imax11=0;
//float Imax21=0;
//float Imax31=0;
//float Imax41=0;
//float Imax51=0;
//float Imax61=0;
//float Imax71=0;
//float Imax81=0;

//float Imax10=0;
//float Imax20=0;
//float Imax30=0;
//float Imax40=0;
//float Imax50=0;
//float Imax60=0;
//float Imax70=0;
//float Imax80=0;

//float cut11[400],cut21[400],cut31[400],cut41[400],cut51[400],cut61[400],cut71[400],cut81[400];
//float cut10[400],cut20[400],cut30[400],cut40[400],cut50[400],cut60[400],cut70[400],cut80[400];
//int p11,p21,p31,p41,p51,p61,p71,p81=0;
//int p10,p20,p30,p40,p50,p60,p70,p80=0;

//float A1,A2,A3,A4,A5,A6,A7,A8=0;
//float B1,B2,B3,B4,B5,B6,B7,B8=0;
//float AB1,AB2,AB3,AB4,AB5,AB6,AB7,AB8=0;
//float cos1,cos2,cos3,cos4,cos5,cos6,cos7,cos8=0;
//int flag1,flag2,flag3,flag4,flag5,flag6,flag7,flag8=0;



///* USER CODE END PV */

///* Private function prototypes -----------------------------------------------*/
//void SystemClock_Config(void);
//static void MX_GPIO_Init(void);
//static void MX_ADC1_Init(void);
//static void MX_ADC2_Init(void);
//static void MX_ADC3_Init(void);
//static void MX_TIM1_Init(void);
//static void MX_USART2_UART_Init(void);
//static void MX_USART3_UART_Init(void);
///* USER CODE BEGIN PFP */

///* USER CODE END PFP */

///* Private user code ---------------------------------------------------------*/
///* USER CODE BEGIN 0 */
//void SET4G(void);
//void sort(int* a,int len);
//int get_ADC(ADC_HandleTypeDef adc);
//void filter_A(int* a);
//int abs(int a);
//int filter_M(int *filter,int len);
///* USER CODE END 0 */

///**
//  * @brief  The application entry point.
//  * @retval int
//  */
//int main(void)
//{
//  /* USER CODE BEGIN 1 */

//  /* USER CODE END 1 */
//  

//  /* Enable I-Cache---------------------------------------------------------*/
//  SCB_EnableICache();

//  /* Enable D-Cache---------------------------------------------------------*/
//  SCB_EnableDCache();

//  /* MCU Configuration--------------------------------------------------------*/

//  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//  HAL_Init();

//  /* USER CODE BEGIN Init */

//  /* USER CODE END Init */

//  /* Configure the system clock */
//  SystemClock_Config();

//  /* USER CODE BEGIN SysInit */

//  /* USER CODE END SysInit */

//  /* Initialize all configured peripherals */
//  MX_GPIO_Init();
//  MX_ADC1_Init();
//  MX_ADC2_Init();
//  MX_ADC3_Init();
//  MX_TIM1_Init();
//  MX_USART2_UART_Init();
//  MX_USART3_UART_Init();
//  /* USER CODE BEGIN 2 */
//  //HAL_ADCEx_Calibration_Start(&hadc1,ADC_CALIB_OFFSET,ADC_DIFFERENTIAL_ENDED);
//  
// // HAL_TIM_Base_Start_IT(&htim1);
//  delay_init(400);
//  //HAL_GPIO_WritePin(RE_GPIO_Port,RE_Pin,GPIO_PIN_SET);
//  HAL_UART_Receive_IT(&huart2, (uint8_t *)&aRxBuffer2,1);
//  HAL_UART_Receive_IT(&huart3, (uint8_t *)&aRxBuffer3, 1);
//  HAL_TIM_Base_Start_IT(&htim1);
//  arm_cfft_radix4_init_f32(&scfft,FFT_LENGTH,0,1);
//  delay_ms(1000);
//	filter_index = filter_len/2;
//  amp_value=(adc2i*limit);			//限制漏电电流转换为波形有效幅值
//									//频谱幅值与波形有效值关系： 		波形有效值=频谱幅值*2/FFT_LEANGTH
//									//波形有效值与漏电电流关系：		ADC测得电压=参考电压(3.3)/(ADC分辨率/2)*波形有效值
//									//								互感器电流=ADC测得电压/负载电阻(内部负载电阻为70欧）
//									//								漏电电流=互感器电流*1000(1000为互感器的线圈比1000：1）
//									
//			

//	input_read = (GPIOH->IDR-->2)&0x0F;
// /* USER CODE END 2 */

//  /* Infinite loop */
//  /* USER CODE BEGIN WHILE */
//  while (1)
//  {

//    /* USER CODE END WHILE */

//    /* USER CODE BEGIN 3 */
////********************************************************************ADC测试*****************************************************************//
//	
////									HAL_ADC_Start(&hadc1);
////									HAL_ADC_PollForConversion(&hadc1,0xffff);
////									adc11[1]= HAL_ADC_GetValue(&hadc1);
////									HAL_ADC_Start(&hadc1);
////									HAL_ADC_PollForConversion(&hadc1,0xffff);
////									adc21[1]= HAL_ADC_GetValue(&hadc1);
////									HAL_ADC_Start(&hadc1);
////									HAL_ADC_PollForConversion(&hadc1,0xffff);
////									adc31[1]= HAL_ADC_GetValue(&hadc1);
////									HAL_ADC_Start(&hadc1);
////									HAL_ADC_PollForConversion(&hadc1,0xffff);
////									adc41[1]= HAL_ADC_GetValue(&hadc1);
////					
////									HAL_ADC_Start(&hadc2);
////									HAL_ADC_PollForConversion(&hadc2,0xffff);
////									adc51[1]= HAL_ADC_GetValue(&hadc2);
////									
////									HAL_ADC_Start(&hadc3);
////									HAL_ADC_PollForConversion(&hadc3,0xffff);
////									adc61[1]= HAL_ADC_GetValue(&hadc3);
////									HAL_ADC_Start(&hadc3);	
////									HAL_ADC_PollForConversion(&hadc3,0xffff);
////									adc71[1]= HAL_ADC_GetValue(&hadc3);
////									HAL_ADC_Start(&hadc3);	
////									HAL_ADC_PollForConversion(&hadc3,0xffff);
////									adc81[1]= HAL_ADC_GetValue(&hadc3);
////									//delay_ms(1);
////								printf("%i\t%i\t%i\t%i\t%i\t%i\t%i\t%i\r\n",adc11[1],adc21[1],adc31[1],adc41[1],adc51[1],adc61[1],adc71[1],adc81[1]);

//									
////********************************************************************led、BEEP测试*************************************************************//
////		while(1){delay_ms(1000);
////				HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
////				HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
////				HAL_GPIO_TogglePin(LED3_GPIO_Port,LED3_Pin);
////				HAL_GPIO_TogglePin(LED4_GPIO_Port,LED4_Pin);
////				HAL_GPIO_TogglePin(LED5_GPIO_Port,LED5_Pin);
////				HAL_GPIO_TogglePin(LED6_GPIO_Port,LED6_Pin);
////				HAL_GPIO_TogglePin(LED7_GPIO_Port,LED7_Pin);
////				HAL_GPIO_TogglePin(LED8_GPIO_Port,LED8_Pin);
////				HAL_GPIO_TogglePin(LED9_GPIO_Port,LED9_Pin);
////				HAL_GPIO_TogglePin(LED10_GPIO_Port,LED10_Pin);
////			
////				HAL_GPIO_TogglePin(BEE_GPIO_Port,BEE_Pin);
////				

////			
////		}

////*******************************************************************继电器测试*************************************************//
//		
////			HAL_GPIO_TogglePin(KM1_GPIO_Port,KM1_Pin);
////		delay_ms(1000);
////			HAL_GPIO_TogglePin(KM2_GPIO_Port,KM2_Pin);
////		delay_ms(1000);
////			HAL_GPIO_TogglePin(KM2_GPIO_Port,KM2_Pin);
////		delay_ms(1000);
////			HAL_GPIO_TogglePin(KM3_GPIO_Port,KM3_Pin);
////		delay_ms(1000);
////			HAL_GPIO_TogglePin(KM4_GPIO_Port,KM4_Pin);
////		delay_ms(1000);
////			HAL_GPIO_TogglePin(KM5_GPIO_Port,KM5_Pin);
////		delay_ms(1000);
////			HAL_GPIO_TogglePin(KM6_GPIO_Port,KM6_Pin);
////		delay_ms(1000);
////			HAL_GPIO_TogglePin(KM7_GPIO_Port,KM7_Pin);
////		delay_ms(1000);
////			HAL_GPIO_TogglePin(KM8_GPIO_Port,KM8_Pin);
////			delay_ms(10000);
////			

//			
//		
//	  
//	  
//	  
//	  
//	  
//		sw=!sw;
////		//***************************************ADC采样*************************************************//		
////		if(sw){	for(i=0;i<1030;i++){for(k=0;k<filter_len;k++){
////																								HAL_ADC_Start(&hadc1);
////																								HAL_ADC_PollForConversion(&hadc1,0xffff);
////																								filter11[k]= HAL_ADC_GetValue(&hadc1);
////																								HAL_ADC_Start(&hadc1);
////																								HAL_ADC_PollForConversion(&hadc1,0xffff);
////																								filter21[k]= HAL_ADC_GetValue(&hadc1);
////																								HAL_ADC_Start(&hadc1);
////																								HAL_ADC_PollForConversion(&hadc1,0xffff);
////																								filter31[k]= HAL_ADC_GetValue(&hadc1);
////																								HAL_ADC_Start(&hadc1);
////																								HAL_ADC_PollForConversion(&hadc1,0xffff);
////																								filter41[k]= HAL_ADC_GetValue(&hadc1);
////													
////																								HAL_ADC_Start(&hadc2);
////																								HAL_ADC_PollForConversion(&hadc2,0xffff);
////																								filter51[k]= HAL_ADC_GetValue(&hadc2);
////																
////																								HAL_ADC_Start(&hadc3);
////																								HAL_ADC_PollForConversion(&hadc3,0xffff);
////																								filter61[k]= HAL_ADC_GetValue(&hadc3);
////																								HAL_ADC_Start(&hadc3);	
////																								HAL_ADC_PollForConversion(&hadc3,0xffff);
////																								filter71[k]= HAL_ADC_GetValue(&hadc3);
////																								HAL_ADC_Start(&hadc3);	
////																								HAL_ADC_PollForConversion(&hadc3,0xffff);
////																								filter81[k]= HAL_ADC_GetValue(&hadc3);}
////																//printf("**********1***********\r\n");
////																sort(filter11,filter_len);
////																sort(filter21,filter_len);								
////																sort(filter31,filter_len);								
////																sort(filter41,filter_len);								
////																sort(filter51,filter_len);								
////																sort(filter61,filter_len);								
////																sort(filter71,filter_len);								
////																sort(filter81,filter_len);
////																adc11[i]=filter11[filter_index];
////																adc21[i]=filter21[filter_index];									
////																adc31[i]=filter31[filter_index];								
////																adc41[i]=filter41[filter_index];								
////																adc51[i]=filter51[filter_index];								
////																adc61[i]=filter61[filter_index];								
////																adc71[i]=filter71[filter_index];								
////																adc81[i]=filter81[filter_index];
////																delay_us(20);
////				}}
////		else{for(i=0;i<1030;i++){for(k=0;k<filter_len;k++){HAL_ADC_Start(&hadc1);
////																							HAL_ADC_PollForConversion(&hadc1,0xffff);
////																							filter10[k]= HAL_ADC_GetValue(&hadc1);
////																							HAL_ADC_Start(&hadc1);
////																							HAL_ADC_PollForConversion(&hadc1,0xffff);
////																							filter20[k]= HAL_ADC_GetValue(&hadc1);
////																							HAL_ADC_Start(&hadc1);
////																							HAL_ADC_PollForConversion(&hadc1,0xffff);
////																							filter30[k]= HAL_ADC_GetValue(&hadc1);
////																							HAL_ADC_Start(&hadc1);
////																							HAL_ADC_PollForConversion(&hadc1,0xffff);
////																							filter40[k]= HAL_ADC_GetValue(&hadc1);
////									
////																							HAL_ADC_Start(&hadc2);
////																							HAL_ADC_PollForConversion(&hadc2,0xffff);
////																							filter50[k]= HAL_ADC_GetValue(&hadc2);
////									
////																							HAL_ADC_Start(&hadc3);
////																							HAL_ADC_PollForConversion(&hadc3,0xffff);
////																							filter60[k]= HAL_ADC_GetValue(&hadc3);
////																							HAL_ADC_Start(&hadc3);
////																							HAL_ADC_PollForConversion(&hadc3,0xffff);
////																							filter70[k]= HAL_ADC_GetValue(&hadc3);
////																							HAL_ADC_Start(&hadc3);
////																							HAL_ADC_PollForConversion(&hadc3,0xffff);
////																							filter80[k]= HAL_ADC_GetValue(&hadc3);}

////																sort(filter10,filter_len);
////																sort(filter20,filter_len);								
////																sort(filter30,filter_len);								
////																sort(filter40,filter_len);								
////																sort(filter50,filter_len);								
////																sort(filter60,filter_len);								
////																sort(filter70,filter_len);								
////																sort(filter80,filter_len);
////																adc10[i]=filter10[filter_index];
////																adc20[i]=filter20[filter_index];									
////																adc30[i]=filter30[filter_index];								
////																adc40[i]=filter40[filter_index];								
////																adc50[i]=filter50[filter_index];								
////																adc60[i]=filter60[filter_index];								
////																adc70[i]=filter70[filter_index];								
////																adc80[i]=filter80[filter_index];	
////																delay_us(20);
////			}}

////			
////       if(sw){printf("%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\r\n",Imax11/adc2i,Imax21/adc2i,Imax31/adc2i,Imax41/adc2i,Imax51/adc2i,Imax61/adc2i,Imax71/adc2i,Imax81/adc2i);}			
////									if(sw){	printf("*****************片段1*********************");   //测试采集到的adc数据
////											for(i=0;i<1024;i++){printf("%i\r\n",adc11[i]);}}	
//////									else{	printf("*****************片段0*********************");
////											for(i=0;i<1024;i++){printf("%i\r\n",adc10[i]);}}
//											
////**************************************************************ADC采样2************************************************************//		
//		if(sw){	for(i=0;i<1030;i++){for(k=0;k<filter_len;k++){filter11[k]= get_ADC(hadc1);
//																													filter21[k]= get_ADC(hadc1);
//																													filter31[k]= get_ADC(hadc1);
//																													filter41[k]= get_ADC(hadc1);
//																													filter51[k]= get_ADC(hadc2);
//																													filter61[k]= get_ADC(hadc3);
//																													filter71[k]= get_ADC(hadc3);
//																													filter81[k]= get_ADC(hadc3);
//																													//printf("**%i\r\n",filter81[k]);
//																														}
//																adc11[i]=filter_M(filter11,filter_len);
//																adc21[i]=filter_M(filter21,filter_len);	
//																adc31[i]=filter_M(filter31,filter_len);
//																adc41[i]=filter_M(filter41,filter_len);		
//																adc51[i]=filter_M(filter51,filter_len);
//																adc61[i]=filter_M(filter61,filter_len);
//																adc71[i]=filter_M(filter71,filter_len);
//																adc81[i]=filter_M(filter81,filter_len);
//																//printf("***%i\r\n",adc81[i]);
//																delay_us(20);}
//					filter_A(adc11);
//					filter_A(adc21);
//					filter_A(adc31);				
//					filter_A(adc41);	
//					filter_A(adc51);
//					filter_A(adc61);
//					filter_A(adc71);
//					filter_A(adc81);
//																
//																
//																
//		}else{for(i=0;i<1030;i++){for(k=0;k<filter_len;k++){	filter10[k]= get_ADC(hadc1);
//																													filter20[k]= get_ADC(hadc1);
//																													filter30[k]= get_ADC(hadc1);
//																													filter40[k]= get_ADC(hadc1);
//																													filter50[k]= get_ADC(hadc2);
//																													filter60[k]= get_ADC(hadc3);
//																													filter70[k]= get_ADC(hadc3);
//																													filter80[k]= get_ADC(hadc3);	}
//																adc10[i]=filter_M(filter10,filter_len);
//																adc20[i]=filter_M(filter20,filter_len);	
//																adc30[i]=filter_M(filter30,filter_len);
//																adc40[i]=filter_M(filter40,filter_len);		
//																adc50[i]=filter_M(filter50,filter_len);
//																adc60[i]=filter_M(filter60,filter_len);
//																adc70[i]=filter_M(filter70,filter_len);
//																adc80[i]=filter_M(filter80,filter_len);
//																delay_us(20);}
//					filter_A(adc10);
//					filter_A(adc20);
//					filter_A(adc30);				
//					filter_A(adc40);	
//					filter_A(adc50);
//					filter_A(adc60);
//					filter_A(adc70);
//					filter_A(adc80);
//		}
//		
////		if(sw){printf("**********************1**********************\r\n");
////					for (i=0;i<1024;i++)printf("%i\r\n",adc11[i]);}
////		else {printf("**********************0**********************\r\n");
////					for (i=0;i<1024;i++)printf("%i\r\n",adc10[i]);}
//					

//					//	printf("***************************************\r\n");
////						if(flag2){for(i=0;i<1024;i++)printf("%i\t%i\t%i\t%i\t%i\t%i\t%i\t%i\r\n",
////																			adc20[i],adc21[i],adc31[i],adc41[i],adc51[i],adc61[i],adc71[i],adc81[i]);}			
////		




//											
////											
////		if(sw){	for(i=0;i<1030;i++){for(j=0;j<4;j++){HAL_ADC_Start(&hadc1);
////									HAL_ADC_PollForConversion(&hadc1,0xffff);
////									adc11[i]= HAL_ADC_GetValue(&hadc1);
////									HAL_ADC_Start(&hadc1);
////									HAL_ADC_PollForConversion(&hadc1,0xffff);
////									adc21[i]= HAL_ADC_GetValue(&hadc1);
////									HAL_ADC_Start(&hadc1);
////									HAL_ADC_PollForConversion(&hadc1,0xffff);
////									adc31[i]= HAL_ADC_GetValue(&hadc1);
////									HAL_ADC_Start(&hadc1);
////									HAL_ADC_PollForConversion(&hadc1,0xffff);
////									adc41[i]= HAL_ADC_GetValue(&hadc1);}
////					
////									HAL_ADC_Start(&hadc2);
////									HAL_ADC_PollForConversion(&hadc2,0xffff);
////									adc51[i]= HAL_ADC_GetValue(&hadc2);
////									
////									for(j=0;j<3;j++){HAL_ADC_Start(&hadc3);
////									HAL_ADC_PollForConversion(&hadc3,0xffff);
////									adc61[i]= HAL_ADC_GetValue(&hadc3);
////									HAL_ADC_Start(&hadc3);	
////									HAL_ADC_PollForConversion(&hadc3,0xffff);
////									adc71[i]= HAL_ADC_GetValue(&hadc3);
////									HAL_ADC_Start(&hadc3);	
////									HAL_ADC_PollForConversion(&hadc3,0xffff);
////									adc81[i]= HAL_ADC_GetValue(&hadc3);}
////									delay_us(150);}
////		
////						filter(adc11,avg11);
////				}
////		else{for(i=0;i<1030;i++){HAL_ADC_Start(&hadc1);
////									HAL_ADC_PollForConversion(&hadc1,0xffff);
////									adc10[i]= HAL_ADC_GetValue(&hadc1);
////									HAL_ADC_Start(&hadc1);
////									HAL_ADC_PollForConversion(&hadc1,0xffff);
////									adc20[i]= HAL_ADC_GetValue(&hadc1);
////									HAL_ADC_Start(&hadc1);
////									HAL_ADC_PollForConversion(&hadc1,0xffff);
////									adc30[i]= HAL_ADC_GetValue(&hadc1);
////									HAL_ADC_Start(&hadc1);
////									HAL_ADC_PollForConversion(&hadc1,0xffff);
////									adc40[i]= HAL_ADC_GetValue(&hadc1);
////									
////									HAL_ADC_Start(&hadc2);
////									HAL_ADC_PollForConversion(&hadc2,0xffff);
////									adc50[i]= HAL_ADC_GetValue(&hadc2);
////									
////									HAL_ADC_Start(&hadc3);
////									HAL_ADC_PollForConversion(&hadc3,0xffff);
////									adc60[i]= HAL_ADC_GetValue(&hadc3);
////									HAL_ADC_Start(&hadc3);
////									HAL_ADC_PollForConversion(&hadc3,0xffff);
////									adc70[i]= HAL_ADC_GetValue(&hadc3);
////									HAL_ADC_Start(&hadc3);
////									HAL_ADC_PollForConversion(&hadc3,0xffff);
////									adc80[i]= HAL_ADC_GetValue(&hadc3);	
////									delay_us(150);
////									}}
////		
////							
////				
////									
//////****************************************************adc取滑动平均并放入input数组*********************************************************//		


//		if(sw){	for(i=0;i<1024;i++){input11[2*i]=avg11[i]=(adc11[i]+adc11[i+1]+adc11[i+2]+adc11[i+3]+adc11[i+4])/5.0;	input11[2*i+1]=0;
//																								//printf("%f",adc10[i]);
//									input21[2*i]=avg21[i]=(adc21[i]+adc21[i+1]+adc21[i+2]+adc21[i+3]+adc21[i+4])/5.0; input21[2*i+1]=0;
//									input31[2*i]=avg31[i]=(adc31[i]+adc31[i+1]+adc31[i+2]+adc31[i+3]+adc31[i+4])/5.0;	input31[2*i+1]=0;
//									input41[2*i]=avg41[i]=(adc41[i]+adc41[i+1]+adc41[i+2]+adc41[i+3]+adc41[i+4])/5.0;	input41[2*i+1]=0;
//									input51[2*i]=avg51[i]=(adc51[i]+adc51[i+1]+adc51[i+2]+adc51[i+3]+adc51[i+4])/5.0;	input51[2*i+1]=0;
//									input61[2*i]=avg61[i]=(adc61[i]+adc61[i+1]+adc61[i+2]+adc61[i+3]+adc61[i+4])/5.0;	input61[2*i+1]=0;
//									input71[2*i]=avg71[i]=(adc71[i]+adc71[i+1]+adc71[i+2]+adc71[i+3]+adc71[i+4])/5.0;	input71[2*i+1]=0;
//									input81[2*i]=avg81[i]=(adc81[i]+adc81[i+1]+adc81[i+2]+adc81[i+3]+adc81[i+4])/5.0;	input81[2*i+1]=0;}
//			
//				max11=max21=max31=max41=max51=max61=max71=max81=0;
//				for(i=0;i<200;i++){	if(avg11[i]>max11){max11=avg11[i];}
//									if(avg21[i]>max21){max21=avg21[i];}
//									if(avg31[i]>max31){max31=avg31[i];}
//									if(avg41[i]>max41){max41=avg41[i];}
//									if(avg51[i]>max51){max51=avg51[i];}	
//									if(avg61[i]>max61){max61=avg61[i];}
//									if(avg71[i]>max71){max71=avg71[i];}	
//									if(avg81[i]>max81){max81=avg81[i];}};	
//				for(i=0;i<200;i++){	if(avg11[i]==max11)p11=i;
//									if(avg21[i]==max21)p21=i;
//									if(avg31[i]==max31)p31=i;
//									if(avg41[i]==max41)p41=i;
//									if(avg51[i]==max51)p51=i;
//									if(avg61[i]==max61)p61=i;
//									if(avg71[i]==max71)p71=i;
//									if(avg81[i]==max81)p81=i;}	
//				for(i=0;i<400;i++){	cut11[i]=avg11[i+p11];
//									cut21[i]=avg21[i+p21];
//									cut31[i]=avg31[i+p31];
//									cut41[i]=avg41[i+p41];
//									cut51[i]=avg51[i+p51];
//									cut61[i]=avg61[i+p61];	
//									cut71[i]=avg71[i+p71];
//									cut81[i]=avg81[i+p81];
//									//printf("%f\r\n",cut11[i]);
//									}}
//		
//		else{for(i=0;i<1024;i++){	input10[2*i]=avg10[i]=(adc10[i]+adc10[i+1]+adc10[i+2]+adc10[i+3]+adc10[i+4])/5.0;	input10[2*i+1]=0;	
//																						//	printf("%i",adc10[i]);
//									input20[2*i]=avg20[i]=(adc20[i]+adc20[i+1]+adc20[i+2]+adc20[i+3]+adc20[i+4])/5.0;	input20[2*i+1]=0;
//									input30[2*i]=avg30[i]=(adc30[i]+adc30[i+1]+adc30[i+2]+adc30[i+3]+adc30[i+4])/5.0;	input30[2*i+1]=0;
//									input40[2*i]=avg40[i]=(adc40[i]+adc40[i+1]+adc40[i+2]+adc40[i+3]+adc40[i+4])/5.0;	input40[2*i+1]=0;
//									input50[2*i]=avg50[i]=(adc50[i]+adc50[i+1]+adc50[i+2]+adc50[i+3]+adc50[i+4])/5.0;	input50[2*i+1]=0;
//									input60[2*i]=avg60[i]=(adc60[i]+adc60[i+1]+adc60[i+2]+adc60[i+3]+adc60[i+4])/5.0;	input60[2*i+1]=0;
//									input70[2*i]=avg70[i]=(adc70[i]+adc70[i+1]+adc70[i+2]+adc70[i+3]+adc70[i+4])/5.0;	input70[2*i+1]=0;
//									input80[2*i]=avg80[i]=(adc80[i]+adc80[i+0]+adc80[i+2]+adc80[i+3]+adc80[i+4])/5.0;	input80[2*i+1]=0;}
//			
//									max10=max20=max30=max40=max50=max60=max70=max80=0;
//				for(i=0;i<200;i++){	if(avg10[i]>max10){max10=avg10[i];}
//									if(avg20[i]>max20){max20=avg20[i];}
//									if(avg30[i]>max30){max30=avg30[i];}
//									if(avg40[i]>max40){max40=avg40[i];}
//									if(avg50[i]>max50){max50=avg50[i];}	
//									if(avg60[i]>max60){max60=avg60[i];}
//									if(avg70[i]>max70){max70=avg70[i];}	
//									if(avg80[i]>max80){max80=avg80[i];}};	
//				for(i=0;i<200;i++){	if(avg10[i]==max10)p10=i;
//									if(avg20[i]==max20)p20=i;
//									if(avg30[i]==max30)p30=i;
//									if(avg40[i]==max40)p40=i;
//									if(avg50[i]==max50)p50=i;
//									if(avg60[i]==max60)p60=i;
//									if(avg70[i]==max70)p70=i;
//									if(avg80[i]==max80)p80=i;}
//				for(i=0;i<400;i++){	cut10[i]=avg10[i+p10];
//									cut20[i]=avg20[i+p20];
//									cut30[i]=avg30[i+p30];
//									cut40[i]=avg40[i+p40];
//									cut50[i]=avg50[i+p50];
//									cut60[i]=avg60[i+p60];	
//									cut70[i]=avg70[i+p70];
//									cut80[i]=avg80[i+p80];
//									//printf("%f\r\n",cut11[i]);
//									}}
////		if(sw){			printf("***************************************input********************\r\n");		//input测试
////						for(i=0;i<1024;i++)printf("%f\r\n",input11[2*i]);	}
////		else if(start){	printf("**************************************input0********************\r\n");
////						for(i=0;i<1024;i++)printf("%f\r\n",input10[2*i]);}
////*********************************************************相似度计算**********************************************************//
//		A1=A2=A3=A4=A5=A6=A7=A8=B1=B2=B3=B4=B5=B6=B7=B8=AB1=AB2=AB3=AB4=AB5=AB6=AB7=AB8=0;								
//		if(start){for(i=0;i<400;i++){	
//							A1+=(cut10[i]-zero)*(cut10[i]-zero);B1+=(cut11[i]-zero)*(cut11[i]-zero);AB1+=(cut10[i]-zero)*(cut11[i]-zero);
//							A2+=(cut20[i]-zero)*(cut20[i]-zero);B2+=(cut21[i]-zero)*(cut21[i]-zero);AB2+=(cut20[i]-zero)*(cut21[i]-zero);
//							A3+=(cut30[i]-zero)*(cut30[i]-zero);B3+=(cut31[i]-zero)*(cut31[i]-zero);AB3+=(cut30[i]-zero)*(cut31[i]-zero);
//							A4+=(cut40[i]-zero)*(cut40[i]-zero);B4+=(cut41[i]-zero)*(cut41[i]-zero);AB4+=(cut40[i]-zero)*(cut41[i]-zero);
//							A5+=(cut50[i]-zero)*(cut50[i]-zero);B5+=(cut51[i]-zero)*(cut51[i]-zero);AB5+=(cut50[i]-zero)*(cut51[i]-zero);
//							A6+=(cut60[i]-zero)*(cut60[i]-zero);B6+=(cut61[i]-zero)*(cut61[i]-zero);AB6+=(cut60[i]-zero)*(cut61[i]-zero);
//							A7+=(cut70[i]-zero)*(cut70[i]-zero);B7+=(cut71[i]-zero)*(cut71[i]-zero);AB7+=(cut60[i]-zero)*(cut71[i]-zero);
//							A8+=(cut80[i]-zero)*(cut80[i]-zero);B8+=(cut81[i]-zero)*(cut81[i]-zero);AB8+=(cut80[i]-zero)*(cut81[i]-zero);}}
//						
//		if(start){	cos1=AB1*AB1/A1/B1;
//					cos2=AB2*AB2/A2/B2;
//					cos3=AB3*AB3/A3/B3;
//					cos4=AB4*AB4/A4/B4;
//					cos5=AB5*AB5/A5/B5;
//					cos6=AB6*AB6/A6/B6;	
//					cos7=AB7*AB7/A7/B7;	
//					cos8=AB8*AB8/A8/B8;	}
//			
////		if(sw){			printf("***************************************input********************\r\n");		//input测试
////						for(i=0;i<1024;i++)printf("%f\r\n",input11[2*i]);	}
////		else if(start){	printf("**************************************input0********************\r\n");
////						for(i=0;i<1024;i++)printf("%f\r\n",input10[2*i]);}
//									
//									
////****************************************************对input进行FFT变换******************************************************//
//			
//		if(sw){	arm_cfft_radix4_f32(&scfft,input11);
//				arm_cmplx_mag_f32(input11,output11,FFT_LENGTH);
//				arm_cfft_radix4_f32(&scfft,input21);
//				arm_cmplx_mag_f32(input21,output21,FFT_LENGTH);
//				arm_cfft_radix4_f32(&scfft,input31);
//				arm_cmplx_mag_f32(input31,output31,FFT_LENGTH);				
//				arm_cfft_radix4_f32(&scfft,input41);
//				arm_cmplx_mag_f32(input41,output41,FFT_LENGTH);
//				arm_cfft_radix4_f32(&scfft,input51);
//				arm_cmplx_mag_f32(input51,output51,FFT_LENGTH);
//				arm_cfft_radix4_f32(&scfft,input61);
//				arm_cmplx_mag_f32(input61,output61,FFT_LENGTH);
//				arm_cfft_radix4_f32(&scfft,input71);
//				arm_cmplx_mag_f32(input71,output71,FFT_LENGTH);
//				arm_cfft_radix4_f32(&scfft,input81);
//				arm_cmplx_mag_f32(input81,output81,FFT_LENGTH);}
//		
//		else if(start){ 	
//				arm_cfft_radix4_f32(&scfft,input10);
//				arm_cmplx_mag_f32(input10,output10,FFT_LENGTH);
//				arm_cfft_radix4_f32(&scfft,input20);
//				arm_cmplx_mag_f32(input20,output20,FFT_LENGTH);
//				arm_cfft_radix4_f32(&scfft,input30);
//				arm_cmplx_mag_f32(input30,output30,FFT_LENGTH);				
//				arm_cfft_radix4_f32(&scfft,input40);
//				arm_cmplx_mag_f32(input40,output40,FFT_LENGTH);
//				arm_cfft_radix4_f32(&scfft,input50);
//				arm_cmplx_mag_f32(input50,output50,FFT_LENGTH);
//				arm_cfft_radix4_f32(&scfft,input60);
//				arm_cmplx_mag_f32(input60,output60,FFT_LENGTH);
//				arm_cfft_radix4_f32(&scfft,input70);
//				arm_cmplx_mag_f32(input70,output70,FFT_LENGTH);
//				arm_cfft_radix4_f32(&scfft,input80);
//				arm_cmplx_mag_f32(input80,output80,FFT_LENGTH);}
//					
////		if(sw){	printf("**************************output1***********************\r\n");   //output 测试
////				for(i=1;i<100;i++)printf("%f\r\n",output21[i]);}
////				
////		else if(start){ 
////				printf("**************************output0***********************\r\n");
////				for(i=1;i<100;i++)printf("%f\r\n",output20[i]);}			
//////****************************************************频谱取最大值**************************************//			
//			if(sw){	Imax11=Imax21=Imax31=Imax41=Imax51=Imax61=Imax71=Imax81=0;
//					for(i=1;i<100;i++){	if(output11[i]>Imax11)Imax11=output11[i];
//										if(output21[i]>Imax21)Imax21=output21[i];
//										if(output31[i]>Imax31)Imax31=output31[i];
//										if(output41[i]>Imax41)Imax41=output41[i];
//										if(output51[i]>Imax51)Imax51=output51[i];
//										if(output61[i]>Imax61)Imax61=output61[i];
//										if(output71[i]>Imax71)Imax71=output71[i];
//										if(output81[i]>Imax81)Imax81=output81[i];}}
//			else if(start){	Imax10=Imax20=Imax30=Imax40=Imax50=Imax60=Imax70=Imax80=0;
//					for(i=1;i<100;i++){	if(output10[i]>Imax10)Imax10=output10[i];
//												if(output20[i]>Imax20)Imax20=output20[i];
//												if(output30[i]>Imax30)Imax30=output30[i];
//												if(output40[i]>Imax40)Imax40=output40[i];
//												if(output50[i]>Imax50)Imax50=output50[i];
//												if(output60[i]>Imax60)Imax60=output60[i];
//												if(output70[i]>Imax70)Imax70=output70[i];
//												if(output80[i]>Imax80)Imax80=output80[i];}}
//												
//				
////*****************************************************前后波形振幅比较******************//				
//		if(start){if(sw){if((Imax11-Imax10)>amp_value)har1=(int)(Imax11-Imax10);
//						if((Imax21-Imax20)>amp_value)har2=(int)(Imax21-Imax20);
//						if((Imax31-Imax30)>amp_value)har3=(int)(Imax31-Imax30);
//						if((Imax41-Imax40)>amp_value)har4=(int)(Imax41-Imax40);
//						if((Imax51-Imax50)>amp_value)har5=(int)(Imax51-Imax50);
//						if((Imax61-Imax60)>amp_value)har6=(int)(Imax61-Imax60);	
//						if((Imax71-Imax70)>amp_value)har7=(int)(Imax71-Imax70);
//						if((Imax81-Imax80)>amp_value)har8=(int)(Imax81-Imax80);}
//					else{	
//						if((Imax10-Imax11)>amp_value)har1=(int)(Imax10-Imax11);
//						if((Imax20-Imax21)>amp_value)har2=(int)(Imax20-Imax21);
//						if((Imax30-Imax31)>amp_value)har3=(int)(Imax30-Imax31);
//						if((Imax40-Imax41)>amp_value)har4=(int)(Imax40-Imax41);
//						if((Imax50-Imax51)>amp_value)har5=(int)(Imax50-Imax51);
//						if((Imax60-Imax61)>amp_value)har6=(int)(Imax60-Imax61);	
//						if((Imax70-Imax71)>amp_value)har7=(int)(Imax70-Imax71);
//						if((Imax80-Imax81)>amp_value)har8=(int)(Imax80-Imax81);}}
//		
////		if(start){if(fabs(Imax11-Imax10)>amp_value)har1=(int)fabs(Imax11-Imax10);
////							if(fabs(Imax21-Imax20)>amp_value)har2=(int)fabs(Imax21-Imax20);
////							if(fabs(Imax31-Imax30)>amp_value)har3=(int)fabs(Imax31-Imax30);
////							if(fabs(Imax41-Imax40)>amp_value)har4=(int)fabs(Imax41-Imax40);
////							if(fabs(Imax51-Imax50)>amp_value)har5=(int)fabs(Imax51-Imax50);
////							if(fabs(Imax61-Imax60)>amp_value)har6=(int)fabs(Imax61-Imax60);
////							if(fabs(Imax71-Imax70)>amp_value)har7=(int)fabs(Imax71-Imax70);
////							if(fabs(Imax81-Imax80)>amp_value)har8=(int)fabs(Imax81-Imax80);}
//							
//	
////		if(sw)	{	printf("漏电电流1：%fmA\t",Imax11/adc2i);printf("突变电流：%fmA\r\n",(Imax11-Imax10)/adc2i);}
////		else if(start) 
////				{printf("漏电电流0：%fmA\t",Imax10/adc2i);printf("突变电流：%fmA\r\n",(Imax10-Imax11)/adc2i);}
////				printf("%i\r\n",(int)HAL_GPIO_ReadPin(K1_GPIO_Port,K1_Pin));
//		
////		if(sw){printf("%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\r\n",Imax11/adc2i,Imax21/adc2i,Imax31/adc2i,Imax41/adc2i,Imax51/adc2i,Imax61/adc2i,Imax71/adc2i,Imax81/adc2i);}
//				
////		if(sw)	{	printf("漏电电流1：%fmA\t%fmA\t%fmA\t%fmA\t%fmA\t%fmA\t%fmA\t%fmA\r\n",Imax11/adc2i,Imax21/adc2i,Imax31/adc2i,Imax41/adc2i,Imax51/adc2i,Imax61/adc2i,Imax71/adc2i,Imax81/adc2i);
////					printf("突变电流：%fmA\t%fmA\t%fmA\t%fmA\t%fmA\t%fmA\t%fmA\t%fmA\r\n",(Imax11-Imax10)/adc2i,(Imax21-Imax20)/adc2i,(Imax31-Imax30)/adc2i,(Imax41-Imax40)/adc2i,(Imax51-Imax50)/adc2i,(Imax61-Imax60)/adc2i,(Imax71-Imax70)/adc2i,(Imax81-Imax80)/adc2i);}
////		else if(start) 
////				{	printf("漏电电流1：%fmA\t%fmA\t%fmA\t%fmA\t%fmA\t%fmA\t%fmA\t%fmA\r\n",Imax11/adc2i,Imax21/adc2i,Imax31/adc2i,Imax41/adc2i,Imax51/adc2i,Imax61/adc2i,Imax71/adc2i,Imax81/adc2i);
////					printf("突变电流：%fmA\t%fmA\t%fmA\t%fmA\t%fmA\t%fmA\t%fmA\t%fmA\r\n",(Imax10-Imax11)/adc2i,(Imax20-Imax21)/adc2i,(Imax30-Imax31)/adc2i,(Imax40-Imax41)/adc2i,(Imax50-Imax51)/adc2i,(Imax60-Imax61)/adc2i,(Imax70-Imax71)/adc2i,(Imax80-Imax81)/adc2i);}
////		
////		if(har1){	printf("**************************频谱12************************\r\n");
////					for(i=1;i<100;i++)printf("%f\t%f\r\n",output11[i],output10[i]);
////				}
//		if((cos1<0.95f)&&har1){	flag1=1;
//								memcpy(adc0,avg10,sizeof(adc0));
//								memcpy(adc1,avg11,sizeof(adc0));
//								har=har1;
//								cos0=cos2;
//								I1=Imax11;
//								I0=Imax10;
//								printf("AT+CIPSEND=1,52,\"219.128.73.196\",20030\r\n");
//								delay_ms(100);
//								printf("漏电电流11：%05i\t漏电电流10：%05i\t突变电流1：%05i",(int)Imax11/adc2i,(int)Imax10/adc2i,(har1/adc2i));
//								har1=0;}
//		if((cos2<0.9f)&&har2){	flag2=1;
//								memcpy(adc0,avg20,sizeof(adc0));
//								memcpy(adc1,avg21,sizeof(adc0));
//								har=har2;
//								cos0=cos2;
//								I1=Imax21;
//								I0=Imax20;
//								printf("AT+CIPSEND=1,52,\"219.128.73.196\",20030\r\n");
//								delay_ms(100);
//								printf("漏电电流21：%05i\t漏电电流20：%05i\t突变电流2：%05i",(int)Imax21/adc2i,(int)Imax20/adc2i,har2/adc2i);
//								har2=0;}
//									
//		if((cos3<0.9f)&&har3){	flag3=1;
//								memcpy(adc0,avg30,sizeof(adc0));
//								memcpy(adc1,avg31,sizeof(adc0));
//								har=har3;
//								cos0=cos2;
//								I1=Imax31;
//								I0=Imax30;
//								printf("AT+CIPSEND=1,52,\"219.128.73.196\",20030\r\n");
//								delay_ms(100);
//								printf("漏电电流31：%05i\t漏电电流30：%05i\t突变电流3：%05i",(int)Imax31/adc2i,(int)Imax30/adc2i,har3/adc2i);
//								har3=0;}
//		
//		if((cos4<0.9f)&&har4){	flag4=1;
//								memcpy(adc0,avg40,sizeof(adc0));
//								memcpy(adc1,avg41,sizeof(adc0));
//								har=har4;
//								cos0=cos2;
//								I1=Imax41;
//								I0=Imax40;
//								printf("AT+CIPSEND=1,52,\"219.128.73.196\",20030\r\n");
//								delay_ms(100);
//								printf("漏电电流41：%05i\t漏电电流40：%05i\t突变电流4：%05i",(int)Imax41/adc2i,(int)Imax40/adc2i,har4/adc2i);
//								har4=0;}
//							
//		if((cos5<0.9f)&&har5){	flag5=1;
//								memcpy(adc0,avg50,sizeof(adc0));
//								memcpy(adc1,avg51,sizeof(adc0));
//								har=har5;
//								cos0=cos2;
//								I1=Imax51;
//								I0=Imax50;
//								printf("AT+CIPSEND=1,52,\"219.128.73.196\",20030\r\n");
//								delay_ms(100);
//								printf("漏电电流51：%05i\t漏电电流50：%05i\t突变电流5：%05i",(int)Imax51/adc2i,(int)Imax50/adc2i,har5/adc2i);
//								har5=0;}
//							
//		if((cos6<0.9f)&&har6){	flag6=1;
//								memcpy(adc0,avg60,sizeof(adc0));
//								memcpy(adc1,avg61,sizeof(adc0));
//								har=har6;
//								cos0=cos2;
//								I1=Imax61;
//								I0=Imax60;
//								printf("AT+CIPSEND=1,52,\"219.128.73.196\",20030\r\n");
//								delay_ms(100);
//								printf("漏电电流61：%05i\t漏电电流60：%05i\t突变电流6：%05i",(int)Imax61/adc2i,(int)Imax60/adc2i,har6/adc2i);
//								har6=0;}
//								
//		if((cos7<0.9f)&&har7){	flag7=1;
//								memcpy(adc0,avg70,sizeof(adc0));
//								memcpy(adc1,avg71,sizeof(adc0));
//								har=har7;
//								cos0=cos2;
//								I1=Imax71;
//								I0=Imax70;
//								printf("AT+CIPSEND=1,52,\"219.128.73.196\",20030\r\n");
//								delay_ms(100);
//								printf("漏电电流71：%05i\t漏电电流70：%05i\t突变电流7：%05i",(int)Imax71/adc2i,(int)Imax70/adc2i,har7/adc2i);
//								har7=0;}
//								
//		if((cos8<0.9f)&&har8){	flag8=1;
//								memcpy(adc0,avg80,sizeof(adc0));
//								memcpy(adc1,avg81,sizeof(adc0));
//								har=har8;
//								cos0=cos2;
//								I1=Imax81;
//								I0=Imax80;
//								printf("AT+CIPSEND=1,52,\"219.128.73.196\",20030\r\n");
//								delay_ms(100);
//								printf("漏电电流81：%05i\t漏电电流80：%05i\t突变电流8：%05i",(int)Imax81/adc2i,(int)Imax80/adc2i,har8/adc2i);
//								har8=0;}
//		
//								
//								
//												
//		if(connect_confirm >30){printf("AT+CIPSEND=1,15,\"219.128.73.196\",20030\r\n");										//30秒发送一次数据
//								delay_ms(100);
//								printf("%i\t%i\t%i\t%i\t%i\t%i\t%i\t%i",flag1,flag2,flag3,flag4,flag5,flag6,flag7,flag8);
//								connect_confirm=0;}
//		if(time_out>120){//HAL_GPIO_WritePin(RE_GPIO_Port,RE_Pin,GPIO_PIN_SET);												//超时（120秒）时重置4G模块
//						delay_ms(100);
//						HAL_GPIO_WritePin(reset_4G_GPIO_Port,reset_4G_Pin,GPIO_PIN_SET);
//						delay_ms(100);
//						HAL_GPIO_WritePin(reset_4G_GPIO_Port,reset_4G_Pin,GPIO_PIN_RESET);
//						SET4G();
//						//HAL_GPIO_WritePin(RE_GPIO_Port,RE_Pin,GPIO_PIN_RESET);
//						time_out=0;}		
//					
////						
////				if(1){for(i=0;i<1024;i++)printf("%i\t%i\t%i\t%i\t%i\t%i\t%i\t%i\r\n",
////																			adc11[i],adc21[i],adc31[i],adc41[i],adc51[i],adc61[i],adc71[i],adc81[i]);}			

//					if(sw){printf("%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\r\n",Imax11/adc2i,Imax21/adc2i,Imax31/adc2i,Imax41/adc2i,Imax51/adc2i,Imax61/adc2i,Imax71/adc2i,Imax81/adc2i);}			
//					else{printf("%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\r\n",Imax10/adc2i,Imax20/adc2i,Imax30/adc2i,Imax40/adc2i,Imax50/adc2i,Imax60/adc2i,Imax70/adc2i,Imax80/adc2i);}
//						
////																						printf_flag=0;
////																						HAL_GPIO_WritePin(RE_GPIO_Port,RE_Pin,GPIO_PIN_SET);
////																						for (i=0;i<1024;i++){printf("%i\t%i\r\n",adc1[i],adc0[i]);}
////																						HAL_GPIO_WritePin(RE_GPIO_Port,RE_Pin,GPIO_PIN_RESET);
////																						printf_flag=1;
//			
//		start=1;
//	} 
//  /* USER CODE END 3 */
//}

///**
//  * @brief System Clock Configuration
//  * @retval None
//  */
//void SystemClock_Config(void)
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

//  /** Supply configuration update enable 
//  */
//  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
//  /** Configure the main internal regulator output voltage 
//  */
//  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

//  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
//  /** Macro to configure the PLL clock source 
//  */
//  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
//  /** Initializes the CPU, AHB and APB busses clocks 
//  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//  RCC_OscInitStruct.PLL.PLLM = 2;
//  RCC_OscInitStruct.PLL.PLLN = 50;
//  RCC_OscInitStruct.PLL.PLLP = 2;
//  RCC_OscInitStruct.PLL.PLLQ = 2;
//  RCC_OscInitStruct.PLL.PLLR = 2;
//  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
//  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
//  RCC_OscInitStruct.PLL.PLLFRACN = 0;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Initializes the CPU, AHB and APB busses clocks 
//  */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
//                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
//  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
//  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_USART2
//                              |RCC_PERIPHCLK_ADC;
//  PeriphClkInitStruct.PLL2.PLL2M = 2;
//  PeriphClkInitStruct.PLL2.PLL2N = 10;
//  PeriphClkInitStruct.PLL2.PLL2P = 1;
//  PeriphClkInitStruct.PLL2.PLL2Q = 2;
//  PeriphClkInitStruct.PLL2.PLL2R = 2;
//  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
//  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
//  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
//  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
//  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
//  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}

///**
//  * @brief ADC1 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_ADC1_Init(void)
//{

//  /* USER CODE BEGIN ADC1_Init 0 */

//  /* USER CODE END ADC1_Init 0 */

//  ADC_MultiModeTypeDef multimode = {0};
//  ADC_ChannelConfTypeDef sConfig = {0};

//  /* USER CODE BEGIN ADC1_Init 1 */

//  /* USER CODE END ADC1_Init 1 */
//  /** Common config 
//  */
//  hadc1.Instance = ADC1;
//  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV6;
//  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
//  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
//  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
//  hadc1.Init.LowPowerAutoWait = DISABLE;
//  hadc1.Init.ContinuousConvMode = DISABLE;
//  hadc1.Init.NbrOfConversion = 4;
//  hadc1.Init.DiscontinuousConvMode = ENABLE;
//  hadc1.Init.NbrOfDiscConversion = 1;
//  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
//  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
//  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
//  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
//  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
//  hadc1.Init.OversamplingMode = DISABLE;
//  if (HAL_ADC_Init(&hadc1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure the ADC multi-mode 
//  */
//  multimode.Mode = ADC_MODE_INDEPENDENT;
//  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure Regular Channel 
//  */
//  sConfig.Channel = ADC_CHANNEL_2;
//  sConfig.Rank = ADC_REGULAR_RANK_1;
//  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
//  sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
//  sConfig.OffsetNumber = ADC_OFFSET_1;
//  sConfig.Offset = 0;
//  sConfig.OffsetRightShift = DISABLE;
//  sConfig.OffsetSignedSaturation = DISABLE;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure Regular Channel 
//  */
//  sConfig.Channel = ADC_CHANNEL_3;
//  sConfig.Rank = ADC_REGULAR_RANK_2;
//  sConfig.OffsetNumber = ADC_OFFSET_NONE;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure Regular Channel 
//  */
//  sConfig.Channel = ADC_CHANNEL_4;
//  sConfig.Rank = ADC_REGULAR_RANK_3;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure Regular Channel 
//  */
//  sConfig.Channel = ADC_CHANNEL_5;
//  sConfig.Rank = ADC_REGULAR_RANK_4;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN ADC1_Init 2 */

//  /* USER CODE END ADC1_Init 2 */

//}

///**
//  * @brief ADC2 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_ADC2_Init(void)
//{

//  /* USER CODE BEGIN ADC2_Init 0 */

//  /* USER CODE END ADC2_Init 0 */

//  ADC_ChannelConfTypeDef sConfig = {0};

//  /* USER CODE BEGIN ADC2_Init 1 */

//  /* USER CODE END ADC2_Init 1 */
//  /** Common config 
//  */
//  hadc2.Instance = ADC2;
//  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV6;
//  hadc2.Init.Resolution = ADC_RESOLUTION_16B;
//  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
//  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
//  hadc2.Init.LowPowerAutoWait = DISABLE;
//  hadc2.Init.ContinuousConvMode = DISABLE;
//  hadc2.Init.NbrOfConversion = 1;
//  hadc2.Init.DiscontinuousConvMode = DISABLE;
//  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
//  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
//  hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
//  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
//  hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
//  hadc2.Init.OversamplingMode = DISABLE;
//  if (HAL_ADC_Init(&hadc2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure Regular Channel 
//  */
//  sConfig.Channel = ADC_CHANNEL_2;
//  sConfig.Rank = ADC_REGULAR_RANK_1;
//  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
//  sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
//  sConfig.OffsetNumber = ADC_OFFSET_NONE;
//  sConfig.Offset = 0;
//  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN ADC2_Init 2 */

//  /* USER CODE END ADC2_Init 2 */

//}

///**
//  * @brief ADC3 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_ADC3_Init(void)
//{

//  /* USER CODE BEGIN ADC3_Init 0 */

//  /* USER CODE END ADC3_Init 0 */

//  ADC_ChannelConfTypeDef sConfig = {0};

//  /* USER CODE BEGIN ADC3_Init 1 */

//  /* USER CODE END ADC3_Init 1 */
//  /** Common config 
//  */
//  hadc3.Instance = ADC3;
//  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV6;
//  hadc3.Init.Resolution = ADC_RESOLUTION_16B;
//  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
//  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
//  hadc3.Init.LowPowerAutoWait = DISABLE;
//  hadc3.Init.ContinuousConvMode = DISABLE;
//  hadc3.Init.NbrOfConversion = 3;
//  hadc3.Init.DiscontinuousConvMode = ENABLE;
//  hadc3.Init.NbrOfDiscConversion = 1;
//  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
//  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
//  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
//  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
//  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
//  hadc3.Init.OversamplingMode = DISABLE;
//  if (HAL_ADC_Init(&hadc3) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure Regular Channel 
//  */
//  sConfig.Channel = ADC_CHANNEL_1;
//  sConfig.Rank = ADC_REGULAR_RANK_1;
//  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
//  sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
//  sConfig.OffsetNumber = ADC_OFFSET_NONE;
//  sConfig.Offset = 0;
//  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure Regular Channel 
//  */
//  sConfig.Channel = ADC_CHANNEL_2;
//  sConfig.Rank = ADC_REGULAR_RANK_2;
//  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure Regular Channel 
//  */
//  sConfig.Channel = ADC_CHANNEL_3;
//  sConfig.Rank = ADC_REGULAR_RANK_3;
//  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN ADC3_Init 2 */

//  /* USER CODE END ADC3_Init 2 */

//}

///**
//  * @brief TIM1 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_TIM1_Init(void)
//{

//  /* USER CODE BEGIN TIM1_Init 0 */

//  /* USER CODE END TIM1_Init 0 */

//  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//  TIM_MasterConfigTypeDef sMasterConfig = {0};

//  /* USER CODE BEGIN TIM1_Init 1 */

//  /* USER CODE END TIM1_Init 1 */
//  htim1.Instance = TIM1;
//  htim1.Init.Prescaler = 200;
//  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim1.Init.Period = 1000;
//  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim1.Init.RepetitionCounter = 0;
//  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM1_Init 2 */

//  /* USER CODE END TIM1_Init 2 */

//}

///**
//  * @brief USART2 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_USART2_UART_Init(void)
//{

//  /* USER CODE BEGIN USART2_Init 0 */

//  /* USER CODE END USART2_Init 0 */

//  /* USER CODE BEGIN USART2_Init 1 */

//  /* USER CODE END USART2_Init 1 */
//  huart2.Instance = USART2;
//  huart2.Init.BaudRate = 115200;
//  huart2.Init.WordLength = UART_WORDLENGTH_8B;
//  huart2.Init.StopBits = UART_STOPBITS_1;
//  huart2.Init.Parity = UART_PARITY_NONE;
//  huart2.Init.Mode = UART_MODE_TX_RX;
//  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
//  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
//  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
//  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
//  if (HAL_UART_Init(&huart2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN USART2_Init 2 */

//  /* USER CODE END USART2_Init 2 */

//}

///**
//  * @brief USART3 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_USART3_UART_Init(void)
//{

//  /* USER CODE BEGIN USART3_Init 0 */

//  /* USER CODE END USART3_Init 0 */

//  /* USER CODE BEGIN USART3_Init 1 */

//  /* USER CODE END USART3_Init 1 */
//  huart3.Instance = USART3;
//  huart3.Init.BaudRate = 115200;
//  huart3.Init.WordLength = UART_WORDLENGTH_8B;
//  huart3.Init.StopBits = UART_STOPBITS_1;
//  huart3.Init.Parity = UART_PARITY_NONE;
//  huart3.Init.Mode = UART_MODE_TX_RX;
//  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
//  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
//  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
//  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
//  if (HAL_UART_Init(&huart3) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN USART3_Init 2 */

//  /* USER CODE END USART3_Init 2 */

//}

///**
//  * @brief GPIO Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_GPIO_Init(void)
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};

//  /* GPIO Ports Clock Enable */
//  __HAL_RCC_GPIOE_CLK_ENABLE();
//  __HAL_RCC_GPIOC_CLK_ENABLE();
//  __HAL_RCC_GPIOF_CLK_ENABLE();
//  __HAL_RCC_GPIOH_CLK_ENABLE();
//  __HAL_RCC_GPIOA_CLK_ENABLE();
//  __HAL_RCC_GPIOB_CLK_ENABLE();
//  __HAL_RCC_GPIOI_CLK_ENABLE();

//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOE, LED1_Pin|LED2_Pin|reset_4G_Pin, GPIO_PIN_RESET);

//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOH, BEE_Pin|RE_Pin, GPIO_PIN_RESET);

//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOB, LED8_Pin|LED9_Pin|LED10_Pin|LED3_Pin 
//                          |LED4_Pin|LED5_Pin|LED6_Pin|LED7_Pin, GPIO_PIN_RESET);

//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOI, KM1_Pin|KM2_Pin|KM3_Pin|KM4_Pin 
//                          |KM5_Pin|KM6_Pin|KM7_Pin|KM8_Pin, GPIO_PIN_RESET);

//  /*Configure GPIO pins : LED1_Pin LED2_Pin reset_4G_Pin */
//  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|reset_4G_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

//  /*Configure GPIO pins : K1_Pin K3_Pin K2_Pin K4_Pin 
//                           K5_Pin K6_Pin K7_Pin K8_Pin */
//  GPIO_InitStruct.Pin = K1_Pin|K3_Pin|K2_Pin|K4_Pin 
//                          |K5_Pin|K6_Pin|K7_Pin|K8_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

//  /*Configure GPIO pins : BEE_Pin RE_Pin */
//  GPIO_InitStruct.Pin = BEE_Pin|RE_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

//  /*Configure GPIO pins : LED8_Pin LED9_Pin LED10_Pin LED3_Pin 
//                           LED4_Pin LED5_Pin LED6_Pin LED7_Pin */
//  GPIO_InitStruct.Pin = LED8_Pin|LED9_Pin|LED10_Pin|LED3_Pin 
//                          |LED4_Pin|LED5_Pin|LED6_Pin|LED7_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

//  /*Configure GPIO pins : KM1_Pin KM2_Pin KM3_Pin KM4_Pin 
//                           KM5_Pin KM6_Pin KM7_Pin KM8_Pin */
//  GPIO_InitStruct.Pin = KM1_Pin|KM2_Pin|KM3_Pin|KM4_Pin 
//                          |KM5_Pin|KM6_Pin|KM7_Pin|KM8_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

//}

///* USER CODE BEGIN 4 */
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//  /* Prevent unused argument(s) compilation warning */
//  UNUSED(huart);
//  /* NOTE: This function Should not be modified, when the callback is needed,
//           the HAL_UART_TxCpltCallback could be implemented in the user file
//   */
//	if(huart->Instance == USART2){	if(Uart2_Rx_Cnt >= 255){
//										Uart2_Rx_Cnt = 0;
//										memset(Uart2_RxBuff,0x00,sizeof(Uart2_RxBuff));
//										HAL_UART_Transmit(&huart2, (uint8_t *)&cAlmStr, sizeof(cAlmStr),0xFFFF);}
//									else{
//										Uart2_RxBuff[Uart2_Rx_Cnt++] = aRxBuffer2;   //接收数据转存
//										if(aRxBuffer2==0x06)	time_out=0;
//										if((Uart2_RxBuff[Uart2_Rx_Cnt-1] == 0x0A)&&(Uart2_RxBuff[Uart2_Rx_Cnt-2] == 0x0D)) //判断结束位
//										{	
////											HAL_GPIO_WritePin(RE_GPIO_Port,RE_Pin,GPIO_PIN_SET);
////											HAL_UART_Transmit(&huart3, (uint8_t *)&Uart2_RxBuff, Uart2_Rx_Cnt,0xFFFF);//将串口2收到的信息发送到485串口
////											HAL_GPIO_WritePin(RE_GPIO_Port,RE_Pin,GPIO_PIN_RESET);
//											if((Uart2_RxBuff[0] == 0x50)||(Uart2_RxBuff[1] == 0x42))SET4G();// 判断接收的消息为 PB DONE 时初始化4G模块
//											Uart2_Rx_Cnt = 0;
//											memset(Uart2_RxBuff,0x00,sizeof(Uart2_RxBuff)); //清空数组
//										}
//									}

//									HAL_UART_Receive_IT(&huart2, (uint8_t *)&aRxBuffer2, 1);}   //再开启接收中断
//	if(huart->Instance == USART3){if(Uart3_Rx_Cnt >= 255)  //溢出判断
//									{
//										Uart3_Rx_Cnt = 0;
//										memset(Uart3_RxBuff,0x00,sizeof(Uart3_RxBuff));
//										HAL_UART_Transmit(&huart2, (uint8_t *)&cAlmStr, sizeof(cAlmStr),0xFFFF);	
//									}
//									else
//									{
//										Uart3_RxBuff[Uart3_Rx_Cnt++] = aRxBuffer3;   //接收数据转存
//											//HAL_GPIO_TogglePin(BEE_GPIO_Port,BEE_Pin);
//										if(aRxBuffer3==0x06){	printf_flag=0;
//																					HAL_GPIO_WritePin(RE_GPIO_Port,RE_Pin,GPIO_PIN_SET);
//																					for (i=0;i<1024;i++){printf("%f\t%f\r\n",adc1[i],adc0[i]);}
//																					printf("相似度：%f\t漏电值1：%f\t漏电值0：%f\t突变值：%i\r\n",cos0,I1/adc2i,I0/adc2i,har/adc2i);
//																					HAL_GPIO_WritePin(RE_GPIO_Port,RE_Pin,GPIO_PIN_RESET);
//																					printf_flag=1;
//																					}
//										if((Uart3_RxBuff[Uart3_Rx_Cnt-1] == 0x0A)&&(Uart3_RxBuff[Uart3_Rx_Cnt-2] == 0x0D)) //判断结束位
//										{
////											HAL_UART_Transmit(&huart2, (uint8_t *)&Uart3_RxBuff, Uart3_Rx_Cnt,0xFFFF);//将485串口收到的信息发送到串口2
//											Uart3_Rx_Cnt = 0;
//											memset(Uart3_RxBuff,0x00,sizeof(Uart3_RxBuff)); //清空数组
//										}
//									}
//									HAL_UART_Receive_IT(&huart3, (uint8_t *)&aRxBuffer3, 1);
//									HAL_UART_Receive_IT(&huart2, (uint8_t *)&aRxBuffer2, 1);}
//		
//		
//							
//}
////**************************************printf*****************************//
//int fputc(int ch, FILE *f)
//{ 	if(printf_flag){
//			while((USART2->ISR&0X40)==0);  
//			USART2->TDR=(uint8_t)ch; } 
//		else{while((USART3->ISR&0X40)==0);
//				USART3->TDR=(uint8_t)ch;}
//	return ch;
//}

//int abs(int a){if(a>=0)return a;else return -a;}
////********************************************限幅***************************************//
//void filter_A(int * a){for(i=1;i<1020;i++){
//			if(((a[i]-a[i-1])>limit_A && (a[i]-a[i+1])>limit_A)||((a[i-1]-a[i])>limit_A && (a[i+1]-a[i])>limit_A)) a[i]=a[i-1];
//			if(((a[i]-a[i-1])>limit_A && (a[i+1]-a[i+2])>limit_A)||((a[i-1]-a[i])>limit_A && (a[i+2]-a[i+1])>limit_A)) a[i]=a[i+1]=a[i-1];
//			if(((a[i]-a[i-1])>limit_A && (a[i+3]-a[i+2])>limit_A)||((a[i-1]-a[i])>limit_A && (a[i+2]-a[i+3])>limit_A)) a[i+2]=a[i]=a[i+1]=a[i-1];
//}
//		


//}




////****************************************get  ADC***************************************//
//int get_ADC(ADC_HandleTypeDef adc){
//			HAL_ADC_Start(&adc);
//			HAL_ADC_PollForConversion(&adc,0xffff);
//			return HAL_ADC_GetValue(&adc);
//}


////*********************************************中值**************************************//
//int filter_M(int *filter,int len){
//	sort(filter,len);
//	return filter[len/2];
//}
// 
////******************************************排序************************************//
//void sort(int* a,int len)
//{
//    int begin = 1;
//    int i = 0;
//    while(begin < len)
//    {
//        int key = a[begin];
//        for(i = begin-1;i>=0;i--)
//        {
//            if(a[i]<=key)    
//            {
//                a[i+1] = key;
//                break;
//            }
//            a[i+1] = a[i];
//        }
//        if(i<0)
//            a[0] = key;
//        begin++;
//    }
//}


//void SET4G(void){
//	delay_ms(300);
//	printf("AT+CGSOCKCONT=1,\"IP\",\"CMNET\"\r\n");
//	delay_ms(300);
//	printf("AT+CSOCKSETPN=1\r\n");
//	delay_ms(300);
//	printf("AT+NETOPEN\r\n");
//	delay_ms(300);
//	printf("AT+CIPOPEN=1,\"UDP\",,,20030\r\n");
//	delay_ms(300);
//	printf("AT+CIPSEND=1,18,\"219.128.73.196\",20030\r\n");
//	delay_ms(300);
//	printf("4G模块初始化完成\r\n");
////	HAL_GPIO_TogglePin(BEE_GPIO_Port,BEE_Pin);
////	delay_ms(1000);
////	HAL_GPIO_TogglePin(BEE_GPIO_Port,BEE_Pin);
//	
//}
// 
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	
//	tim_count++;
//	if(tim_count==1000){HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);tim_count=0;
//						time_out++;
//						connect_confirm++;
//						led_flag=!led_flag;
//						if(flag1){	
//									HAL_GPIO_WritePin(KM1_GPIO_Port,KM1_Pin,GPIO_PIN_SET);
//									beep_flag=1;
//									}
//						
//						if(flag2){	
//									HAL_GPIO_WritePin(KM2_GPIO_Port,KM2_Pin,GPIO_PIN_SET);
//									beep_flag=1;
//									
//									}
//									
//						if(flag3){	
//									HAL_GPIO_WritePin(KM3_GPIO_Port,KM3_Pin,GPIO_PIN_SET);
//									beep_flag=1;
//									
//									}
//									
//						if(flag4){	
//									HAL_GPIO_WritePin(KM4_GPIO_Port,KM4_Pin,GPIO_PIN_SET);
//									beep_flag=1;
//									
//									}
//									
//						if(flag5){	HAL_GPIO_TogglePin(LED7_GPIO_Port,LED7_Pin);
//									HAL_GPIO_WritePin(KM5_GPIO_Port,KM5_Pin,GPIO_PIN_SET);
//									beep_flag=1;
//									
//									}
//									
//						if(flag6){	HAL_GPIO_TogglePin(LED8_GPIO_Port,LED8_Pin);
//									HAL_GPIO_WritePin(KM6_GPIO_Port,KM6_Pin,GPIO_PIN_SET);
//									beep_flag=1;
//									
//									}
//									
//						if(flag7){	HAL_GPIO_TogglePin(LED9_GPIO_Port,LED9_Pin);
//									HAL_GPIO_WritePin(KM7_GPIO_Port,KM7_Pin,GPIO_PIN_SET);
//									beep_flag=1;
//									
//									}
//									
//						if(flag8){	HAL_GPIO_TogglePin(LED10_GPIO_Port,LED10_Pin);
//									HAL_GPIO_WritePin(KM8_GPIO_Port,KM8_Pin,GPIO_PIN_SET);
//									beep_flag=1;
//									
//									}
//						
//						if(led_flag){			if(flag1)HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_SET);
//															if(flag2)HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_SET);
//															if(flag3)HAL_GPIO_WritePin(LED5_GPIO_Port,LED5_Pin,GPIO_PIN_SET);
//															if(flag4)HAL_GPIO_WritePin(LED6_GPIO_Port,LED6_Pin,GPIO_PIN_SET);
//															if(flag5)HAL_GPIO_WritePin(LED7_GPIO_Port,LED7_Pin,GPIO_PIN_SET);
//															if(flag6)HAL_GPIO_WritePin(LED8_GPIO_Port,LED8_Pin,GPIO_PIN_SET);
//															if(flag7)HAL_GPIO_WritePin(LED9_GPIO_Port,LED9_Pin,GPIO_PIN_SET);
//															if(flag8)HAL_GPIO_WritePin(LED10_GPIO_Port,LED10_Pin,GPIO_PIN_SET);}
//												else{ if(flag1)HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_RESET);
//															if(flag2)HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_RESET);
//															if(flag3)HAL_GPIO_WritePin(LED5_GPIO_Port,LED5_Pin,GPIO_PIN_RESET);
//															if(flag4)HAL_GPIO_WritePin(LED6_GPIO_Port,LED6_Pin,GPIO_PIN_RESET);
//															if(flag5)HAL_GPIO_WritePin(LED7_GPIO_Port,LED7_Pin,GPIO_PIN_RESET);
//															if(flag6)HAL_GPIO_WritePin(LED8_GPIO_Port,LED8_Pin,GPIO_PIN_RESET);
//															if(flag7)HAL_GPIO_WritePin(LED9_GPIO_Port,LED9_Pin,GPIO_PIN_RESET);
//															if(flag8)HAL_GPIO_WritePin(LED10_GPIO_Port,LED10_Pin,GPIO_PIN_RESET);}			
//						if(beep_flag)HAL_GPIO_TogglePin(BEE_GPIO_Port,BEE_Pin);
//						}
//	
//	
//	if(!HAL_GPIO_ReadPin(K1_GPIO_Port,K1_Pin)){flag1=0;HAL_GPIO_WritePin(KM1_GPIO_Port,KM1_Pin,GPIO_PIN_RESET);
//																						HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_RESET);
//																						beep_flag=0;HAL_GPIO_WritePin(BEE_GPIO_Port,BEE_Pin,GPIO_PIN_RESET);}
//	if(!HAL_GPIO_ReadPin(K2_GPIO_Port,K2_Pin)){flag2=0;HAL_GPIO_WritePin(KM1_GPIO_Port,KM1_Pin,GPIO_PIN_RESET);
//																						HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_RESET);
//																						beep_flag=0;HAL_GPIO_WritePin(BEE_GPIO_Port,BEE_Pin,GPIO_PIN_RESET);}					
//	if(!HAL_GPIO_ReadPin(K3_GPIO_Port,K3_Pin)){flag3=0;HAL_GPIO_WritePin(KM1_GPIO_Port,KM1_Pin,GPIO_PIN_RESET);
//																						HAL_GPIO_WritePin(LED5_GPIO_Port,LED5_Pin,GPIO_PIN_RESET);
//																						beep_flag=0;HAL_GPIO_WritePin(BEE_GPIO_Port,BEE_Pin,GPIO_PIN_RESET);}
//	if(!HAL_GPIO_ReadPin(K4_GPIO_Port,K4_Pin)){flag4=0;HAL_GPIO_WritePin(KM1_GPIO_Port,KM1_Pin,GPIO_PIN_RESET);
//																						HAL_GPIO_WritePin(LED6_GPIO_Port,LED6_Pin,GPIO_PIN_RESET);
//																						beep_flag=0;HAL_GPIO_WritePin(BEE_GPIO_Port,BEE_Pin,GPIO_PIN_RESET);}					
//	if(!HAL_GPIO_ReadPin(K5_GPIO_Port,K5_Pin)){flag5=0;HAL_GPIO_WritePin(KM1_GPIO_Port,KM1_Pin,GPIO_PIN_RESET);
//																						HAL_GPIO_WritePin(LED7_GPIO_Port,LED7_Pin,GPIO_PIN_RESET);
//																						beep_flag=0;HAL_GPIO_WritePin(BEE_GPIO_Port,BEE_Pin,GPIO_PIN_RESET);}					
//	if(!HAL_GPIO_ReadPin(K6_GPIO_Port,K6_Pin)){flag6=0;HAL_GPIO_WritePin(KM1_GPIO_Port,KM1_Pin,GPIO_PIN_RESET);
//																						HAL_GPIO_WritePin(LED8_GPIO_Port,LED8_Pin,GPIO_PIN_RESET);
//																						beep_flag=0;HAL_GPIO_WritePin(BEE_GPIO_Port,BEE_Pin,GPIO_PIN_RESET);}					
//	if(!HAL_GPIO_ReadPin(K7_GPIO_Port,K7_Pin)){flag7=0;HAL_GPIO_WritePin(KM1_GPIO_Port,KM1_Pin,GPIO_PIN_RESET);
//																						HAL_GPIO_WritePin(LED9_GPIO_Port,LED9_Pin,GPIO_PIN_RESET);
//																						beep_flag=0;HAL_GPIO_WritePin(BEE_GPIO_Port,BEE_Pin,GPIO_PIN_RESET);}					
//	if(!HAL_GPIO_ReadPin(K8_GPIO_Port,K8_Pin)){flag8=0;HAL_GPIO_WritePin(KM1_GPIO_Port,KM1_Pin,GPIO_PIN_RESET);
//																						HAL_GPIO_WritePin(LED10_GPIO_Port,LED10_Pin,GPIO_PIN_RESET);
//																						beep_flag=0;HAL_GPIO_WritePin(BEE_GPIO_Port,BEE_Pin,GPIO_PIN_RESET);}					
//}

///* USER CODE END 4 */

///**
//  * @brief  This function is executed in case of error occurrence.
//  * @retval None
//  */
//void Error_Handler(void)
//{
//  /* USER CODE BEGIN Error_Handler_Debug */
//  /* User can add his own implementation to report the HAL error return state */

//  /* USER CODE END Error_Handler_Debug */
//}

//#ifdef  USE_FULL_ASSERT
///**
//  * @brief  Reports the name of the source file and the source line number
//  *         where the assert_param error has occurred.
//  * @param  file: pointer to the source file name
//  * @param  line: assert_param error line source number
//  * @retval None
//  */
//void assert_failed(uint8_t *file, uint32_t line)
//{ 
//  /* USER CODE BEGIN 6 */
//  /* User can add his own implementation to report the file name and line number,
//     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//  /* USER CODE END 6 */
//}
//#endif /* USE_FULL_ASSERT */

///************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
