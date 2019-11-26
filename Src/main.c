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
#include "adc.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//USE_HAL_DRIVER, STM32H743xx,ARM_MATH_CM7,__CC_ARM,ARM_MATH_MATRIX_CHECK,ARM_MATH_ROUNDING  ������к궨������FFT�㷨
//#include "printf.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "sys.h"
#include "delay.h"
#include "arm_math.h"
#include "usmart.h"
#include "stmflash.h"
#define FFT_LENGTH 		1024
#define adc2i					670							//adcת���ɵ���i�ı�ֵ
#define bee				HAL_GPIO_To
#define FLASH_SAVE_ADDR  0X08020000 	//����FLASH �����ַ(����Ϊ4�ı���������������,Ҫ���ڱ�������ռ�õ�������.
										//����,д������ʱ��,���ܻᵼ�²�����������,�Ӷ����𲿷ֳ���ʧ.��������.

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

/* USER CODE BEGIN PV */
int i=0;							//forѭ������
int j=0;							//forѭ������
int k=0;							//forѭ������
int sw=0;							//�л�ǰ�������ݿ��Ʊ���		
int count=0;
float amp_value=0.0;				//����©�����ת��Ϊ��ֵ��Чֵ
int zero = 33220;
int reset4G =0;  //4g���� ���ճ�ʱ���ñ�־
int connect_confirm =0; //��ʱ������Ϣȷ������
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
u32 adc[100];
u8 USART_RX_STA;
u8 USART_RX_BUF[1];
RTC_DateTypeDef sdatestructure;
RTC_TimeTypeDef stimestructure;
int year,month,date;
int hour,minute,second;
int flash[2];
int limit;
int I;				

//struct {
//	int K1_Pin:1;
//	
//}sw_input;


uint8_t aRxBuffer2;			//�����жϻ���
uint8_t Uart2_RxBuff[256];		//���ջ���
uint8_t Uart2_Rx_Cnt = 0;		//���ջ������

uint8_t aRxBuffer1;			//�����жϻ���
uint8_t Uart3_RxBuff[256];		//���ջ���
uint8_t Uart3_Rx_Cnt = 0;		//���ջ������
uint8_t	cAlmStr[] = "�������(����256)\r\n";
								

int tim_count=0;
int start=0;						//��һ��ִ��main���� ֻ�ɼ���һ�����β����Ƚϣ����ڵ�һ�������Ƚ�
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
/* USER CODE BEGIN PFP */
//void SET4G(void);
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
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc3,ADC_CALIB_OFFSET,ADC_DIFFERENTIAL_ENDED);
  
 // HAL_TIM_Base_Start_IT(&htim1);
  delay_init(480);
  //HAL_GPIO_WritePin(RE_GPIO_Port,RE_Pin,GPIO_PIN_SET);
  HAL_UART_Receive_IT(&huart2, (uint8_t *)&aRxBuffer2,1);
  HAL_UART_Receive_IT(&huart1, (uint8_t *)&aRxBuffer1, 1);
  HAL_TIM_Base_Start_IT(&htim1);
  arm_cfft_radix4_init_f32(&scfft,FFT_LENGTH,0,1);
  delay_ms(1000);
  filter_index = filter_len/2;
  
 // HAL_ADC_Start(&hadc3);
 // HAL_ADC_Start_DMA(&hadc3,adc,100);
  amp_value=(adc2i*limit);			//����©�����ת��Ϊ������Ч��ֵ
									//Ƶ�׷�ֵ�벨����Чֵ��ϵ�� 		������Чֵ=Ƶ�׷�ֵ*2/FFT_LEANGTH
									//������Чֵ��©�������ϵ��		ADC��õ�ѹ=�ο���ѹ(3.3)/(ADC�ֱ���/2)*������Чֵ
									//								����������=ADC��õ�ѹ/���ص���(�ڲ����ص���Ϊ70ŷ��
									//								©�����=����������*1000(1000Ϊ����������Ȧ��1000��1��
									
			

	input_read = (GPIOH->IDR-->2)&0x0F;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		sw=!sw;
//		HAL_ADC_Start(&hadc3);
//		HAL_ADC_PollForConversion(&hadc3,0xffff);
//		printf("%i\r\n",HAL_ADC_GetValue(&hadc3));



//**************************************ADC����***************************************************//		
		if(sw){	for(i=0;i<1030;i++){for(k=0;k<filter_len;k++)filter1[k]=get_ADC(hadc3);
																adc1[i]=filter_M(filter1,filter_len);
																delay_us(85);}
						filter_A(adc1);
																		
		}else{for(i=0;i<1030;i++){for(k=0;k<filter_len;k++)filter1[k]=get_ADC(hadc3);
																adc0[i]=filter_M(filter1,filter_len);
																delay_us(85);}
						filter_A(adc0);
		}
		
		
	//*************************************����ƽ����cut��input***********************************************//
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
		
						
						
	//***************************************���ƶȼ���*********************************************************//
												
		if(start){A=B=AB=0;
							for(i=0;i<400;i++){A+=(cut0[i]-zero)*(cut0[i]-zero);B+=(cut1[i]-zero)*(cut1[i]-zero);AB+=(cut0[i]-zero)*(cut1[i]-zero);}
							cos1=AB*AB/A/B;}
		
							
							
	//***************************************FFT����************************************************************//
		if(sw){	arm_cfft_radix4_f32(&scfft,input1);
						arm_cmplx_mag_f32(input1,output1,FFT_LENGTH);
						Imax1=0;
						for(i=1;i<100;i++){	if(output1[i]>Imax1)Imax1=output1[i];}}
		else if(start){ 	
				arm_cfft_radix4_f32(&scfft,input0);
				arm_cmplx_mag_f32(input0,output0,FFT_LENGTH);
				Imax0=0;
				for(i=1;i<100;i++){	if(output0[i]>Imax0)Imax0=output0[i];}}
		
				
				
	//*****************************************************ǰ��������Ƚ�******************//				
		if(start){if(sw){if((Imax1-Imax0)>amp_value)har=(int)(Imax1-Imax0);}
							else{	if((Imax0-Imax1)>amp_value)har=(int)(Imax0-Imax1);}}
		
							
							
							
	//***************************************************����****************************************//
		if((cos1<0.95f)&&har){	flag=1;
						memcpy(adc0,avg0,sizeof(adc0));
						memcpy(adc1,avg1,sizeof(adc0));
						har_error=har;
						cos_error=cos1;
						I1_error=Imax1;
						I0_error=Imax0;
						printf("AT+CIPSEND=1,52,\"219.128.73.196\",20030\r\n");
						delay_ms(100);
						printf("©�����11��%05i\t©�����10��%05i\tͻ�����1��%05i",(int)Imax1/adc2i,(int)Imax0/adc2i,(har/adc2i));
								har=0;}
//		for(i=0;i<1024;i++)printf("%i\r\n",adc0[i]);
//		printf("*********************************");
		printf("%f\r\n",Imax1/adc2i);
		start=1;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_ADC;
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
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_UART_TxCpltCallback could be implemented in the user file
   */
	if(huart->Instance==USART1)//����Ǵ���1
	{
		if(aRxBuffer1==0x23)USART_RX_STA|=0x8000;	//��������� 
		if((USART_RX_STA&0x8000)==0)//����δ���
		{
			if(USART_RX_STA&0x4000)//���յ���0x0d
			{
				if(aRxBuffer1!=0x0a)USART_RX_STA=0;//���մ���,���¿�ʼ
				else USART_RX_STA=0;//USART_RX_STA|=0x8000;	//��������� 
			}
			else //��û�յ�0X0D
			{	
				if(aRxBuffer1==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=aRxBuffer1 ;
					USART_RX_STA++;
					if(USART_RX_STA>(200-1))USART_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
				}		 
			}
			
		}

									HAL_UART_Receive_IT(&huart2, (uint8_t *)&aRxBuffer2, 1);}   //�ٿ��������ж�
//	if(huart->Instance == USART3){if(Uart3_Rx_Cnt >= 255)  //����ж�
//									{
//										Uart3_Rx_Cnt = 0;
//										memset(Uart3_RxBuff,0x00,sizeof(Uart3_RxBuff));
//										HAL_UART_Transmit(&huart2, (uint8_t *)&cAlmStr, sizeof(cAlmStr),0xFFFF);	
//									}
//									else
//									{
//										Uart3_RxBuff[Uart3_Rx_Cnt++] = aRxBuffer1;   //��������ת��
//											//HAL_GPIO_TogglePin(BEE_GPIO_Port,BEE_Pin);
//										if(aRxBuffer1==0x06){	printf_flag=0;
//																					HAL_GPIO_WritePin(RE_GPIO_Port,RE_Pin,GPIO_PIN_SET);
//																					for (i=0;i<1024;i++){printf("%f\t%f\r\n",adc1_error[i],adc0_error[i]);}
//															                                  						printf("���ƶȣ�%f\t©��ֵ1��%f\t©��ֵ0��%f\tͻ��ֵ��%i\r\n",cos0,I1_error/adc2i,I0_error/adc2i,har/adc2i);
//																					HAL_GPIO_WritePin(RE_GPIO_Port,RE_Pin,GPIO_PIN_RESET);
//																					printf_flag=1;
//																					}
//										if((Uart3_RxBuff[Uart3_Rx_Cnt-1] == 0x0A)&&(Uart3_RxBuff[Uart3_Rx_Cnt-2] == 0x0D)) //�жϽ���λ
//										{
////											HAL_UART_Transmit(&huart2, (uint8_t *)&Uart3_RxBuff, Uart3_Rx_Cnt,0xFFFF);//��485�����յ�����Ϣ���͵�����2
//											Uart3_Rx_Cnt = 0;
//											memset(Uart3_RxBuff,0x00,sizeof(Uart3_RxBuff)); //�������
//										}
//									}
//									HAL_UART_Receive_IT(&huart1, (uint8_t *)&aRxBuffer1, 1);
//									HAL_UART_Receive_IT(&huart2, (uint8_t *)&aRxBuffer2, 1);}
		
		
							
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
//********************************************�޷�***************************************//
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


//*********************************************��ֵ**************************************//
int filter_M(int *filter,int len){
	sort(filter,len);
	return filter[len/2];
}
 
//******************************************����************************************//
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
//*************************************©���趨***************************************//
void set_limitA(int a){
	flash[0]=I;
	flash[1]=limit=a;
	
	STMFLASH_Write(FLASH_SAVE_ADDR,(u32*)flash,2);
	printf("limit=%i\r\n",limit);
	
}

//********************************************�趨©�����*************************************//
void set_I(int i)
{	
	flash[0]=I=i;
	flash[1]=limit;
	STMFLASH_Write(FLASH_SAVE_ADDR,(u32*)flash,2);
}

//***************************************ʱ���趨*****************************************//



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
//	printf("4Gģ���ʼ�����\r\n");
////	HAL_GPIO_TogglePin(BEE_GPIO_Port,BEE_Pin);
////	delay_ms(1000);
////	HAL_GPIO_TogglePin(BEE_GPIO_Port,BEE_Pin);
//	
//}
 
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	
	tim_count++;
	if(tim_count==1000)
		{	
			HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);tim_count=0;
			time_out++;
			connect_confirm++;
			led_flag=!led_flag;
			if(flag)
				{	
					HAL_GPIO_WritePin(KM1_GPIO_Port,KM1_Pin,GPIO_PIN_SET);
					beep_flag=1;
				}
						
					
									
		}
						
		if(led_flag){if(flag)HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_SET);}
		else{ if(flag)HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_RESET);}			
		if(beep_flag&&led_flag)HAL_GPIO_TogglePin(BEEP_GPIO_Port,BEEP_Pin);
						
	
	if(!HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin))
		{
			flag=0;HAL_GPIO_WritePin(KM1_GPIO_Port,KM1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_RESET);
			beep_flag=0;HAL_GPIO_WritePin(BEEP_GPIO_Port,BEEP_Pin,GPIO_PIN_RESET);
		}			
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
