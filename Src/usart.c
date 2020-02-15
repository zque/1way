/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
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

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "stdio.h"
/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

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

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

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

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();
  
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */



#include "usart.h"
////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用os,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//os 使用	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32H7开发板
//串口1初始化		   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2017/6/8
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved
//********************************************************************************
//V1.0修改说明 
////////////////////////////////////////////////////////////////////////////////// 	  
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
//#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)	
#if 1
//#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
//int fputc(int ch, FILE *f)
//{ 	
//	while((USART1->ISR&0X40)==0);//循环发送,直到发送完毕   
//	USART1->TDR=(u8)ch;      
//	return ch;
//}
#endif 

#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	

u8 aRxBuffer[RXBUFFERSIZE];//HAL库使用的串口接收缓冲
UART_HandleTypeDef UART1_Handler; //UART句柄

//初始化IO 串口1 
//bound:波特率
//void uart_init(u32 bound)
//{	
//	//UART 初始化设置
//	UART1_Handler.Instance=USART1;					    //USART1
//	UART1_Handler.Init.BaudRate=bound;				    //波特率
//	UART1_Handler.Init.WordLength=UART_WORDLENGTH_8B;   //字长为8位数据格式
//	UART1_Handler.Init.StopBits=UART_STOPBITS_1;	    //一个停止位
//	UART1_Handler.Init.Parity=UART_PARITY_NONE;		    //无奇偶校验位
//	UART1_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //无硬件流控
//	UART1_Handler.Init.Mode=UART_MODE_TX_RX;		    //收发模式
//	HAL_UART_Init(&UART1_Handler);					    //HAL_UART_Init()会使能UART1
//	
//	HAL_UART_Receive_IT(&UART1_Handler, (u8 *)aRxBuffer, RXBUFFERSIZE);//该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量
//  
//}

//UART底层初始化，时钟使能，引脚配置，中断配置
//此函数会被HAL_UART_Init()调用
//huart:串口句柄

//void HAL_UART_MspInit(UART_HandleTypeDef *huart)
//{
//    //GPIO端口设置
//	GPIO_InitTypeDef GPIO_Initure;
//	
//	if(huart->Instance==USART1)//如果是串口1，进行串口1 MSP初始化
//	{
//		__HAL_RCC_GPIOA_CLK_ENABLE();			//使能GPIOA时钟
//		__HAL_RCC_USART1_CLK_ENABLE();			//使能USART1时钟
//	
//		GPIO_Initure.Pin=GPIO_PIN_2;			//PA9
//		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//复用推挽输出
//		GPIO_Initure.Pull=GPIO_PULLUP;			//上拉
//		GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;//高速
//		GPIO_Initure.Alternate=GPIO_AF7_USART1;	//复用为USART1
//		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//初始化PA9

//		GPIO_Initure.Pin=GPIO_PIN_3;			//PA10
//		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//初始化PA10
//		
//#if EN_USART1_RX
//		HAL_NVIC_EnableIRQ(USART1_IRQn);				//使能USART1中断通道
//		HAL_NVIC_SetPriority(USART1_IRQn,3,3);			//抢占优先级3，子优先级3
//#endif	
//	}

//}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if(huart->Instance==USART2)//如果是串口1
	{
		if(aRxBuffer[0]==0x23)USART_RX_STA|=0x8000;	//接收完成了 
		if((USART_RX_STA&0x8000)==0)//接收未完成
		{
			if(USART_RX_STA&0x4000)//接收到了0x0d
			{
				if(aRxBuffer[0]!=0x0a)USART_RX_STA=0;//接收错误,重新开始
				else USART_RX_STA=0;//USART_RX_STA|=0x8000;	//接收完成了 
			}
			else //还没收到0X0D
			{	
				if(aRxBuffer[0]==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=aRxBuffer[0] ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
				}		 
			}
			
		}
		
		
//		USART_RX_BUF[USART_RX_STA&0X3FFF]=aRxBuffer[0] ;
//		if(aRxBuffer[0]==0x23)
//		{
//			USART_RX_STA|=0x8000;
//			USART_RX_STA=0;
//		}
//		USART_RX_STA++;
		printf("hello");
		

	}
	if(huart->Instance == USART1)
//		{if(Uart3_Rx_Cnt >= 255)  //溢出判断
//		{
//			Uart3_Rx_Cnt = 0;
//			memset(Uart3_RxBuff,0x00,sizeof(Uart3_RxBuff));
//			HAL_UART_Transmit(&huart3, (uint8_t *)&cAlmStr, sizeof(cAlmStr),0xFFFF);	
//		}
//		else
//		{
//			Uart3_RxBuff[Uart3_Rx_Cnt++] = aRxBuffer3;   //接收数据转存
//				//HAL_GPIO_TogglePin(BEE_GPIO_Port,BEE_Pin);
//			if(aRxBuffer3==0x06){	printf_flag=0;
//									HAL_GPIO_WritePin(RE_GPIO_Port,RE_Pin,GPIO_PIN_SET);
//									for (int i=0;i<1024;i++){printf("%f\t%f\r\n",adc1[i],adc0[i]);}
//									printf("相似度：%f\t漏电值1：%f\t漏电值0：%f\t突变值：%i\r\n",cos0,I1/530,I0/530,har/530);
//									printf("漏电时间：\r\n");
//									printf("%02d/%02d/%02d\r\n",2000 + year, month, date);
//									printf("%02d:%02d:%02d\r\n",hour, minute, second);
//									printf("当前时间：\r\n");
//									HAL_RTC_GetTime(&hrtc, &stimestruct, RTC_FORMAT_BIN);
//									HAL_RTC_GetDate(&hrtc, &sdatestruct, RTC_FORMAT_BIN);
//									printf("%02d/%02d/%02d\r\n",2000 + sdatestruct.Year, sdatestruct.Month, sdatestruct.Date);
//									printf("%02d:%02d:%02d\r\n",stimestruct.Hours, stimestruct.Minutes, stimestruct.Seconds);
//									
//									HAL_GPIO_WritePin(RE_GPIO_Port,RE_Pin,GPIO_PIN_RESET);
//									printf_flag=1;
//									}
//			if((Uart3_RxBuff[Uart3_Rx_Cnt-1] == 0x0A)&&(Uart3_RxBuff[Uart3_Rx_Cnt-2] == 0x0D)) //判断结束位
//			{
////											HAL_UART_Transmit(&huart2, (uint8_t *)&Uart3_RxBuff, Uart3_Rx_Cnt,0xFFFF);//将485串口收到的信息发送到串口2
//				Uart3_Rx_Cnt = 0;
//				memset(Uart3_RxBuff,0x00,sizeof(Uart3_RxBuff)); //清空数组
//			}
//		}
//		HAL_UART_Receive_IT(&huart3, (uint8_t *)&aRxBuffer3, 1);}
		{
			//uartFlag=1;
			if(aRxBuffer[0]==0x23)USART_RX_STA|=0x8000;	//接收完成了 
		if((USART_RX_STA&0x8000)==0)//接收未完成
		{
			if(USART_RX_STA&0x4000)//接收到了0x0d
			{
				if(aRxBuffer[0]!=0x0a)USART_RX_STA=0;//接收错误,重新开始
				else USART_RX_STA=0;//USART_RX_STA|=0x8000;	//接收完成了 
			}
			else //还没收到0X0D
			{	
				if(aRxBuffer[0]==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=aRxBuffer[0] ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
				}		 
			}
			
		}
		HAL_UART_Receive_IT(&huart1, (uint8_t *)&aRxBuffer, 1);
		}
}
 
//串口1中断服务程序
void USART2_IRQHandler(void)                	
{ 
	u32 timeout=0;
    u32 maxDelay=0x1FFFF;
#if SYSTEM_SUPPORT_OS	 	//使用OS
	OSIntEnter();    
#endif
	
	HAL_UART_IRQHandler(&huart2);	//调用HAL库中断处理公用函数
	
	timeout=0;
    while (HAL_UART_GetState(&huart2)!=HAL_UART_STATE_READY)//等待就绪
	{
        timeout++;////超时处理
        if(timeout>maxDelay) break;		
	}
     
	timeout=0;
	while(HAL_UART_Receive_IT(&huart2,(u8 *)aRxBuffer, RXBUFFERSIZE)!=HAL_OK)//一次处理完成之后，重新开启中断并设置RxXferCount为1
	{
        timeout++; //超时处理
        if(timeout>maxDelay) break;	
	}
#if SYSTEM_SUPPORT_OS	 	//使用OS
	OSIntExit();  											 
#endif
} 
#endif	

/*下面代码我们直接把中断控制逻辑写在中断服务函数内部。*/
/*

//串口1中断服务程序
void USART1_IRQHandler(void)                	
{ 
	u8 Res;
#if SYSTEM_SUPPORT_OS	 	//使用OS
	OSIntEnter();    
#endif
	if((__HAL_UART_GET_FLAG(&UART1_Handler,UART_FLAG_RXNE)!=RESET))  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
        HAL_UART_Receive(&UART1_Handler,&Res,1,1000); 
		if((USART_RX_STA&0x8000)==0)//接收未完成
		{
			if(USART_RX_STA&0x4000)//接收到了0x0d
			{
				if(Res!=0x0a)USART_RX_STA=0;//接收错误,重新开始
				else USART_RX_STA|=0x8000;	//接收完成了 
			}
			else //还没收到0X0D
			{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
				}		 
			}
		}   		 
	}
	HAL_UART_IRQHandler(&UART1_Handler);	
#if SYSTEM_SUPPORT_OS	 	//使用OS
	OSIntExit();  											 
#endif
} 
#endif	
*/











/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
