
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

#include <stdbool.h>
#include "stdio.h"
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint16_t vtZ=0,vtG=0,vtG_all=0;
bool volatile Ispressed=false,IsOntime=false, IsAutoMode;
uint8_t send_data=42,receive_dataZ,receive_dataG;
char receiveZ[10000],receiveG[10000],send[10000],receiveG_all[10000],data_server_R[10000],data_server_T[10000],stt_RL[11],thres[30];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
bool Init_GPRS(__IO uint32_t milis);
bool Get_Method(char url[],char data[],__IO uint32_t milis);
bool Post_Method(char url[],char data[],__IO uint32_t milis);
bool Connect_Server(__IO uint32_t milis);
bool Disconnect_Server(__IO uint32_t milis);
bool Check_str(char a[],char b[], uint16_t v);
bool Cmp_str(char a[],char b[],char stt[]);
bool Cmp_str2(char a[],char b[]);
bool get_Zigbee(char zigbee[],char data[],__IO uint32_t milis);
void Check_Thres(char value[],char thres[],char stt[]);
void Ctrl_RL(char stt[]);
void Error (int msg);
int error_msg=0;
int _Split(char str[],char a,float des[]);
void mem_clear(char a[],char value,int i);
void Getvalue( char a[],char b[]);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void Getvalue(char a[],char b[])
{
	int i=0;
	while(a[i]!='\r')
	{
		b[i] = a[i];
		i++;
	}
	b[i]=0;
}
void Error (int msg)
{
	switch(msg)
	{
		case 1: default:
			Disconnect_Server(500);
		case 2:		case 3:
			Connect_Server(500);
			break;
	}
}
bool Check_str(char a[],char b[], uint16_t v)
{
		for (int i = strlen(b); i > 0; i--)
		{
				if ( a[v-strlen(b)+i-1] != b[i-1]) return false; 
		}
		
		return true;
}
bool Cmp_str(char a[],char b[],char stt[])
{
	bool Isok=false;
	int vt_b=0,vt_stt=0;
	for (int i = 0; i < strlen(a); i++)
		{
				if ( a[i] == b[vt_b]) vt_b++;
				else vt_b=0; 
			if (vt_b==strlen(b)) Isok = true; 			
			if (Isok)
			{
				if(a[i+1]=='\r' && a[i+3]==0) return true;
				stt[vt_stt] = a[i+1];
				vt_stt ++;
			}		 
		}
	if(Isok) return true;
		else return false;
}
bool Cmp_str2(char a[],char b[])
{
	int vt_b=0,vt_stt=0;
	for (int i = 0; i < strlen(a); i++)
		{
				if ( a[i] == b[vt_b]) vt_b++;
				else vt_b=0; 
			if (vt_b==strlen(b)) return true; 			
		}
	return false;
}
bool Init_GPRS(__IO uint32_t milis)
{
			milis *= (SystemCoreClock / 1000) / 9;
			uint32_t milis_tmp=milis;
			strcpy(send,"AT\r\n");
 	    HAL_UART_Transmit_IT(&huart6,send,strlen(send));
			while (milis--)
			{
				if (Check_str(receiveG,"\r\nOK\r\n",vtG)) break;
				else if (Cmp_str2(receiveG,"ERROR")) {return false;}
//				else if (Cmp_str2(receiveG,"+CME ERROR:50")) {error_msg=1; return false;}
//				else if (Cmp_str2(receiveG,"+CME ERROR:53")) {error_msg=2; return false;}
//				else if (Cmp_str2(receiveG,"+CME ERROR:58")) {error_msg=3; return false;}
			}
			milis=milis_tmp;
			mem_clear(receiveG, 0, 300);
			vtG=0;
			strcpy(send,"ATE0\r\n");
			HAL_UART_Transmit_IT(&huart6,send,strlen(send));
			while (milis--)
			{
				if (Check_str(receiveG,"\r\nOK\r\n",vtG)) break;
			}
			if(++milis<=0) return false;
			mem_clear(receiveG, 0, 300);
			vtG=0;
			return true;
}
bool Connect_Server(__IO uint32_t milis)
{
			milis *= (SystemCoreClock / 1000) / 9;
			strcpy(send,"AT+CIPSTART=\"TCP\",\"1413264.somee.com\",80\r\n");
  	  HAL_UART_Transmit_IT(&huart6,send,strlen(send));
			while (milis--)
			{
				if (Check_str(receiveG,"\r\nOK\r\n",vtG)) break;
				else if (Cmp_str2(receiveG,"ERROR")) {return false;}
			}
			if(++milis<=0) return false;
				mem_clear(receiveG, 0, 300);
				vtG=0;
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);
				return true;
}
bool Disconnect_Server(__IO uint32_t milis)
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET);
			milis *= (SystemCoreClock / 1000) / 9;
			strcpy(send,"AT+CIPSHUT\r\n");
  	  HAL_UART_Transmit_IT(&huart6,send,strlen(send));
			while (milis--)
			{
				if (Check_str(receiveG,"\r\nOK\r\n",vtG)) break;
				else if (Cmp_str2(receiveG,"ERROR")) {return false;}
			}
			if(++milis<=0) return false;
			mem_clear(receiveG, 0, 300);
			vtG=0;
			return true;
}
bool Get_Method(char url[],char data[],__IO uint32_t milis)
{
			milis *= (SystemCoreClock / 1000) / 9;
			uint32_t milis_tmp=milis;
			char tmp[200];
			strcpy(tmp," HTTP/1.1\r\nHost: 1413264.somee.com\r\nConnection: Keep-Alive\r\n\r\n\x1A");
		//	strcpy(receiveG_all,"");
	    //vtG_all=0;
			strcpy(send,"AT+CIPSEND\r\n");
  	  HAL_UART_Transmit_IT(&huart6,send,strlen(send));
			while (milis--)
			{
				if (Check_str(receiveG,"> ",vtG)) break;
				else if (Cmp_str2(receiveG,"ERROR")) {return false;}				
			}
			if(++milis<=0) return false;
			milis=milis_tmp;
			mem_clear(receiveG, 0, 300);
			vtG=0;
			strcpy(send,"GET ");
			strcat(send,url);
//			strcat(send,"?");
//			strcat(send,data);
			strcat(send,tmp);
			HAL_UART_Transmit_IT(&huart6,send,strlen(send));
			 int vtG_old=0;
			while (milis--)
			{
				if (Cmp_str2(receiveG,"Content-Length: ")) break;
				else if (Cmp_str2(receiveG,"ERROR")) {return false;}
			}
			if(++milis<=0) return false;
			HAL_Delay(5);
			strcpy(data,receiveG);
			mem_clear(receiveG, 0, 300);
			vtG=0;
			return true;
}
bool Post_Method(char url[],char data[],__IO uint32_t milis)
{
			milis *= (SystemCoreClock / 1000) / 9;
			uint32_t milis_tmp=milis;
			char tmp[200];
			strcpy(tmp," HTTP/1.1\r\nHost: 1413264.somee.com\r\nContent-Type: application/x-www-form-urlencoded\r\nConnection: Keep-Alive\r\nContent-Length: ");
			mem_clear(receiveG, 0, 500);
	    vtG=0;
			strcpy(send,"AT+CIPSEND\r\n");
  	  HAL_UART_Transmit_IT(&huart6,send,strlen(send));
			while (milis--)
			{
				if (Check_str(receiveG,"> ",vtG)) break;
				else if (Cmp_str2(receiveG,"ERROR")) {return false;}
			}
			if(++milis<=0) return false;
			milis=milis_tmp;
			mem_clear(receiveG, 0, 500);
			vtG=0;
			strcpy(send,"POST ");
			strcat(send,url);
//			strcat(send,"?");
//			strcat(send,data);
			strcat(send,tmp);
			sprintf(tmp,"%d\r\n\r\n",strlen(data)+4);
			strcat(send,tmp);
			strcat(send,"msg=");
			strcat(send,data);
			strcat(send,"\r\n\x1A");
			HAL_UART_Transmit_IT(&huart6,send,strlen(send));
			while (milis--)
			{
				if (Check_str(receiveG,"\r\n\r\n",vtG)) break;
				else if (Cmp_str2(receiveG,"ERROR")) {return false;}
			}
			if(++milis<=0) return false;
			milis=milis_tmp;

			strcpy(data,receiveG);
			mem_clear(receiveG, 0, 300);
			vtG=0;
	    vtG_all=0;
			return true;
}
int _Split(char str[],char a,float des[])
{
	int vt_des=0,div=1;
	float tmp=0;
	bool Ismul=true;
	for (int i = 0;i<strlen(str);i++)
	{
		if (str[i]==a )
		{
			des[vt_des++]=tmp;
			div=1;
			tmp=0;
		}
		else if(str[i]=='.')
		{
			Ismul=false;
		}
		else 
		{
			if(!Ismul)
			{
					div*=10;
				tmp += (float)(str[i]-48)/div;
			}		
			else
			{
			tmp=tmp*10+(float)(str[i]-48);
			}
		}
	}
	des[vt_des++]=tmp;
	return vt_des;
}
void Check_Thres(char value[],char thres[],char stt[])
{
	float vl[10];
	float	thrs[10];
	int n;
	n=_Split(value,'@',vl);
	_Split(thres,0x20,thrs);
	for (int i = 0; i<n; i++)
	{
		if (vl[i]<thrs[i]) stt[2*i]='0'; 
		else stt[2*i]='1'; 
		if (i<n-1) stt[2*i+1]=0x20;
	}	
}
void Ctrl_RL(char stt[])
{
	if (stt[0] == '1')  {RL1_ON;}
	else {RL1_OFF;}
	if (stt[2] == '1')  {RL2_ON;}
	else {RL2_OFF;}
	if (stt[4] == '1')  {RL3_ON;}
	else {RL3_OFF;}
	if (stt[6] == '1')  {RL4_ON;}
	else {RL4_OFF;}
	if (stt[8] == '1') { RL5_ON;}
	else {RL5_OFF;}
	if (stt[10] == '1')  {RL6_ON;}
	else {RL6_OFF;}
//	if (stt[11] == '1') { RL7_ON;}
//	else {RL7_OFF;}
//	if (stt[12] == '1') {RL8_ON;}
//	else {RL8_OFF;}
}
void mem_clear(char a[],char value,int i)
{
	for (int j=0; j< i; j++)
	{
		a[j]=value;
	}
}
bool get_Zigbee(char zigbee[],char data[],__IO uint32_t milis)
{
	milis *= (SystemCoreClock / 1000) / 9;
	HAL_UART_Transmit_IT(&huart2,zigbee,2);
			while (milis--)
			{
				if( Check_str(receiveZ,"\r\n",vtZ)) break;
			}
			if (++milis<=0){return false;}
			milis=50;
			milis *= (SystemCoreClock / 1000) / 9;
			Getvalue(receiveZ,data);
			mem_clear(receiveZ,0,100);
			vtZ=0;
			return true;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
	{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_DMA(&huart2,&receive_dataZ,1);
	HAL_UART_Receive_DMA(&huart6,&receive_dataG,1);
	HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	while(Ispressed==false);
	HAL_Delay(1000);
	Init_GPRS(500); 
	while(!Connect_Server(500))
	{
				HAL_Delay(100);
		Disconnect_Server(500);
		HAL_Delay(100);
	}
	__IO uint32_t milis=50;
	milis *= (SystemCoreClock / 1000) / 9;
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		if(IsOntime)
		{
			IsOntime=false;
			strcpy(data_server_T,"");
			while (!Get_Method("/RL.txt",data_server_R,500))
			{
				HAL_Delay(100);
				Error(error_msg);
			}
			if (Cmp_str(data_server_R,"Manual ",stt_RL)) IsAutoMode = false;
			else IsAutoMode=true;
			mem_clear(data_server_R,0,400);
						HAL_Delay(50);
				while(!get_Zigbee("1@",data_server_T,50));
			//strcpy(receiveZ,"");
//			strcpy(send,"2@");
//			HAL_UART_Transmit_IT(&huart2,send,2);
//			while (!Check_str(receiveZ,"\r\n",vtZ));
//			vtZ=0;
//			strcpy(receiveZ,"");
			strcpy(receiveZ,"@25@500@0.5");
			
			strcat(data_server_T,receiveZ);
			mem_clear(receiveZ,0,100);
			vtZ=0;
			if (IsAutoMode)
			{
				while (!Get_Method("/TH.txt",data_server_R,500))
				{
					HAL_Delay(100);					
					Error(error_msg);

				}
				Cmp_str(data_server_R,"Content-Length: ",thres);
				for (int i =0;i<strlen(thres);i++)
				{
					if ((i+6)>=strlen(thres)) thres[i]=0;
						thres[i]=thres[i+6];
				}
				Check_Thres(data_server_T,thres,stt_RL);
				Ctrl_RL(stt_RL);
				//strcpy(stt_RL,"1 1 0 0 1 1");
				strcat(data_server_T,"@Auto ");
				strcat(data_server_T,stt_RL);
				while(!Post_Method("/Home/CreateDataAll",data_server_T,500))
				{
					HAL_Delay(100);
					Error(error_msg);
					
				}
			}
			else
			{
				Ctrl_RL(stt_RL);
				while (!Post_Method("/Home/CreateData",data_server_T,500))
					{
					HAL_Delay(100);
					//Ctrl_RL(stt_RL);
					Error(error_msg);
				}
			}
		}
	}
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 59999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1399;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART6 init function */
static void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PE8 PE10 PE12 PE14 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB12 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD12 PD13 PD14 
                           PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==huart2.Instance)
	{
	//	HAL_UART_Receive_IT(&huart2,&data_dieukhien,1);
		HAL_UART_Receive_IT(&huart2,&receive_dataZ,1);
		receiveZ[vtZ]= receive_dataZ;
		vtZ++;
	}
		if(huart->Instance==huart6.Instance)
	{
	//	HAL_UART_Receive_IT(&huart2,&data_dieukhien,1);
		HAL_UART_Receive_IT(&huart6,&receive_dataG,1);
		receiveG[vtG]= receive_dataG;
	//	receiveG_all[vtG_all]= receive_dataG;
		vtG++;
//vtG_all++;
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  if(htim->Instance==htim2.Instance)
	{

		IsOntime=true;
	}
  /* NOTE : This function Should not be modified, when the callback is needed,
            the __HAL_TIM_PeriodElapsedCallback could be implemented in the user file
   */
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  Ispressed = true;
	while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0));
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
