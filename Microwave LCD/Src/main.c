
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
  * COPYRIGHT(c) 2020 STMicroelectronics
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
#include "stm32f7xx_hal.h"
#include "i2c.h"
#include "rng.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "ILI9341_Touchscreen.h"
#include "ILI9341_STM32_Driver.h"
#include "ILI9341_GFX.h"
#include "string.h"
#include "stdio.h"
#include "snow_tiger.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void DisplayTimer(uint32_t);
void Menu(int);
void TouchTimer(int);
void CheckDoor(int);
void CheckClear(int);
void Buzzer(int);
void Read_Data(int);
void Write_Data(int);
uint8_t menu=0,min=0,start=0,mode=0,Digit[2];
int pTimer=1,buzzer=0;
int debounce1=0,debounce2=1,dbTimer=0;
int door=0,logm=0;
char str[30];	
uint16_t x_pos = 0,y_pos = 0,position_array[2];	
int sec=0;
GPIO_PinState statedoor;
GPIO_PinState stateclear;
uint8_t Buffer[10];	

uint8_t turn,preturn;
int count=0,precount,rota;
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

  /* Enable I-Cache-------------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache-------------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_USART3_UART_Init();
  MX_SPI5_Init();
  MX_TIM1_Init();
  MX_RNG_Init();
  MX_TIM8_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT (&htim8);
	ILI9341_Init();//initial driver setup to drive ili9341
	ILI9341_Fill_Screen(BLACK);
	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);	
	
	Read_Data(1);
	logm=mode;
	CheckDoor(1);
	if(sec>0&&menu==2)
		DisplayTimer(sec);
	if(sec==0)
		menu=0;
	pTimer=1;	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		Buzzer(1);  
		CheckClear(1);
		
		if((menu==0||menu==1)&&door==0)
				Menu(1);

		if(menu==2&&door==0)
		    TouchTimer(1);
	  
	  if(start==1&&door==0){
			HAL_GPIO_WritePin(GPIOG, MOTOR_Pin, GPIO_PIN_RESET);	
		 	switch (mode){
				case 1: HAL_GPIO_WritePin(GPIOD, LED_RED_Pin, GPIO_PIN_SET);   break;
				case 2: HAL_GPIO_WritePin(GPIOD, LED_GREEN_Pin, GPIO_PIN_SET); break;
				case 3: HAL_GPIO_WritePin(GPIOD, LED_BLUE_Pin, GPIO_PIN_SET);	 break;}}
		 else{
		 		HAL_GPIO_WritePin(GPIOD, LED_RED_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOD, LED_GREEN_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOD, LED_BLUE_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOG, MOTOR_Pin, GPIO_PIN_SET);}
		 
		
//	 sprintf(str,"stateclear = %d\n\r",	stateclear);		  
//	 HAL_UART_Transmit(&huart3, (uint8_t*) str, strlen(str),1000);
	}
  /* USER CODE END 3 */

}
void Menu(int x){
	HAL_Delay(20);
	if(menu==0){
		ILI9341_Fill_Screen(BLACK);
		ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);		
		ILI9341_Draw_Rectangle(1,229, 105, 10, RED);	
		ILI9341_Draw_Rectangle(107,229, 106, 10, GREEN);	
		ILI9341_Draw_Rectangle(214,229, 105, 10, BLUE);
		ILI9341_Draw_Hollow_Rectangle_Coord(0, 0, 319, 239, WHITE);
		ILI9341_Draw_Rectangle(240, 80, 80, 1, WHITE);	
		ILI9341_Draw_Rectangle(240, 0, 1, 160, WHITE);	
		ILI9341_Draw_Rectangle(106, 160, 1, 80, WHITE);	
		ILI9341_Draw_Rectangle(213, 160, 1, 80, WHITE);	
		ILI9341_Draw_Rectangle(0, 160, 320, 1, WHITE);	
		sprintf(str, "%.2d",min);		
		ILI9341_Draw_Text(str, 65, 10, WHITE, 10, BLACK);
		ILI9341_Draw_Text("Minute", 34, 100, WHITE, 5, BLACK);
		ILI9341_Draw_Text("H", 247, 165, BLUE, 8, BLACK);		
		ILI9341_Draw_Text("M", 141, 165, GREEN, 8, BLACK);	
		ILI9341_Draw_Text("L", 34, 165, RED, 8, BLACK);	
		ILI9341_Draw_Rectangle(260,116, 40, 8, WHITE);	
		ILI9341_Draw_Rectangle(260,36, 40, 8, WHITE);	
		ILI9341_Draw_Rectangle(276,20, 8, 40, WHITE);	
	  menu=1;}
		if(TP_Touchpad_Pressed()){
			dbTimer=1;
					if(TP_Read_Coordinates(position_array) == TOUCHPAD_DATA_OK){
					x_pos = position_array[0];
					y_pos = position_array[1];		
					HAL_Delay(50);	
					if(x_pos<=80&&y_pos<=105&&min>=1)
						mode=1;					
					if(x_pos<=80&&(y_pos>=106&&y_pos<=213)&&min>=1)
						mode=2;	
					if(x_pos<=80&&y_pos>=214&&min>=1)
						mode=3;
					if(x_pos>=165&&y_pos>=242){
						if(min<60){
							buzzer=1;
							min++;
							sprintf(str, "%.2d",min);		
							ILI9341_Draw_Text(str, 65, 10, WHITE, 10, BLACK);}}															
					if((x_pos>=81&&x_pos<=161)&&y_pos>=242){
						if(min!=0){	
							buzzer=1;
							min--;
							sprintf(str, "%.2d",min);		
							ILI9341_Draw_Text(str, 65, 10, WHITE, 10, BLACK);}}
					if(x_pos<=80&&min>=1){
						buzzer=1;
						sec=min*60;
						start=1;
						menu=2;}
		}
	}
	 if(turn){
	   turn=0;
		 if(rota){
			if(min<60){
			 buzzer=1;
			 min+=1;
			 sprintf(str, "%.2d",min);		
			 ILI9341_Draw_Text(str, 65, 10, WHITE, 10, BLACK);
		   mode=0;}}
		 else if(!rota){
			if(min!=0){
			 buzzer=1;
			 min-=1;
		   sprintf(str, "%.2d",min);		
			 ILI9341_Draw_Text(str, 65, 10, WHITE, 10, BLACK);
		   mode=0;}}
	 }
}
void DisplayTimer(uint32_t x){
  		if(start==1||door==0){
				 if(pTimer==1){
				 ILI9341_Fill_Screen(BLACK);
				 ILI9341_Draw_Hollow_Rectangle_Coord(0, 0, 319, 239, WHITE);
				 ILI9341_Draw_Text("Remaining time", 15, 23, WHITE, 3, BLACK);
				 ILI9341_Draw_Rectangle(0, 160, 320, 1, WHITE);
				 ILI9341_Draw_Rectangle(160, 160, 1, 80, WHITE);	
				 ILI9341_Draw_Text("STOP", 180, 182, WHITE,4, BLACK);
				 switch (mode){
				   case 1: ILI9341_Draw_Rectangle(280, 189, 22, 22,RED);	  break;
				 	 case 2: ILI9341_Draw_Rectangle(280, 189, 22, 22,GREEN);	break;
					 case 3: ILI9341_Draw_Rectangle(280, 189, 22, 22,BLUE);   break;}
				 if(start==0)
							ILI9341_Draw_Text("START", 22, 182, WHITE,4, BLACK);
						else
							ILI9341_Draw_Text("PAUSE", 22, 182, WHITE,4, BLACK);	
				 pTimer=0;}		
				 
					if(sec==-1){
					   HAL_GPIO_WritePin(GPIOD, LED_RED_Pin, GPIO_PIN_RESET);
						 HAL_GPIO_WritePin(GPIOD, LED_GREEN_Pin, GPIO_PIN_RESET);
						 HAL_GPIO_WritePin(GPIOD, LED_BLUE_Pin, GPIO_PIN_RESET);
						 HAL_GPIO_WritePin(GPIOG, MOTOR_Pin, GPIO_PIN_SET);	
						 buzzer=2;
						 menu=0;
						 pTimer=1;
						 sec=0;
						 min=0;
						 start=0;}
					 else{
					 sprintf(str,"%02d:%02d",sec/60,sec%60);		  
					 ILI9341_Draw_Text(str, 15, 60, WHITE,10, BLACK);}
	}		
}
void TouchTimer(int x){
		 if(sec==0)
	  	 menu=0;
		 if(TP_Touchpad_Pressed()){
      if(TP_Read_Coordinates(position_array) == TOUCHPAD_DATA_OK){
					x_pos = position_array[0];
					y_pos = position_array[1];		
					HAL_Delay(50);	
					if(x_pos<=80&&y_pos<=160&&dbTimer==2){
						if(TP_Touchpad_Pressed())
							debounce1=1;
						else{		
							debounce1=0;
							debounce2=1;}
						if(debounce1==1){
							if(debounce2==1){
								buzzer=1;
								start=!start;
								debounce2=0;}}	
						if(start==1)
							ILI9341_Draw_Text("PAUSE", 22, 182, WHITE,4, BLACK);
						else
							ILI9341_Draw_Text("START", 22, 182, WHITE,4, BLACK);}
					if(x_pos<=80&&y_pos>=161&&dbTimer==2){
					   HAL_GPIO_WritePin(GPIOD, LED_RED_Pin, GPIO_PIN_RESET);
						 HAL_GPIO_WritePin(GPIOD, LED_GREEN_Pin, GPIO_PIN_RESET);
						 HAL_GPIO_WritePin(GPIOD, LED_BLUE_Pin, GPIO_PIN_RESET);
						 HAL_GPIO_WritePin(GPIOG, MOTOR_Pin, GPIO_PIN_SET);	
						 buzzer=1;
						 menu=0;
						 pTimer=1;
						 sec=0;
						 min=0;
						 start=0;}
			 }
		 }
		 else
			 dbTimer=2;
}

void CheckDoor(int x){
statedoor=HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7);
if (statedoor == GPIO_PIN_RESET){
	  door=1;
		logm=menu;	
		ILI9341_Fill_Screen(BLACK);
		ILI9341_Draw_Hollow_Rectangle_Coord(0, 0, 319, 239, WHITE);
		ILI9341_Draw_Text("The door", 66, 23, RED, 4, BLACK);
		ILI9341_Draw_Text("is", 138, 73, RED, 4, BLACK);
		ILI9341_Draw_Text("Open",113, 124, RED, 4, BLACK);
		ILI9341_Draw_Text("Please Close", 53, 200, WHITE, 3, BLACK);
		start=0;
		buzzer=3;}
	else{
	door=0;
	 if(logm==0||logm==1){
		 ILI9341_Fill_Screen(BLACK);
		 menu=0;}
	 if(logm==2){
		pTimer=1;
		DisplayTimer(sec);
		menu=2;
}
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){ 
	CheckDoor(1);}
void Write_Data(int x){
	 uint8_t Digit[2]={sec&0xff,sec>>8};
   Buffer[1]=menu;
	 Buffer[2]=mode;
	 Buffer[3]=Digit[0];
	 Buffer[4]=Digit[1];	
	 Buffer[5]=min;	
 for(int x=1; x<=5; x++){
	 HAL_I2C_Mem_Write(&hi2c1, 0xA0, x, 2, &Buffer[x], 1, 5); 
	 HAL_I2C_Mem_Write(&hi2c1, 0xA0,x, 2, &Buffer[x], 0, 5);
	 HAL_Delay(10);}
}
void Read_Data(int x){
	HAL_I2C_Mem_Read(&hi2c1, 0xA1,1, 2, &menu, 1, 5); 
	HAL_I2C_Mem_Read(&hi2c1, 0xA1,2, 2, &mode, 1, 5); 
	HAL_I2C_Mem_Read(&hi2c1, 0xA1,3, 2, &Digit[0], 1, 5); 
	HAL_I2C_Mem_Read(&hi2c1, 0xA1,4, 2, &Digit[1], 1, 5); 
	HAL_I2C_Mem_Read(&hi2c1, 0xA1,5, 2, &min, 1, 5); 
	sec = ((uint16_t)Digit[1]<< 8) |Digit[0];
}
void Buzzer(int x){
		 if(buzzer==1){
			HAL_GPIO_WritePin(GPIOD, BUZZER_Pin, GPIO_PIN_RESET);
			HAL_Delay(1);
			HAL_GPIO_WritePin(GPIOD, BUZZER_Pin, GPIO_PIN_SET);
			buzzer=0;}
		 if(buzzer==2){
			HAL_GPIO_WritePin(GPIOD, BUZZER_Pin, GPIO_PIN_RESET);
			HAL_Delay(1300);
			HAL_GPIO_WritePin(GPIOD, BUZZER_Pin, GPIO_PIN_SET);
			buzzer=0;}
		 if(buzzer==3){
			HAL_GPIO_WritePin(GPIOD, BUZZER_Pin, GPIO_PIN_RESET);
			HAL_Delay(50);
			HAL_GPIO_WritePin(GPIOD, BUZZER_Pin, GPIO_PIN_SET);
			buzzer=0;}	 
}
void CheckClear(int x){
	stateclear=HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6);	 
	if (stateclear == GPIO_PIN_RESET){
	 					 HAL_GPIO_WritePin(GPIOD, LED_RED_Pin, GPIO_PIN_RESET);
						 HAL_GPIO_WritePin(GPIOD, LED_GREEN_Pin, GPIO_PIN_RESET);
						 HAL_GPIO_WritePin(GPIOD, LED_BLUE_Pin, GPIO_PIN_RESET);
						 HAL_GPIO_WritePin(GPIOG, MOTOR_Pin, GPIO_PIN_SET);	
						 buzzer=1;
						 menu=0;
						 pTimer=1;
						 sec=0;
						 min=0;
						 start=0;}}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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

/* USER CODE BEGIN 4 */

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
