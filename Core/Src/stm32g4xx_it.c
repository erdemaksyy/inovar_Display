/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//extern char rxBuf[20];

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim1;

extern uint32_t rxBuff;
extern uint8_t buffDisplay[99];
extern uint16_t buffCounter;
extern int q,w,e,r,g;
extern int z,x,c,v;
extern uint32_t sayi;
extern uint8_t gelenData_24[4];
extern uint8_t Sifrem[4];

extern float sayi_;
extern int i,m,u;
extern char buff[100];
extern char buff1[100];
extern  uint8_t Sifrem1[4];

static uint8_t dataread[4];

bool vitrinac = false;

bool vitrinsifre = false;

bool sifredegis = false;
//extern uint8_t gelen_mesaj_kutusu[8];
//extern uint32_t x;
//extern int counter;
//
//int v,sifrem;
//
//extern uint8_t r;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM1 update interrupt and TIM16 global interrupt.
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */




  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
  HAL_UART_Receive_IT(&huart2, &rxBuff,1);

//  if(buffCounter == 0 && rxBuff != 0)
//	  return;
  buffDisplay[buffCounter++]=rxBuff;

	 if(rxBuff == 0XBA ){


		 if(buffDisplay[5] == 'O' && buffDisplay[6] == 'P' && buffDisplay[7] == 'E' && buffDisplay[8] == 'N')
		 {
			 vitrinac = true;

			 NEXTION_SendString("page ","4");
		 }
		 else if(buffDisplay[5] == 'P' && buffDisplay[6] == 'S' && buffDisplay[7] == 'W' && vitrinac)
		 {
			q = buffDisplay[8]-48;
			w = buffDisplay[9]-48;
			e = buffDisplay[10]-48;
			r = buffDisplay[11]-48;

			 Sifrem1[0] = q,
			 Sifrem1[1] = w,
			 Sifrem1[2] = e,
			 Sifrem1[3] = r;
			for (m=0; m<sizeof(Sifrem1); m++) {
			sprintf(&buff[m],"%d",Sifrem1[m]);
			}
			m = atoi(buff);

	 		vitrinac = false;

	 		if(m == i)
	 		{
	 			NEXTION_SendString("page ","2");
	 		}
	 		else
	 		{

	 			NEXTION_SendString("page ","1");


	 		}

		 }
		 else if(buffDisplay[5] == 'S' && buffDisplay[6] == 'T' && buffDisplay[7] == 'N' && buffDisplay[8] == 'G')
		 {

			 NEXTION_SendString("page ","5");


		 }
		 else if(buffDisplay[5] == 'C' && buffDisplay[6] == 'L' && buffDisplay[7] == 'S' && buffDisplay[8] == 'E')
		 {
			 //sifreOlustur = true;
			 NEXTION_SendString("page ","2");
		 }
		 else if(buffDisplay[5] == 'P' && buffDisplay[6] == 'S' && buffDisplay[7] == 'W' && buffDisplay[8] == 'C')
		 {

			 vitrinsifre=true;
			 if(sifredegis==true)
			 {
				 vitrinsifre=false;
			 }

			 NEXTION_SendString("page ","4");

		 }
		 else if(buffDisplay[5] == 'P' && buffDisplay[6] == 'S' && buffDisplay[7] == 'W' && vitrinsifre==true)
		 		 {
			 //sifredegis == true;

 			 q = buffDisplay[8]-48;
 			 w = buffDisplay[9]-48;
 			 e = buffDisplay[10]-48;
 			 r = buffDisplay[11]-48;

 			 Sifrem[0] = q,
 			 Sifrem[1] = w,
 			 Sifrem[2] = e,
 			 Sifrem[3] = r;

 		    for (i=0; i<sizeof(Sifrem); i++) {
 		        sprintf(&buff[i],"%d",Sifrem[i]);
 		    }
 		    i = atoi(buff);

			//ee_write(0, 4, i);
			ee_read(0,4,dataread);
			if(i!=1100)
			{
				vitrinsifre=false;
				sifredegis=true;
				//ee_write(0, 4, i);
			//NEXTION_SendString("page ","1");
				NEXTION_SendString("page ","6");
			}

		 		 }
		 else if(buffDisplay[5] == 'P' && buffDisplay[6] == 'S' && buffDisplay[7] == 'W' && sifredegis==true)
				 		 {


		 			 q = buffDisplay[8]-48;
		 			 w = buffDisplay[9]-48;
		 			 e = buffDisplay[10]-48;
		 			 r = buffDisplay[11]-48;

		 			 Sifrem[0] = q,
		 			 Sifrem[1] = w,
		 			 Sifrem[2] = e,
		 			 Sifrem[3] = r;

		 		    for (u=0; u<sizeof(Sifrem); u++) {
		 		        sprintf(&buff[u],"%d",Sifrem[u]);
		 		    }
		 		    u = atoi(buff);

					//ee_write(0, 4, i);
					ee_read(0,4,dataread);



					if(u==i)
					{
						NEXTION_SendString("page ","7");
						 sifredegis = false;
						 vitrinsifre = true;
						//ee_write(0, 4, i);
				     NEXTION_SendString("page ","4");


					}
					else
					{
						NEXTION_SendString("page ","1");
					}

				 		 }

		 else if(buffDisplay[5] == 'S' && buffDisplay[6] == 'T' && buffDisplay[7] == 'R' && buffDisplay[8] == 'T')
				 {

					 NEXTION_SendString("page ","3");

				 }
		 else if(buffDisplay[5] == 'S' && buffDisplay[6] == 'T' && buffDisplay[7] == 'O' && buffDisplay[8] == 'P')
				 {

					 NEXTION_SendString("page ","1");

				 }


		 buffCounter=0;
		 memset(buffDisplay,'0',sizeof(buffDisplay));

	 }
//  v = atoll(rxBuf);

  /* USER CODE END USART2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
