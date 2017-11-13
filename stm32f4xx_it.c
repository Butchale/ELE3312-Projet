/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

/* USER CODE BEGIN 0 */
#include "usart.h"
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

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
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
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
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
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
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
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
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles ADC1 global interrupt.
*/
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */
	if(idx_adc1 < 2048){
		data_volt[idx_adc1] = HAL_ADC_GetValue(&hadc1);
	}
	idx_adc1++;
  /* USER CODE END ADC_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC_IRQn 1 */

  /* USER CODE END ADC_IRQn 1 */
}

/**
* @brief This function handles EXTI line[9:5] interrupts.
*/
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
	int output[4]={GPIO_PIN_12,GPIO_PIN_13,GPIO_PIN_14,GPIO_PIN_15};
	int i =0;
	HAL_Delay(7);
	//int i =12;
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
	if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_8) == 0)
	{
		for(i = 0; i<4;i++){
			HAL_GPIO_WritePin(GPIOB,output[i] , GPIO_PIN_SET);
			HAL_Delay(7);
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_8)== 1){
				HAL_GPIO_WritePin(GPIOB,output[i] , GPIO_PIN_RESET);
				break;
			}
		}
		boutton =(1<<(4+i))+1;
		sprintf(buffer,"Bouton input: %d\r\n",boutton);
		HAL_UART_Transmit(&huart2,(uint8_t*) &buffer,50,10000);
		}
		if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_9) == 0){
			for(i = 0; i<4;i++){
				HAL_GPIO_WritePin(GPIOB,output[i] , GPIO_PIN_SET);
				HAL_Delay(7);
				if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_9)== 1){
					HAL_GPIO_WritePin(GPIOB,output[i] , GPIO_PIN_RESET);
					break;
			}
		}
		boutton =(1<<(4+i))+(1<<1);
		sprintf(buffer,"Bouton input: %d\r\n",boutton);
		HAL_UART_Transmit(&huart2,(uint8_t*) &buffer,50,10000);
		}
		if(boutton==18){
			if(freq > 1000){
				freq = freq + 1000;
				TIM3->ARR = (8400000/freq)-1;
				idx_adc1 = 0;
			}
		}
		if(boutton==66){
			if(freq < 100000){
				freq = freq - 1000;
				TIM3->ARR = (8400000/freq)-1;
				idx_adc1 = 0;
			}
		}
	/*sprintf(buffer,"GPIO input: %d%d%d%d\r\n",HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_11),HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10),HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_9),HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_8));
	HAL_UART_Transmit(&huart2,(uint8_t*) &buffer,50,10000);
	sprintf(buffer,"GPIO output: %d%d%d%d\r\n",HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15),HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14),HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13),HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12));
	HAL_UART_Transmit(&huart2,(uint8_t*) &buffer,50,10000);*/
  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */
	/*while(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_8) == 0){
		HAL_GPIO_WritePin(GPIOB, i , GPIO_PIN_SET);
		i++;
	}*/

	/*sprintf(buffer,"Boutton: 1\r\n");
	HAL_UART_Transmit(&huart2,(uint8_t*) &buffer,50,10000);
	sprintf(buffer,"GPIO input: %d%d%d%d\r\n",HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_11),HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10),HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_9),HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_8));
	HAL_UART_Transmit(&huart2,(uint8_t*) &buffer,50,10000);
	sprintf(buffer,"GPIO output: %d%d%d%d\r\n",HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15),HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14),HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13),HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12));
	HAL_UART_Transmit(&huart2,(uint8_t*) &buffer,50,10000);*/
  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
* @brief This function handles EXTI line[15:10] interrupts.
*/
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
	int i =0;
	int position[4]={GPIO_ODR_OD12_Msk, GPIO_ODR_OD13_Msk, GPIO_ODR_OD14_Msk, GPIO_ODR_OD15_Msk};
	GPIOB->ODR &= ~position[0];
	GPIOB->ODR &= ~position[1];
	GPIOB->ODR &= ~position[2];
	GPIOB->ODR &= ~position[3];
	HAL_Delay(7);
	if((GPIOC->IDR&GPIO_IDR_IDR_10) == 0){
		for(i = 0; i<3;i++){
			GPIOB->ODR |= position[i];
			HAL_Delay(7);
			if(((GPIOC->IDR&GPIO_IDR_IDR_10)>>10) == 1){
				GPIOB->ODR &= ~position[i];
				break;
			}
		}
		boutton =(1<<(4+i))+(1<<2);
		sprintf(buffer,"Bouton input: %d\r\n",boutton);
		HAL_UART_Transmit(&huart2,(uint8_t*) &buffer,50,10000);	
	}
		if((GPIOC->IDR&GPIO_IDR_IDR_11) == 0){
		for(i = 0; i<3;i++){
			GPIOB->ODR |= position[i];
			HAL_Delay(7);
			if(((GPIOC->IDR&GPIO_IDR_IDR_11)>>11) == 1){
				GPIOB->ODR &= ~position[i];
				break;
			}
		}
		boutton =(1<<(4+i))+(1<<3);
		sprintf(buffer,"Bouton input: %d\r\n",boutton);
		HAL_UART_Transmit(&huart2,(uint8_t*) &buffer,50,10000);	
	}
	if(boutton==20){
		if((duty/100.0) <	1){
			duty++;
			TIM3->CCR1 = (duty/100.0)*TIM3->ARR;
			idx_adc1 = 0;
		}
	}
	if(boutton==68){
		if((duty/100.0) > 0){
			duty--;
			TIM3->CCR1 = (duty/100.0)*TIM3->ARR;
			idx_adc1 = 0;
		}
	}
	 //GPIOC->ODR = 1<<12;
  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
