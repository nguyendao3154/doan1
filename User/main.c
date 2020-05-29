/******************************************************************************
*
* Truong Dai hoc Bach Khoa Ha Noi.
* Vien Dien SEE.
* ALL RIGHTS RESERVED.
*
***************************************************************************/
/**
 *
 * @file      main.c
 *
 * @author    Nguyen Dao
 * 
 * @version   1.0
 * 
 * @date      May-23-2020
 * 
 * @brief     Brief description of the file
 *
 * Detailed Description of the file. If not used, remove the separator above.
 *
 */
/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define LED_PORT GPIOD
#define LED_PIN GPIO_PIN_4

#define BUZZER_PORT (EXTI_PORT_GPIOD)
#define BUZZER_PIN GPIO_PIN_5

#define BUZZER_TIMEOUT 10
#define LED_TIMEOUT 5

bool tu_flag = false;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void Clock_Init(void)
{
  CLK_DeInit();
  CLK_SYSCLKConfig(CLK_PRESCALER_HSIDIV1);
  CLK_ClockSwitchConfig(CLK_SWITCHMODE_AUTO, CLK_SOURCE_HSI, DISABLE, CLK_CURRENTCLOCKSTATE_DISABLE);
}

void Timer_Init(void)
{
  GPIO_Init(GPIOD, GPIO_PIN_0, GPIO_MODE_OUT_PP_HIGH_FAST);
  TIM2_TimeBaseInit(TIM2_PRESCALER_512, 3124);
  TIM2_ClearFlag(TIM2_FLAG_UPDATE);
  TIM2_ITConfig(TIM2_IT_UPDATE, ENABLE);
  enableInterrupts();
  TIM2_Cmd(ENABLE);
}

void GPIO_Config(void)
{
  GPIO_Init(LED_PORT, LED_PIN, GPIO_MODE_OUT_PP_HIGH_FAST);
  GPIO_WriteHigh(LED_PORT, LED_PIN);
}

void Interrupt_init(void)
{
  EXTI_SetExtIntSensitivity(BUZZER_PORT, EXTI_SENSITIVITY_FALL_ONLY);
  EXTI_SetTLISensitivity(EXTI_TLISENSITIVITY_FALL_ONLY);

  enableInterrupts();
}
void task_interrupt(void)
{
  if (tu_flag)
  {
    // do sth
    tu_flag = false;
  }
}
void main(void)
{
  Clock_Init();
  Timer_Init();
  GPIO_Config();
  Interrupt_init();
  /* Infinite loop */
  while (1)
  {
    task_interrupt();
  }
}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(u8 *file, u32 line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
