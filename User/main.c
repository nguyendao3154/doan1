/**
 ******************************************************************************
 * @file    main.c
 * @author  Makipos Co.,LTD.
 * Nguyen DD - Quan Vu
 * @version 1.0
 * @date    2019
 * @brief   
 ******************************************************************************/
/*******************************************************************************
 * Include
 ******************************************************************************/
#include "stm8s.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#ifdef _RAISONANCE_
#define PUTCHAR_PROTOTYPE int putchar(char c)
#define GETCHAR_PROTOTYPE int getchar(void)
#elif defined(_COSMIC_)
#define PUTCHAR_PROTOTYPE char putchar(char c)
#define GETCHAR_PROTOTYPE char getchar(void)
#else /* _IAR_ */
#define PUTCHAR_PROTOTYPE int putchar(int c)
#define GETCHAR_PROTOTYPE int getchar(void)
#endif /* _RAISONANCE_ */

#define Hot_Sensing_Port GPIOD
#define Hot_Sensing_Pin GPIO_PIN_2
#define Cold_Sensing_Port GPIOD
#define Cold_Sensing_Pin GPIO_PIN_3

#define Relay_Port GPIOA
#define Relay_Nong_Pin GPIO_PIN_1
#define Relay_Lanh_Pin GPIO_PIN_2

#define Hot_Input_Port GPIOC
#define Hot_Input_Pin GPIO_PIN_4
#define Cold_Input_Port GPIOD
#define Cold_Input_Pin GPIO_PIN_6

#define ADC_Hot_Sensing_Channel ADC1_CHANNEL_3
#define ADC_Cold_Sensing_Channel ADC1_CHANNEL_4
#define ADC_Hot_Input_Channel ADC1_CHANNEL_2
#define ADC_Cold_Input_Channel ADC1_CHANNEL_6

#define HOT_CHANGE_DELTA 50  // +- 5.0 C
#define COLD_CHANGE_DELTA 25 // +- 2.5 C

#define HOT_INPUT_DEFAULT 80
#define COLD_INPUT_DEFAULT 13
#define HOT_INPUT_MAX 90
#define HOT_INPUT_MIN 70
#define COLD_INPUT_MAX 10
#define COLD_INPUT_MIN 5

// ma lenh
#define CMD_TYPE_WRITE 0x01
#define CMD_TYPE_READ 0x02

// flash address
#define FLASH_ADD_HOT_INPUT 0x4000
#define FLASH_ADD_COLD_INPUT 0x4001

/*******************************************************************************
 * External Variables
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

bool Tim2_Flag = false;
uint8_t countTimNong, countTimLanh, countUART = 0;
bool FlagHotTemp = false, FlagColdTemp = false;
uint16_t Conversion_Hot_input_value = 0, Conversion_Cold_input_value = 0;
uint16_t Conversion_Hot_sensing_value = 0, Conversion_Cold_sensing_value = 0;
uint8_t lanh_input = 0, nong_input = 0;
uint16_t do_lanh, do_nong;
uint8_t countTimHot = 0, countTimCold = 0;

uint16_t ADC_Calculated_lanh[2] = {443, 770}; // tuong ung voi 2.5 va 32.5
uint16_t ADC_Calculated_nong[2] = {374, 537};

#define TABLE_NUM_COLD_INPUT 6
#define TABLE_NUM_HOT_INPUT 5
uint16_t ADC_Calculated_cold_input[TABLE_NUM_COLD_INPUT] = {0, 300, 550, 662, 745, 755};
uint16_t ADC_Calculated_hot_input[TABLE_NUM_HOT_INPUT] = {0, 450, 611, 743, 765};
#define MAX_ADC_COLD (ADC_Calculated_cold_input[5] + 20)
#define MIN_ADC_COLD (ADC_Calculated_cold_input[0] - 0)
#define MAX_ADC_HOT (ADC_Calculated_hot_input[4] + 20)
#define MIN_ADC_HOT (ADC_Calculated_hot_input[0] - 0)

uint8_t degree_set_cold_input[TABLE_NUM_COLD_INPUT] = {7, 9, 11, 13, 15, 17};
uint8_t degree_set_hot_input[TABLE_NUM_HOT_INPUT] = {70, 75, 80, 85, 90};

uint8_t rxBuffer[5];
uint8_t rxCounter = 0;
uint8_t rxBufferSize;

uint8_t txCounter = 0;
uint8_t txBuffer[5];
uint8_t txBufferSize;

uint8_t TxBufferSize3;
uint8_t TxCounter3 = 0;
uint8_t TxBuffer3[] = "nhap vao 5 byte hex\nstart 01\ntap lenh: 01 receive, 02 transmit\ninput nong 70-90 decimal\ninput lanh 5-10 decimal\nstop 01\n";
bool s_needSendTemp = false;
bool s_needSendInput = true;
uint8_t s_sendInputTime = 0;
#define TIME_REPEAT_SEND_INPUT 2
/*******************************************************************************
 * Macro
 ******************************************************************************/
#define DatNhietDoBangTro
/*******************************************************************************
 * Local Function Prototypes
 ******************************************************************************/

void processMessage(void);

/*******************************************************************************
 * Local Function
 ******************************************************************************/

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
void UART_Init(void)
{
  UART1_DeInit();
  UART1_Init((uint32_t)9600, UART1_WORDLENGTH_8D, UART1_STOPBITS_1, UART1_PARITY_NO,
             UART1_SYNCMODE_CLOCK_DISABLE, UART1_MODE_TXRX_ENABLE);
}
void GPIO_Config(void)
{
  GPIO_Init(Hot_Sensing_Port, Hot_Sensing_Pin, GPIO_MODE_IN_FL_NO_IT);
  GPIO_Init(Cold_Sensing_Port, Cold_Sensing_Pin, GPIO_MODE_IN_FL_NO_IT);
  GPIO_Init(Hot_Input_Port, Hot_Input_Pin, GPIO_MODE_IN_FL_NO_IT);
  GPIO_Init(Cold_Input_Port, Cold_Input_Pin, GPIO_MODE_IN_FL_NO_IT);
  GPIO_Init(Relay_Port, Relay_Nong_Pin, GPIO_MODE_OUT_PP_HIGH_FAST);
  GPIO_Init(Relay_Port, Relay_Lanh_Pin, GPIO_MODE_OUT_PP_HIGH_FAST);
  GPIO_WriteLow(Relay_Port, Relay_Nong_Pin);
  GPIO_WriteLow(Relay_Port, Relay_Lanh_Pin);
  GPIO_Init(GPIOD, GPIO_PIN_4, GPIO_MODE_OUT_PP_HIGH_FAST);
}
void Scan_All_ADC_Channels(void)
{

  ADC1_Init(ADC1_CONVERSIONMODE_SINGLE, ADC_Cold_Sensing_Channel, ADC1_PRESSEL_FCPU_D18,
            ADC1_EXTTRIG_TIM, DISABLE, ADC1_ALIGN_RIGHT, ADC1_SCHMITTTRIG_CHANNEL0,
            DISABLE);
  ADC1_StartConversion();
  Conversion_Cold_sensing_value = ADC1_GetConversionValue();
  ADC1_Init(ADC1_CONVERSIONMODE_SINGLE, ADC_Hot_Sensing_Channel, ADC1_PRESSEL_FCPU_D18,
            ADC1_EXTTRIG_TIM, DISABLE, ADC1_ALIGN_RIGHT, ADC1_SCHMITTTRIG_CHANNEL0,
            DISABLE);
  ADC1_StartConversion();
  Conversion_Hot_sensing_value = ADC1_GetConversionValue();
#ifdef DatNhietDoBangTro
  ADC1_Init(ADC1_CONVERSIONMODE_SINGLE, ADC_Hot_Input_Channel, ADC1_PRESSEL_FCPU_D18,
            ADC1_EXTTRIG_TIM, DISABLE, ADC1_ALIGN_RIGHT, ADC1_SCHMITTTRIG_CHANNEL0,
            DISABLE);
  ADC1_StartConversion();
  Conversion_Hot_input_value = ADC1_GetConversionValue();
  ADC1_Init(ADC1_CONVERSIONMODE_SINGLE, ADC_Cold_Input_Channel, ADC1_PRESSEL_FCPU_D18,
            ADC1_EXTTRIG_TIM, DISABLE, ADC1_ALIGN_RIGHT, ADC1_SCHMITTTRIG_CHANNEL0,
            DISABLE);
  ADC1_StartConversion();
  Conversion_Cold_input_value = ADC1_GetConversionValue();
#endif
}

void printTempHex(void)
{
  // char sendBuff[20];
  // sprintf(sendBuff,"t_h:%d,t_c:%d\n",do_nong,do_lanh);
  // txBufferSize = strlen(sendBuff);

  uint8_t sendBuff[10];
  sendBuff[0] = 0x11;
  sendBuff[1] = 0x01;
  sendBuff[2] = (uint8_t)(do_nong >> 8);
  sendBuff[3] = (uint8_t)(do_nong & 0x00FF);
  sendBuff[4] = (uint8_t)(do_lanh >> 8);
  sendBuff[5] = (uint8_t)(do_lanh & 0x00FF);
  sendBuff[6] = 0x22;
  txBufferSize = 7;
  while (txBufferSize--)
  {
    while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET)
    {
    }
    /* Write one byte in the UART1 Transmit Data Register */

    UART1_SendData8(sendBuff[txCounter++]);
  }
  txCounter = 0;
}

void printInputHex(void)
{
  uint8_t sendBuff[10];
  sendBuff[0] = 0x11;
  sendBuff[1] = 0x02;
  sendBuff[2] = nong_input;
  sendBuff[3] = lanh_input;
  sendBuff[4] = 0x33;
  sendBuff[5] = 0x33;
  sendBuff[6] = 0x22;
  txBufferSize = 7;
  while (txBufferSize--)
  {
    while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET)
    {
    }
    /* Write one byte in the UART1 Transmit Data Register */

    UART1_SendData8(sendBuff[txCounter++]);
  }
  txCounter = 0;
}

#ifdef DatNhietDoBangTro

void dat_nhiet_do(void)
{
  static uint8_t newNong = 0, newLanh = 0;
  static uint8_t cntNong = 0, cntLanh = 0;
  uint8_t tmpNong = nong_input;
  uint8_t tmpLanh = lanh_input;
  // kenh nong
  for (int j = 0; j < (TABLE_NUM_HOT_INPUT - 1); j++)
  {
    if ((Conversion_Hot_input_value >= ADC_Calculated_hot_input[j] && Conversion_Hot_input_value <= ADC_Calculated_hot_input[j + 1]))
    {
      if ((Conversion_Hot_input_value - ADC_Calculated_hot_input[j]) < (ADC_Calculated_hot_input[j + 1] - Conversion_Hot_input_value))
      {
        tmpNong = degree_set_hot_input[j];
      }
      else
      {
        tmpNong = degree_set_hot_input[j + 1];
      }
    }
  }
  if ((Conversion_Hot_input_value > MIN_ADC_HOT) && (Conversion_Hot_input_value < ADC_Calculated_hot_input[0]))
  {
    tmpNong = degree_set_hot_input[0];
  }
  if ((Conversion_Hot_input_value < MAX_ADC_HOT) && (Conversion_Hot_input_value > ADC_Calculated_hot_input[TABLE_NUM_HOT_INPUT - 1]))
  {
    tmpNong = degree_set_hot_input[TABLE_NUM_HOT_INPUT - 1];
  }
  if ((Conversion_Hot_input_value > MAX_ADC_HOT) || (Conversion_Hot_input_value < MIN_ADC_HOT)) // hong bien tro
  {
    tmpNong = HOT_INPUT_DEFAULT;
  }
  // kenh lanh
  for (int i = 0; i < (TABLE_NUM_COLD_INPUT - 1); i++)
  {
    if ((Conversion_Cold_input_value >= ADC_Calculated_cold_input[i] && Conversion_Cold_input_value <= ADC_Calculated_cold_input[i + 1]))
    {
      if ((Conversion_Cold_input_value - ADC_Calculated_cold_input[i]) < (ADC_Calculated_cold_input[i + 1] - Conversion_Cold_input_value))
      {
        tmpLanh = degree_set_cold_input[i];
      }
      else
      {
        tmpLanh = degree_set_cold_input[i + 1];
      }
    }
  }

  if ((Conversion_Cold_input_value > MIN_ADC_COLD) && (Conversion_Cold_input_value < ADC_Calculated_cold_input[0]))
  {
    tmpLanh = degree_set_cold_input[0];
  }
  if ((Conversion_Cold_input_value < MAX_ADC_COLD) && (Conversion_Cold_input_value > ADC_Calculated_cold_input[TABLE_NUM_COLD_INPUT - 1]))
  {
    tmpLanh = degree_set_cold_input[TABLE_NUM_COLD_INPUT - 1];
  }
  if ((Conversion_Cold_input_value > MAX_ADC_COLD) || (Conversion_Cold_input_value < MIN_ADC_COLD)) // hong bien tro
  {
    tmpLanh = COLD_INPUT_DEFAULT;
  }

  if (nong_input == 0) // lan dau khoi dong
  {
    nong_input = tmpNong;
    lanh_input = tmpLanh;
    newLanh = tmpLanh;
    newNong = tmpNong;
    // printInputHex();
    s_needSendInput = true;
    s_sendInputTime = TIME_REPEAT_SEND_INPUT;
  }
  // kenh nong
  if (newNong == tmpNong)
  {
    cntNong++;
    if (cntNong > 10)
    {
      cntNong = 0;
      if (nong_input != newNong)
      {
        nong_input = newNong;
        s_needSendInput = true;
        s_sendInputTime = TIME_REPEAT_SEND_INPUT;
      }
    }
  }
  else
  {
    cntNong = 0;
    newNong = tmpNong;
  }
  // kenh lanh
  if (newLanh == tmpLanh)
  {
    cntLanh++;
    if (cntLanh > 10)
    {
      cntLanh = 0;
      if (lanh_input != newLanh)
      {
        lanh_input = newLanh;
        s_needSendInput = true;
        s_sendInputTime = TIME_REPEAT_SEND_INPUT;
      }
    }
  }
  else
  {
    cntLanh = 0;
    newLanh = tmpLanh;
  }

  // // neu co thay doi
  // if ((tmpNong != nong_input) || (tmpLanh != lanh_input))
  // {
  //   nong_input = tmpNong;
  //   lanh_input = tmpLanh;
  //   printInputHex();
  // }
}
#endif

void doc_nhiet_do(void)
{
  uint16_t tmpNong = do_nong;
  uint16_t tmpLanh = do_lanh;

  if (Conversion_Hot_sensing_value > ADC_Calculated_nong[1])
  { // lon hon 95 do
    tmpNong = 960;
  }
  else if (Conversion_Hot_sensing_value < ADC_Calculated_nong[0] && Conversion_Hot_sensing_value > 10)
  { // nho hon 65 do
    tmpNong = 640;
  }
  else if (Conversion_Hot_sensing_value < 10)
  { // khi ko cam day do nhiet do
    tmpNong = 960;
  }
  else
  {
    tmpNong = (uint16_t)(((uint32_t)Conversion_Hot_sensing_value - (uint32_t)ADC_Calculated_nong[0]) * (uint32_t)300 / ((uint32_t)ADC_Calculated_nong[1] - (uint32_t)ADC_Calculated_nong[0])) + 650;
  }

  if (Conversion_Cold_sensing_value > ADC_Calculated_lanh[1])
  { // lon hon 32.5 do
    tmpLanh = 330;
  }
  else if (Conversion_Cold_sensing_value < ADC_Calculated_lanh[0])
  { // nho hon 2 do
    tmpLanh = 10;
  }
  else
  {
    tmpLanh = (uint16_t)((uint32_t)Conversion_Cold_sensing_value - (uint32_t)ADC_Calculated_lanh[0]) * (uint32_t)300 / ((uint32_t)ADC_Calculated_lanh[1] - (uint32_t)ADC_Calculated_lanh[0]) + 25;
  }
  if ((tmpNong != do_nong) || (tmpLanh != do_lanh))
  {
    do_nong = tmpNong;
    do_lanh = tmpLanh;
    s_needSendTemp = true;
  }
}
void can_bang_nhiet_do(void)
{
  if (do_nong < (nong_input * 10 - HOT_CHANGE_DELTA))
  {
    countTimNong++;
    if (countTimNong == 30)
    {
      GPIO_WriteHigh(Relay_Port, Relay_Nong_Pin);
      countTimNong = 0;
    }
  }
  else if (do_nong > (nong_input * 10 + HOT_CHANGE_DELTA))
  {
    countTimNong++;
    if (countTimNong == 30)
    {
      GPIO_WriteLow(Relay_Port, Relay_Nong_Pin);
      countTimNong = 0;
    }
  }
  else
  {
    countTimNong = 0;
  }
  if (do_lanh > (lanh_input * 10 + COLD_CHANGE_DELTA))
  {
    countTimLanh++;
    if (countTimLanh == 30)
    {
      GPIO_WriteHigh(Relay_Port, Relay_Lanh_Pin);
      countTimLanh = 0;
    }
  }
  else if (do_lanh < (lanh_input * 10 - COLD_CHANGE_DELTA))
  {
    countTimLanh++;
    if (countTimLanh == 30)
    {
      GPIO_WriteLow(Relay_Port, Relay_Lanh_Pin);
      countTimLanh = 0;
    }
  }
  else
  {
    countTimLanh = 0;
  }
}
uint32_t timeout = 0;

#ifndef DatNhietDoBangTro
void nhan_input(void)
{
  if (UART1_GetFlagStatus(UART1_FLAG_RXNE) == SET)
  {
    while (1)
    {
      timeout = 0;
      while (UART1_GetFlagStatus(UART1_FLAG_RXNE) == RESET && (++timeout < 800000))
        ;
      if (timeout >= 800000)
      {
        rxCounter = 0;
        break;
      }
      rxBuffer[rxCounter++] = UART1_ReceiveData8();
      if (rxCounter == 5)
      {
        processMessage();
        rxCounter = 0;
        break;
      }
    }
  }
  else
  {
    rxCounter = 0;
  }
}
void gui_thong_bao_dung(void)
{
  memcpy(txBuffer, rxBuffer, 5);
  txBuffer[1] = 0x11;
  txBufferSize = sizeof(txBuffer);
  while (txBufferSize--)
  {
    while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET)
    {
    }
    /* Write one byte in the UART1 Transmit Data Register */

    UART1_SendData8(txBuffer[txCounter++]);
  }
  txCounter = 0;
}
void gui_thong_bao_sai(void)
{
  memcpy(txBuffer, rxBuffer, 5);
  txBuffer[1] = 0x21;
  txBufferSize = sizeof(txBuffer);
  while (txBufferSize--)
  {
    while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET)
    {
    }
    /* Write one byte in the UART1 Transmit Data Register */

    UART1_SendData8(txBuffer[txCounter++]);
  }
  txCounter = 0;
}

void processWriteCmd()
{
  // check input hot trong dai
  if (rxBuffer[2] >= HOT_INPUT_MIN && rxBuffer[2] <= HOT_INPUT_MAX)
  {
    if (rxBuffer[2] != nong_input)
    {
      nong_input = rxBuffer[2];
      FLASH_ProgramByte(FLASH_ADD_HOT_INPUT, nong_input);
    }
  }
  else // input hot ngoai dai
  {
    gui_thong_bao_sai();
    return;
  }
  // check input cold trong dai
  if (rxBuffer[3] >= COLD_INPUT_MIN && rxBuffer[3] <= COLD_INPUT_MAX)
  {
    if (rxBuffer[3] != lanh_input)
    {
      lanh_input = rxBuffer[3];
      FLASH_ProgramByte(FLASH_ADD_COLD_INPUT, lanh_input);
    }
  }
  else // input cold ngoai dai
  {
    gui_thong_bao_sai();
    return;
  }
  // kiem tra du lieu trong flash co trung voi input nong lanh ko?
  if (nong_input != FLASH_ReadByte(FLASH_ADD_HOT_INPUT) || lanh_input != FLASH_ReadByte(FLASH_ADD_COLD_INPUT))
  {
    gui_thong_bao_sai();
    return;
  }
  // neu khong co loi gi
  gui_thong_bao_dung();
  return;
}

void processReadCmd()
{
  memcpy(txBuffer, rxBuffer, 5);
  txBuffer[1] = 0x00;
  txBuffer[2] = nong_input;
  txBuffer[3] = lanh_input;
  txBufferSize = sizeof(txBuffer);
  while (txBufferSize--)
  {
    while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET)
    {
    }
    /* Write one byte in the UART1 Transmit Data Register */

    UART1_SendData8(txBuffer[txCounter++]);
  }
  txCounter = 0;
}
void processMessage(void)
{
  //check start bit & stop bit
  if (rxBuffer[0] == 0xbd && rxBuffer[4] == 0xed)
  {
    switch (rxBuffer[1])
    {
    case CMD_TYPE_WRITE:
    {
      processWriteCmd();
      break;
    }
    case CMD_TYPE_READ:
    {
      processReadCmd();
      break;
    }
    default:
      break;
    }
  }
  else // start bit or stop bit failse
  {
    gui_thong_bao_sai();
    return;
  }
}
void doc_flash(void)
{
  /* Define FLASH programming time */
  FLASH_SetProgrammingTime(FLASH_PROGRAMTIME_STANDARD);

  /* Unlock Data memory */
  FLASH_Unlock(FLASH_MEMTYPE_DATA);

  /* Read a byte at a specified address */
  if (FLASH_ReadByte(FLASH_ADD_HOT_INPUT) != 0)
  {
    nong_input = FLASH_ReadByte(FLASH_ADD_HOT_INPUT);
  }
  else
    nong_input = HOT_INPUT_DEFAULT;
  if (FLASH_ReadByte(FLASH_ADD_COLD_INPUT) != 0)
  {
    lanh_input = FLASH_ReadByte(FLASH_ADD_COLD_INPUT);
  }
  else
    lanh_input = COLD_INPUT_DEFAULT;
  /* Program complement value (of previous read byte) at previous address + 1 */
}
#endif
void main(void)
{
  Clock_Init();
  Timer_Init();
#ifndef DatNhietDoBangTro
  doc_flash();
#endif
  GPIO_Config();
  UART_Init();
  TxBufferSize3 = (sizeof(TxBuffer3) - 1);
  while (TxBufferSize3--)
  {
    while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET)
    {
    }
    /* Write one byte in the UART1 Transmit Data Register */
    UART1_SendData8(TxBuffer3[TxCounter3++]);
  }
  TxCounter3 = 0;
  uint8_t timeCnt = 0;
  /* Infinite loop */
  while (1)
  {

    if (Tim2_Flag == true)
    {
      Scan_All_ADC_Channels();
      GPIO_WriteReverse(GPIOD,GPIO_PIN_4);
#ifdef DatNhietDoBangTro
      dat_nhiet_do();
#endif
      doc_nhiet_do();
      can_bang_nhiet_do();
      if (timeCnt++ == 5)
      {
        if (s_needSendTemp)
        {
          s_needSendTemp = false;
          printTempHex();
        }
      }
      if (timeCnt >= 10)
      {
        timeCnt = 0;
        if (s_sendInputTime)
        {
          printInputHex();
          s_sendInputTime --;
          s_needSendInput = false;
        }
      }

      Tim2_Flag = false;
    }
#ifndef DatNhietDoBangTro
    nhan_input();
#endif
  }
}

PUTCHAR_PROTOTYPE
{
  /* Write a character to the UART1 */
  UART1_SendData8(c);
  /* Loop until the end of transmission */
  while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET)
    ;

  return (c);
}

/**
  * @brief Retargets the C library scanf function to the USART.
  * @param None
  * @retval char Character to Read
  */
GETCHAR_PROTOTYPE
{
#ifdef _COSMIC_
  char c = 0;
#else
  int c = 0;
#endif
  /* Loop until the Read data register flag is SET */
  while (UART1_GetFlagStatus(UART1_FLAG_RXNE) == RESET)
    ;
  c = UART1_ReceiveData8();
  return (c);
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

/*******************************************************************************
 * Public Function
 ******************************************************************************/

/***********************************************/