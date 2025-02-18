/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * 				-serial port settings: flow control must be xon/xoff
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


//#define USE_BLOCKING_US_DELAY

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */
uint8_t rxbuffer[RXBUFFSIZE] = {0};
uint8_t rxbufend = 0;
uint8_t rxflagbyte = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

extern USBD_HandleTypeDef hUsbDeviceFS;
extern uint8_t host_com_port_open;

uint8_t volatile us_delay_flag = 0;

volatile uint8_t btn = 0;

extern CP stepper_pos;
extern volatile uint8_t switches;
volatile uint8_t buttons;

volatile uint64_t num_of_cmd_line = 1;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

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
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  stepper_pos.movespeed |= toolspeed_7;
  STSPIN220_init(&stepper_pos, stepsizeX1_4, stepsizeY1_4, stepsizeZ1_4);

  __enable_irq();

  LL_mDelay(500);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  get_limit_sw_state();

  gotozero(&stepper_pos);

#ifdef DEBUG
	/*USB->CNTR |= USB_CNTR_PDWN;
	LL_mDelay(100);
	USB->CNTR &= (~USB_CNTR_PDWN);

	USB->CNTR |= USB_CNTR_FRES;
	LL_mDelay(100);
	USB->CNTR &= (~USB_CNTR_FRES);*/
   /*USBD_Stop(&hUsbDeviceFS);
   LL_mDelay(100);
   USBD_Start(&hUsbDeviceFS);*/
#endif

  while(host_com_port_open == 0)//wait for the CDC_Control_FS() function to be called with "CDC_SET_CONTROL_LINE_STATE" command
  {
	  LL_mDelay(250);
  }
  msDelay(20);
  uint8_t str[] = {"Connected to PC"};
  ext_brd_transmit_string(PrintInfo_cmd, str, sizeof(str));

  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  USBD_CDC_TransmitPacket(&hUsbDeviceFS);

  uint8_t cmddone[1] = {CMD_DONE_CHAR};
  rxflagbyte = 0;
  rxbufend = 0;


  while(1)
  {
	while(rxflagbyte == 0)
	{
		__NOP();
	}
	/*while(rxflagbyte == 0) //amíg nem jön semmi a com porton keresztül, itt ciklik, a gombokkal lehet irányítani
	{
		__NOP();

		if(buttons & xbtnPbitMSK)	{ manualXdir = FORWARD;}
		else
		{
			if(buttons & xbtnNbitMSK)	{ manualXdir = BACKWARD;}
			else{ manualXdir = 0;}
		}
		if(buttons & ybtnPbitMSK)	{ manualYdir = FORWARD;}
		else
		{
			if(buttons & ybtnNbitMSK)	{ manualYdir = BACKWARD;}
			else{ manualYdir = 0;}
		}
		if(manualXdir || manualYdir)
		{
			stepxyz(abs(manualXdir)?1:0, manualXdir, abs(manualYdir)?1:0, manualYdir, 0, 0, &stepper_pos);
		}
	}*/

	switch(rxbuffer[0])
	{
		//initialize
		case 'I':	//the command only can be the "IN"
		case 'i':	stepper_pos.movespeed |= toolspeed_7;
					gotozero(&stepper_pos);
					msDelay(100);
					break;
		//set move type and move
		case 'P':
		case 'p':	switch(rxbuffer[1])
					{
						case 'A':
						case 'a':	stepper_pos.curr_state &= ~RelativeStep_MSK;
									stepper_pos.curr_state |= AbsoluteStep_MSK;
									break;

						case 'R':
						case 'r':	stepper_pos.curr_state &= ~AbsoluteStep_MSK;
									stepper_pos.curr_state |= RelativeStep_MSK;
									break;

						case 'U':
						case 'u':	stepper_pos.curr_state &= ~FeedMove;
									stepper_pos.curr_state |= RapidMove;
									break;

						case 'D':
						case 'd':	stepper_pos.curr_state &= ~RapidMove;
									stepper_pos.curr_state |= FeedMove;
									break;

					}
					if(rxbufend>3)//ha a command tartalmaz értékeket is akkor legalább 8 lesz a hossza pontosvesszővel együtt: PD0,0,0;
					{
						eval_and_execute_plot_cmd(rxbuffer, rxbufend, &stepper_pos);
					}
					break;
		//set toolspeed
		case 'v':
		case 'V':	if( (rxbuffer[1]=='s') || (rxbuffer[1]=='S') )
					{
						switch(rxbuffer[2])
						{
							case '0':	stepper_pos.movespeed = 0;
										stepper_pos.movespeed |= toolspeed_0;
										break;

							case '1':	stepper_pos.movespeed = 0;
										stepper_pos.movespeed |= toolspeed_1;
										break;

							case '2':	stepper_pos.movespeed = 0;
										stepper_pos.movespeed |= toolspeed_2;
										break;

							case '3':	stepper_pos.movespeed = 0;
										stepper_pos.movespeed |= toolspeed_3;
										break;

							case '4':	stepper_pos.movespeed = 0;
										stepper_pos.movespeed |= toolspeed_4;
										break;

							case '5':	stepper_pos.movespeed = 0;
										stepper_pos.movespeed |= toolspeed_5;
										break;

							case '6':	stepper_pos.movespeed = 0;
										stepper_pos.movespeed |= toolspeed_6;
										break;

							default:
							case '7':	stepper_pos.movespeed = 0;
										stepper_pos.movespeed |= toolspeed_7;
										break;
						}
					}
					break;
		//send filename to LCD
		case 't':
		case 'T':	if( (rxbuffer[1]=='x') || (rxbuffer[1]=='X') )
					{
						msDelay(100);
						rxbuffer[rxbufend]=0;//az utolsó elemet lecseréljük lezáró nullára hogy a string biztos standard legyen
						ext_brd_transmit_string(PrintFilenameTXT_cmd, &rxbuffer[2], rxbufend-1);
						rxbuffer[rxbufend]=CMD_DONE_CHAR;//visszaállítás
					}
					break;
		//get current pos and send to computer
		case 'g':
		case 'G':	if( (rxbuffer[1]=='p') || (rxbuffer[1]=='P') )
					{
						char numstr[10] = {0};
						uint8_t len;

						itoa(stepper_pos.current_pos_x, numstr, 10);
						len=strlen(numstr);
						numstr[len+1] = ';';
						numstr[len+2] = 0;
						CDC_Transmit_FS((uint8_t*)numstr, len+2);
						msDelay(10);

						itoa(stepper_pos.current_pos_y, numstr, 10);
						len=strlen(numstr);
						numstr[len+1] = ';';
						numstr[len+2] = 0;
						CDC_Transmit_FS((uint8_t*)numstr, len+2);
						msDelay(10);

						itoa(stepper_pos.current_pos_z, numstr, 10);
						len=strlen(numstr);
						CDC_Transmit_FS((uint8_t*)numstr, len);
						msDelay(10);

					}
					break;

		default: 	//
					break;
	}

	switch(rxbuffer[0])
	{
		case 'i':
		case 'I':

		case 'P':
		case 'p':

		case 'v':
		case 'V':

		case 'g':
		case 'G':
					char sendbuff[RXBUFFSIZE] = {0};
					char lineNumStr[RXBUFFSIZE] = {0};

					memcpy(sendbuff, rxbuffer, rxbufend);
					sprintf(lineNumStr, "_L%llu", num_of_cmd_line);
					strlcat(sendbuff, lineNumStr, RXBUFFSIZE);
					sendbuff[RXBUFFSIZE-1]=0;//a lezáró elemet lecseréljük lezáró nullára hogy a string biztos standard legyen attól függetlenül hogy az strlcat ezt elvileg megcsinálja
					ext_brd_transmit_string(PrintCNCcmdAndLineNum_cmd, (uint8_t*)sendbuff, strlen(sendbuff));
					num_of_cmd_line++;

					/*
					rxbuffer[rxbufend]=0;//az utolsó elemet lecseréljük lezáró nullára hogy a string biztos standard legyen
					ext_brd_transmit_string(PrintCNCcmd_cmd, rxbuffer, rxbufend+1);
					rxbuffer[rxbufend]=CMD_DONE_CHAR;//visszaállítás*/
					break;
		case 't':
		case 'T':
					break;
	}

	ResetCDCrxBuffer();
	rxflagbyte = 0;
	if(CDCsend(cmddone, 1, 255) != USBD_OK)
	{
		Error_Handler();
	}
  }

  gotozero(&stepper_pos);
  STSPIN220_power_down();

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  __NOP();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

  /* TIM2 interrupt Init */
  NVIC_SetPriority(TIM2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  NVIC_EnableIRQ(TIM2_IRQn);

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  TIM_InitStruct.Prescaler = 22;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 65454;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM2, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM2);
  LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH2);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 6545;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM2, LL_TIM_CHANNEL_CH2);
  LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM2);
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**TIM2 GPIO Configuration
  PA1   ------> TIM2_CH2
  */
  GPIO_InitStruct.Pin = SERVO_PWM_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(SERVO_PWM_GPIO_Port, &GPIO_InitStruct);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);

  /* TIM4 interrupt Init */
  NVIC_SetPriority(TIM4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM4_IRQn);

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  TIM_InitStruct.Prescaler = 72;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 65535;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM4, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM4);
  LL_TIM_SetClockSource(TIM4, LL_TIM_CLOCKSOURCE_INTERNAL);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_FROZEN;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 1;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM4, LL_TIM_CHANNEL_CH1);
  LL_TIM_SetOnePulseMode(TIM4, LL_TIM_ONEPULSEMODE_SINGLE);
  LL_TIM_SetTriggerOutput(TIM4, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM4);
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */

/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(GPIOC, MODE1_1_Pin|MODE2_1_Pin|USB_REENUM_Pin);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, STCK_1_Pin|STCK_3_Pin|STCK_4_Pin|DIR_1_Pin
                          |DIR_3_Pin|DIR_4_Pin|MODE1_4_Pin|MODE2_4_Pin
                          |EN_FAULT_4_Pin|MODE2_3_Pin);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, EN_FAULT_1_Pin|MODE1_3_Pin|EN_FAULT_3_Pin|SPI2_CS_Pin
                          |STBY_RESET_ALL_Pin);

  /**/
  GPIO_InitStruct.Pin = MODE1_1_Pin|MODE2_1_Pin|USB_REENUM_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = STCK_1_Pin|STCK_3_Pin|STCK_4_Pin|DIR_1_Pin
                          |DIR_3_Pin|DIR_4_Pin|MODE1_4_Pin|MODE2_4_Pin
                          |EN_FAULT_4_Pin|MODE2_3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = EN_FAULT_1_Pin|MODE1_3_Pin|EN_FAULT_3_Pin|SPI2_CS_Pin
                          |STBY_RESET_ALL_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = EXTENSION_BRD_BSY_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(EXTENSION_BRD_BSY_GPIO_Port, &GPIO_InitStruct);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTB, LL_GPIO_AF_EXTI_LINE10);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTB, LL_GPIO_AF_EXTI_LINE11);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTB, LL_GPIO_AF_EXTI_LINE4);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTB, LL_GPIO_AF_EXTI_LINE5);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTB, LL_GPIO_AF_EXTI_LINE8);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTB, LL_GPIO_AF_EXTI_LINE9);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_10;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_11;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_4;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_5;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_8;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_9;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  LL_GPIO_SetPinPull(SW1_4_GPIO_Port, SW1_4_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinPull(SW2_4_GPIO_Port, SW2_4_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinPull(SW1_1_GPIO_Port, SW1_1_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinPull(SW2_1_GPIO_Port, SW2_1_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinPull(SW1_3_GPIO_Port, SW1_3_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinPull(SW2_3_GPIO_Port, SW2_3_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinMode(SW1_4_GPIO_Port, SW1_4_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(SW2_4_GPIO_Port, SW2_4_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(SW1_1_GPIO_Port, SW1_1_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(SW2_1_GPIO_Port, SW2_1_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(SW1_3_GPIO_Port, SW1_3_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(SW2_3_GPIO_Port, SW2_3_Pin, LL_GPIO_MODE_INPUT);

  /* EXTI interrupt init*/
  NVIC_SetPriority(EXTI4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  NVIC_EnableIRQ(EXTI4_IRQn);
  NVIC_SetPriority(EXTI9_5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  NVIC_EnableIRQ(EXTI9_5_IRQn);
  NVIC_SetPriority(EXTI15_10_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void ext_brd_transmit_string(uint8_t printCmd, uint8_t* strbuff, uint8_t len)
{
	if((LL_GPIO_IsInputPinSet(EXTENSION_BRD_BSY_GPIO_Port, EXTENSION_BRD_BSY_Pin) == 0) || (printCmd == PrintFilenameTXT_cmd))//only if extension board is not busy, not to slow down cnc movements
	{
		uint8_t* txframe = calloc(len+2,1);//+2 is for the command byte which is byte0, and len byte which is byte1
		uint8_t* dummyrx = calloc(len+2,1);
		txframe[0] = printCmd;
		txframe[1] = len+2;
		memcpy(&txframe[2], strbuff, len);

		LL_GPIO_SetOutputPin(SPI2_CS_GPIO_Port, SPI2_CS_Pin);
		while(LL_GPIO_IsInputPinSet(EXTENSION_BRD_BSY_GPIO_Port, EXTENSION_BRD_BSY_Pin) == 0)	{ __NOP();}
		HAL_SPI_TransmitReceive(&hspi2, txframe, dummyrx, len+2, 1000);
		LL_GPIO_ResetOutputPin(SPI2_CS_GPIO_Port, SPI2_CS_Pin);

		free(txframe);
		free(dummyrx);
	}
}

uint8_t CDCsend(uint8_t *str, uint32_t len, uint8_t retries)
{
	uint8_t retval = USBD_BUSY;
	while(retries>0)
	{
		retval = CDC_Transmit_FS(str, len);
		if(retval==USBD_OK)
		{
			break;
		}
		else
		{
			msDelay(10);
			retries--;
		}

	}
	return retval;
}

void ResetCDCrxBuffer(void)
{
	for(uint32_t bufindx = 0; bufindx<RXBUFFSIZE; bufindx++)
	{
		rxbuffer[bufindx] = 0;
	}
	rxbufend = 0;
}

void eval_and_execute_plot_cmd(uint8_t *cmdstr, uint32_t len, CP *currentpos)
{
	//example: PD10669,9079,35;

	int32_t indx = 0, comma = 0, comma2 = 0, semicolon = 0;
	int32_t pownum = 0;
	int32_t	numofdigits = 0;
	uint8_t tmpstr1[10] = {0};
	uint8_t tmpstr2[10] = {0};
	uint8_t tmpstr3[10] = {0};
	int32_t xval = 0, yval = 0, zval = 0;

	//eval first argument of plot cmd//X /////////////////
	for(indx=2; indx<=len; indx++)//search for the end of the argument (comma) and copy the characters of the argument to a buffer while preparing them for processing
	{
		if(cmdstr[indx] == ',')
		{
			comma = indx;
			break;
		}
		else
		{
			if(cmdstr[indx] != '-')
			{
				tmpstr1[indx-2] = cmdstr[indx]-'0';
			}
			else
			{
				tmpstr1[indx-2] = cmdstr[indx];
			}
		}
	}
	pownum = 0;
	numofdigits = (comma-2);
	indx = numofdigits-1;
	while(numofdigits > 0)
	{
		if( (numofdigits==1) && (tmpstr1[indx]=='-') )
		{
			xval *= (-1);
		}
		else
		{
			xval += tmpstr1[indx]*mypow10(pownum);
			pownum++;
		}
		indx--;
		numofdigits--;
	}

	//eval second argument of plot cmd//Y /////////////////
	for(indx=comma+1; indx<=len; indx++)
	{
		if(cmdstr[indx] == ',')
		{
			comma2 = indx;
			break;
		}
		else
		{
			if(cmdstr[indx] != '-')
			{
				tmpstr2[indx-(comma+1)] = cmdstr[indx]-'0';
			}
			else
			{
				tmpstr2[indx-(comma+1)] = cmdstr[indx];
			}
		}
	}
	numofdigits	= comma2-(comma+1);//azért +1 mert hozzá kell számolni a vesszőt is, azon is átment a ciklus
	indx = (numofdigits-1);
	pownum=0;
	while(numofdigits > 0)
	{
		if( (numofdigits==1) && (tmpstr2[indx]=='-') )
		{
			yval *= (-1);
		}
		else
		{
			yval += tmpstr2[indx]*mypow10(pownum);
			pownum++;
		}
		indx--;
		numofdigits--;
	}

	//eval third argument of plot cmd//Z /////////////////
	for(indx=comma2+1; indx<=len; indx++)
	{
		if(cmdstr[indx] == ';')
		{
			semicolon = indx;
			break;
		}
		else
		{
			if(cmdstr[indx] != '-')
			{
				tmpstr3[indx-(comma2+1)] = cmdstr[indx]-'0';
			}
			else
			{
				tmpstr3[indx-(comma2+1)] = cmdstr[indx];
			}
		}
	}
	numofdigits	= semicolon-(comma2+1);//azért +1 mert hozzá kell számolni a pontosvesszőt is, azon is átment a ciklus
	indx = (numofdigits-1);
	pownum=0;
	while(numofdigits > 0)
	{
		if( (numofdigits==1) && (tmpstr3[indx]=='-') )
		{
			zval *= (-1);
		}
		else
		{
			zval += tmpstr3[indx]*mypow10(pownum);
			pownum++;
		}
		indx--;
		numofdigits--;
	}


	switch(stepper_pos.curr_state & (AbsoluteStep_MSK|RelativeStep_MSK))
	{
		case RelativeStep_MSK:	HPGL_PR(xval, yval, zval, currentpos);
								break;

		case AbsoluteStep_MSK:	//clip to axis limits
								if(xval>XaxisLen)	{ xval=XaxisLen;}
								if(yval>YaxisLen)	{ yval=YaxisLen;}
								if(zval>ZaxisLen)	{ zval=ZaxisLen;}

								HPGL_PA(xval, yval, zval, currentpos);
								break;
	}
}

int32_t mypow10(int32_t exponent)
{
	uint32_t num = 10;

	switch(exponent)
	{
		case 0: num = 1;
				break;

		case 1: //num = 10;
				break;

		default:	exponent--;
					while(exponent)
					{
						num *= 10;
						exponent--;
					}
					break;
	}
	return num;
}

void msDelay(uint32_t val)
{
	while(val)
	{
		val--;
		usDelay(1000);
	}
}

void usDelay(uint32_t val)
{
#ifdef USE_INTERRUPT_US_DELAY
	//__enable_irq();
	LL_TIM_SetCounter(TIM4, 0);
	LL_TIM_EnableIT_CC1(TIM4);
	LL_TIM_OC_SetCompareCH1(TIM4, val);
	LL_TIM_SetAutoReload(TIM4, val);
	LL_TIM_EnableCounter(TIM4);
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH1);
	while(us_delay_flag != 1)	{ __NOP();}
	us_delay_flag=0;
	//__disable_irq();
#endif

#ifdef USE_BLOCKING_US_DELAY
	while(val)
	{//72db nop = 1us (when clok is 72MHz) //rough estimate, bacause it has to do instructions for the loop too
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
		val--;
	}
#endif
}

/*
#ifdef DEBUG

int _write(int file, char *ptr, int len)
{
  (void)file;
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
    ITM_SendChar(*ptr++);
  }
  return len;
}

#endif//DEBUG
*/




/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  gotozero(&stepper_pos);
  STSPIN220_power_down();

  __disable_irq();

  NVIC_SystemReset();

  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
