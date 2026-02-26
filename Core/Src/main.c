/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
  ******************************************************************************
  *
  *
  * -serial port settings: 9600 baud, 1 stop bit, no parity, no flow control
  *
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

cmd_struct* cmdBuff_pp[CMDBUFFSIZE] = {NULL};
uint8_t	cmdBuffIndx = 0;
uint8_t	cmdBuffEndIndx = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
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

volatile uint32_t num_of_cmd_line = 1;//cmd counter

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  cmd_struct cs0 = {0, NULL};
  cmdBuff_pp[0] = &cs0;
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
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  A4988_init(&stepper_pos, stepsize_1_16);

  __enable_irq();

  LL_mDelay(500);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  stepper_pos.toolspeed = DEFAULTTOOLSPEED;
  stepper_pos.curr_state &= ~(RapidMove | RelativeStep_MSK);
  stepper_pos.curr_state |= (FeedMove | AbsoluteStep_MSK);
  num_of_cmd_line = 1;
  get_limit_sw_state();
  gotohome(&stepper_pos);

  while(host_com_port_open == 0)//wait for the CDC_Control_FS() function to be called with "CDC_SET_CONTROL_LINE_STATE" command
  {
	  LL_mDelay(250);
  }
  msDelay(20);
  uint8_t str[] = {"Connected to PC"};
  ext_ctr_transmit_string(PrintInfo_cmd, str, sizeof(str));

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

	LL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

	handleCmd(cmdBuff_pp, &cmdBuffIndx);
	execute_cmd(cmdBuff_pp[cmdBuffIndx], &stepper_pos, &num_of_cmd_line);
	screen_update(cmdBuff_pp[cmdBuffIndx]);

	if(cmdBuff_pp[cmdBuffIndx]->params != NULL)
	{
		free(cmdBuff_pp[cmdBuffIndx]->params);
		cmdBuff_pp[cmdBuffIndx]->params = NULL;//prevents use after free
	}

	memset(rxbuffer, 0, RXBUFFSIZE);
	rxbufend = 0;
	rxflagbyte = 0;
	if(CDCsend(cmddone, 1, 255) != USBD_OK)
	{
		Error_Handler();
	}
  }

  //bab
  gotohome(&stepper_pos);
  A4988_power_down();

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
  NVIC_SetPriority(TIM4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
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
  LL_GPIO_ResetOutputPin(GPIOA, DIR_X_Pin|STEP_X_Pin|_SLEEP_Pin|_RST_Pin
                          |MS3_Pin|MS2_Pin|MS1_Pin|LED1_Pin
                          |LED2_Pin|USB_REENUM_Pin|LED3_Pin);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, _EN_Pin|DIR_Y_Pin|STEP_Y_Pin|DIR_Z_Pin
                          |STEP_Z_Pin|SPI2_CS_Pin|LED4_Pin);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTC, LL_GPIO_AF_EXTI_LINE13);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTB, LL_GPIO_AF_EXTI_LINE4);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTB, LL_GPIO_AF_EXTI_LINE5);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTB, LL_GPIO_AF_EXTI_LINE6);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTB, LL_GPIO_AF_EXTI_LINE7);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTB, LL_GPIO_AF_EXTI_LINE8);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTB, LL_GPIO_AF_EXTI_LINE9);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_13;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
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
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_6;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_7;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
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
  LL_GPIO_SetPinPull(SAFETY_SW_GPIO_Port, SAFETY_SW_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinPull(SW_ZH_GPIO_Port, SW_ZH_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinPull(SW_ZE_GPIO_Port, SW_ZE_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinPull(SW_YH_GPIO_Port, SW_YH_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinPull(SW_YE_GPIO_Port, SW_YE_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinPull(SW_XH_GPIO_Port, SW_XH_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinPull(SW_XE_GPIO_Port, SW_XE_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinMode(SAFETY_SW_GPIO_Port, SAFETY_SW_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(SW_ZH_GPIO_Port, SW_ZH_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(SW_ZE_GPIO_Port, SW_ZE_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(SW_YH_GPIO_Port, SW_YH_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(SW_YE_GPIO_Port, SW_YE_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(SW_XH_GPIO_Port, SW_XH_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(SW_XE_GPIO_Port, SW_XE_Pin, LL_GPIO_MODE_INPUT);

  /**/
  GPIO_InitStruct.Pin = SPI2_EXT_CTR_BSY_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(SPI2_EXT_CTR_BSY_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = DIR_X_Pin|STEP_X_Pin|_SLEEP_Pin|_RST_Pin
                          |MS3_Pin|MS2_Pin|MS1_Pin|LED1_Pin
                          |LED2_Pin|USB_REENUM_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = _EN_Pin|DIR_Y_Pin|STEP_Y_Pin|DIR_Z_Pin
                          |STEP_Z_Pin|SPI2_CS_Pin|LED4_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  NVIC_SetPriority(EXTI4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),2, 0));
  NVIC_EnableIRQ(EXTI4_IRQn);
  NVIC_SetPriority(EXTI9_5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),2, 0));
  NVIC_EnableIRQ(EXTI9_5_IRQn);
  NVIC_SetPriority(EXTI15_10_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void handleCmd(cmd_struct** cmdBuff_f, uint8_t* cmdBuffIndx_f)
{
	switch(rxbuffer[0])
	{
		//commands with 0 argument////////////////////////////////////////////////
		case 'i':
		case 'I':	if( (rxbuffer[1]=='n') || (rxbuffer[1]=='N') )
					{
						cmdBuff_f[*cmdBuffIndx_f]->cmd = CMD_INITIALIZE;
						cmdBuff_f[*cmdBuffIndx_f]->params = NULL;
					}
					else
					{
						cmdBuff_f[*cmdBuffIndx_f]->cmd = CMD_ERROR;
						cmdBuff_f[*cmdBuffIndx_f]->params = NULL;
					}
					break;

		case 'h':
		case 'H':	if( (rxbuffer[1]=='p') || (rxbuffer[1]=='P') )
					{
						cmdBuff_f[*cmdBuffIndx_f]->cmd = CMD_HOMEPOS;
						cmdBuff_f[*cmdBuffIndx_f]->params = NULL;
					}
					else
					{
						cmdBuff_f[*cmdBuffIndx_f]->cmd = CMD_ERROR;
						cmdBuff_f[*cmdBuffIndx_f]->params = NULL;
					}
					break;

		case 'e':
		case 'E':	if( (rxbuffer[1]=='p') || (rxbuffer[1]=='P') )
					{
						cmdBuff_f[*cmdBuffIndx_f]->cmd = CMD_ENDPOS;
						cmdBuff_f[*cmdBuffIndx_f]->params = NULL;
					}
					else
					{
						cmdBuff_f[*cmdBuffIndx_f]->cmd = CMD_ERROR;
						cmdBuff_f[*cmdBuffIndx_f]->params = NULL;
					}
					break;

		case 'g':
		case 'G':	if( (rxbuffer[1]=='p') || (rxbuffer[1]=='P') )
					{
						cmdBuff_f[*cmdBuffIndx_f]->cmd = CMD_GETCURRENTPOS;
						cmdBuff_f[*cmdBuffIndx_f]->params = NULL;
					}
					else if( (rxbuffer[1]=='o') || (rxbuffer[1]=='O') )
					{
						cmdBuff_f[*cmdBuffIndx_f]->cmd = CMD_GETORIGIN;
						cmdBuff_f[*cmdBuffIndx_f]->params = NULL;
					}
					else
					{
						cmdBuff_f[*cmdBuffIndx_f]->cmd = CMD_ERROR;
						cmdBuff_f[*cmdBuffIndx_f]->params = NULL;
					}
					break;

		case 'o':
		case 'O':	switch(rxbuffer[1])
					{
						case 'x':
						case 'X':	cmdBuff_f[*cmdBuffIndx_f]->cmd = CMD_SETAXLEORIGIN_X;
									cmdBuff_f[*cmdBuffIndx_f]->params = NULL;
									break;
						case 'y':
						case 'Y':	cmdBuff_f[*cmdBuffIndx_f]->cmd = CMD_SETAXLEORIGIN_Y;
									cmdBuff_f[*cmdBuffIndx_f]->params = NULL;
									break;
						case 'z':
						case 'Z':	cmdBuff_f[*cmdBuffIndx_f]->cmd = CMD_SETAXLEORIGIN_Z;
									cmdBuff_f[*cmdBuffIndx_f]->params = NULL;
									break;

						default:	cmdBuff_f[*cmdBuffIndx_f]->cmd = CMD_ERROR;
									cmdBuff_f[*cmdBuffIndx_f]->params = NULL;
									break;
					}
					break;
	    //commands with 1 argument////////////////////////////////////////////////
		case 'v':
		case 'V':	if( (rxbuffer[1]=='s') || (rxbuffer[1]=='S') )
					{
						cmdBuff_f[*cmdBuffIndx_f]->cmd = CMD_TOOLSPEED;
						cmdBuff_f[*cmdBuffIndx_f]->params = (int32_t*)calloc(1, sizeof(int32_t)); // eval_arg_in_cmd() fn does not replace the value, it adds the new val to the variable, so it has to be 0
						eval_arg_in_cmd(rxbuffer, 2, ';', rxbufend, cmdBuff_f[*cmdBuffIndx_f]->params);
					}
					else
					{
						cmdBuff_f[*cmdBuffIndx_f]->cmd = CMD_ERROR;
						cmdBuff_f[*cmdBuffIndx_f]->params = NULL;
					}
					break;

		//commands with block argument////////////////////////////////////////////////
		case 'P':
		case 'p':	switch(rxbuffer[1])
					{
						case 'a':
						case 'A':	cmdBuff_f[*cmdBuffIndx_f]->cmd = CMD_PLOT_A;
									cmdBuff_f[*cmdBuffIndx_f]->params = NULL;
									break;

						case 'r':
						case 'R':	cmdBuff_f[*cmdBuffIndx_f]->cmd = CMD_PLOT_R;
									cmdBuff_f[*cmdBuffIndx_f]->params = NULL;
									break;

						case 'u':
						case 'U':	cmdBuff_f[*cmdBuffIndx_f]->cmd = CMD_PLOT_U;
									cmdBuff_f[*cmdBuffIndx_f]->params = NULL;
									break;

						case 'd':
						case 'D':	cmdBuff_f[*cmdBuffIndx_f]->cmd = CMD_PLOT_D;
									cmdBuff_f[*cmdBuffIndx_f]->params = NULL;
									break;

						default:	cmdBuff_f[*cmdBuffIndx_f]->cmd = CMD_ERROR;
									cmdBuff_f[*cmdBuffIndx_f]->params = NULL;
									break;
					}
					if((rxbufend>6) && (cmdBuff_f[*cmdBuffIndx_f]->cmd != CMD_ERROR))//ha a command tartalmaz valid paramétereket is akkor legalább 8 lesz a hossza pontosvesszővel együtt, pl: PD0,0,0;
					{
						if(eval_plot_cmd(rxbuffer, rxbufend, &stepper_pos, cmdBuff_f[*cmdBuffIndx_f]) != 0)
						{
							cmdBuff_f[*cmdBuffIndx_f]->cmd = CMD_ERROR;
							cmdBuff_f[*cmdBuffIndx_f]->params = NULL;
						}
					}
					break;

		case 't':
		case 'T':	if( (rxbuffer[1]=='x') || (rxbuffer[1]=='X') )
					{
						rxbuffer[rxbufend]=0;//az utolsó elemet lecseréljük lezáró nullára hogy a string biztos standard legyen
						cmdBuff_f[*cmdBuffIndx_f]->cmd = CMD_PRINTFILENAME;
						cmdBuff_f[*cmdBuffIndx_f]->params = (uint8_t*)calloc(1, rxbufend-1);
						memcpy((int8_t*)cmdBuff_f[*cmdBuffIndx_f]->params, &rxbuffer[2], rxbufend-1);
						rxbuffer[rxbufend]=CMD_DONE_CHAR;//visszaállítás
					}
					else
					{
						cmdBuff_f[*cmdBuffIndx_f]->cmd = CMD_ERROR;
						cmdBuff_f[*cmdBuffIndx_f]->params = NULL;
					}

					break;

		default: 	cmdBuff_f[*cmdBuffIndx_f]->cmd = CMD_ERROR;
					cmdBuff_f[*cmdBuffIndx_f]->params = NULL;
					break;
	}
}

void execute_cmd(cmd_struct *cmdstr, CP* currentpos, volatile uint32_t *num_of_cmd_line_p)
{
	switch(cmdstr->cmd)
	{
		default:
		case CMD_ERROR: 			break;

		case CMD_INITIALIZE: 		currentpos->toolspeed = DEFAULTTOOLSPEED;
									currentpos->curr_state &= ~(RapidMove | RelativeStep_MSK);
									currentpos->curr_state |= (FeedMove | AbsoluteStep_MSK);
									currentpos->origin_offset_x = 0;
									currentpos->origin_offset_y = 0;
									currentpos->origin_offset_z = 0;
									(*num_of_cmd_line_p) = 1;
									gotohome(currentpos);
									msDelay(100);
							 	 	break;

		case CMD_HOMEPOS: 			currentpos->curr_state &= ~RelativeStep_MSK;
									currentpos->curr_state &= ~FeedMove;
									currentpos->curr_state |= AbsoluteStep_MSK;
									currentpos->curr_state |= RapidMove;
									HPGL_PA(currentpos->current_pos_x, currentpos->current_pos_y, ZaxisLen, currentpos);//lift up tool
									HPGL_PA(0, 0, ZaxisLen, currentpos);
						  	  	  	break;

		case CMD_ENDPOS: 			stepper_pos.curr_state &= ~RelativeStep_MSK;
									currentpos->curr_state &= ~FeedMove;
									currentpos->curr_state |= AbsoluteStep_MSK;
									currentpos->curr_state |= RapidMove;
									HPGL_PA(currentpos->current_pos_x, currentpos->current_pos_y, ZaxisLen, currentpos);//lift up tool
									HPGL_PA(XaxisLen, YaxisLen, ZaxisLen, currentpos);
						 	 	 	break;

		case CMD_GETCURRENTPOS: 	{
										char numstr[10] = {0};
										uint8_t len;
										itoa(currentpos->current_pos_x, numstr, 10);
										len=strlen(numstr);
										numstr[len+1] = ';';
										numstr[len+2] = 0;
										CDC_Transmit_FS((uint8_t*)numstr, len+2);
										msDelay(10);
										itoa(currentpos->current_pos_y, numstr, 10);
										len=strlen(numstr);
										numstr[len+1] = ';';
										numstr[len+2] = 0;
										CDC_Transmit_FS((uint8_t*)numstr, len+2);
										msDelay(10);
										itoa(currentpos->current_pos_z, numstr, 10);
										len=strlen(numstr);
										CDC_Transmit_FS((uint8_t*)numstr, len);
										msDelay(10);
									}
									break;

		case CMD_GETORIGIN: 		{
										char numstr[10] = {0};
										uint8_t len;
										itoa(currentpos->origin_offset_x, numstr, 10);
										len=strlen(numstr);
										numstr[len+1] = ';';
										numstr[len+2] = 0;
										CDC_Transmit_FS((uint8_t*)numstr, len+2);
										msDelay(10);
										itoa(currentpos->origin_offset_y, numstr, 10);
										len=strlen(numstr);
										numstr[len+1] = ';';
										numstr[len+2] = 0;
										CDC_Transmit_FS((uint8_t*)numstr, len+2);
										msDelay(10);
										itoa(currentpos->origin_offset_z, numstr, 10);
										len=strlen(numstr);
										CDC_Transmit_FS((uint8_t*)numstr, len);
										msDelay(10);
									}
									break;

		case CMD_SETAXLEORIGIN_X: 	currentpos->origin_offset_x = currentpos->current_pos_x;
									break;

		case CMD_SETAXLEORIGIN_Y: 	currentpos->origin_offset_y = currentpos->current_pos_y;
									break;

		case CMD_SETAXLEORIGIN_Z: 	currentpos->origin_offset_z = currentpos->current_pos_z;
									break;
		//commands with 1 argument
		case CMD_TOOLSPEED: 		currentpos->toolspeed = *((int32_t*)cmdstr->params);
						    		break;
		//commads with block argument
		case CMD_PLOT_A: 			currentpos->curr_state &= ~RelativeStep_MSK;
									currentpos->curr_state |= AbsoluteStep_MSK;
									if(cmdstr->params != NULL)	{ execute_plot_cmd(cmdstr ,currentpos);}
						 	 	 	break;

		case CMD_PLOT_R: 			currentpos->curr_state &= ~AbsoluteStep_MSK;
									currentpos->curr_state |= RelativeStep_MSK;
									if(cmdstr->params != NULL)	{ execute_plot_cmd(cmdstr ,currentpos);}
						 	 	 	break;

		case CMD_PLOT_U: 			currentpos->curr_state &= ~FeedMove;
									currentpos->curr_state |= RapidMove;
									if(cmdstr->params != NULL)	{ execute_plot_cmd(cmdstr ,currentpos);}
						 	 	 	break;

		case CMD_PLOT_D: 			currentpos->curr_state &= ~RapidMove;
									currentpos->curr_state |= FeedMove;
									if(cmdstr->params != NULL)	{ execute_plot_cmd(cmdstr ,currentpos);}
						 	 	 	break;

		case CMD_PRINTFILENAME: 	{
										char txs[EXTCTRLPRINTMAXLEN] = {0};
										if(cmdstr->params != NULL)
										{
											(void)snprintf( txs, EXTCTRLPRINTMAXLEN-2, "%s", (char*)cmdstr->params);
										}
										msDelay(100);//this way it will always print the text, because under this time it is sure that the ext ccontroller finishes, and won't be busy, I don't mintd the delay, this cmd is useless to me during cutting
										txs[EXTCTRLPRINTMAXLEN-1] = 0;//az utolsó elemet lecseréljük lezáró nullára hogy a string biztos standard legyen
										ext_ctr_transmit_string(PrintFilenameTXT_cmd, (uint8_t*)txs, strlen(txs));
									}
		 	 	 	 	 	 		break;
	}
}

void screen_update(cmd_struct *cmdstr)
{
	char txs[EXTCTRLPRINTMAXLEN] = {0};

	switch(cmdstr->cmd)
	{
		default:
		case CMD_ERROR: 			break;

		case CMD_INITIALIZE: 		txs[0] = 'I';
							 	 	txs[1] = 'N';
							 	 	break;
		case CMD_HOMEPOS: 			txs[0] = 'H';
						  	  	  	txs[1] = 'P';
						  	  	  	break;
		case CMD_ENDPOS: 			txs[0] = 'E';
						 	 	 	txs[1] = 'P';
						 	 	 	break;
		case CMD_GETCURRENTPOS: 	txs[0] = 'G';
									txs[1] = 'P';
									break;
		case CMD_GETORIGIN: 		txs[0] = 'G';
									txs[1] = 'O';
									break;
		case CMD_SETAXLEORIGIN_X: 	txs[0] = 'O';
							      	txs[1] = 'X';
							      	break;
		case CMD_SETAXLEORIGIN_Y: 	txs[0] = 'O';
							      	txs[1] = 'Y';
							      	break;
		case CMD_SETAXLEORIGIN_Z: 	txs[0] = 'O';
							      	txs[1] = 'Z';
							      	break;
		//commands with 1 argument
		case CMD_TOOLSPEED: 		txs[0] = 'V';
						    		txs[1] = 'S';
						    		break;
		//commads with block argument
		case CMD_PLOT_A: 			txs[0] = 'P';
						 	 	 	txs[1] = 'A';
						 	 	 	if(cmdstr->params != NULL)//only if there are arguments, it is totally fine if the command is without arguments!
						 	 	 	{
						 	 	 		(void)snprintf( &txs[2], EXTCTRLPRINTMAXLEN-4, "%li,%li,%li", ((int32_t*)cmdstr->params)[0], ((int32_t*)cmdstr->params)[1], ((int32_t*)cmdstr->params)[2]);
						 	 	 	}
						 	 	 	break;
		case CMD_PLOT_R: 			txs[0] = 'P';
						 	 	 	txs[1] = 'R';
						 	 	 	if(cmdstr->params != NULL)//only if there are arguments, it is totally fine if the command is without arguments!
									{
						 	 	 		(void)snprintf( &txs[2], EXTCTRLPRINTMAXLEN-4, "%li,%li,%li", ((int32_t*)cmdstr->params)[0], ((int32_t*)cmdstr->params)[1], ((int32_t*)cmdstr->params)[2]);
									}
						 	 	 	break;
		case CMD_PLOT_U: 			txs[0] = 'P';
						 	 	 	txs[1] = 'U';
						 	 	 	if(cmdstr->params != NULL)//only if there are arguments, it is totally fine if the command is without arguments!
									{
						 	 	 		(void)snprintf( &txs[2], EXTCTRLPRINTMAXLEN-4, "%li,%li,%li", ((int32_t*)cmdstr->params)[0], ((int32_t*)cmdstr->params)[1], ((int32_t*)cmdstr->params)[2]);
									}
						 	 	 	break;
		case CMD_PLOT_D: 			txs[0] = 'P';
						 	 	 	txs[1] = 'D';
						 	 	 	if(cmdstr->params != NULL)//only if there are arguments, it is totally fine if the command is without arguments!
									{
						 	 	 		(void)snprintf( &txs[2], EXTCTRLPRINTMAXLEN-4, "%li,%li,%li", ((int32_t*)cmdstr->params)[0], ((int32_t*)cmdstr->params)[1], ((int32_t*)cmdstr->params)[2]);
									}
						 	 	 	break;

		case CMD_PRINTFILENAME: 	break; //handled completely in execute_cmd()
	}
	if(txs[1] != 0)
	{
		char lineNumStr[10] = {0};
		memcpy(txs, rxbuffer, rxbufend);
		sprintf(lineNumStr, "_L%lu", num_of_cmd_line);
		strlcat(txs, lineNumStr, EXTCTRLPRINTMAXLEN);
		txs[EXTCTRLPRINTMAXLEN-1]=0;//a lezáró elemet lecseréljük lezáró nullára hogy a string biztos standard legyen attól függetlenül hogy az strlcat ezt elvileg megcsinálja
		ext_ctr_transmit_string(PrintCNCcmdAndLineNum_cmd, (uint8_t*)txs, strlen(txs));
		num_of_cmd_line++;
	}
}

int8_t eval_plot_cmd(uint8_t *cmdstr, uint32_t len, CP *currentpos, cmd_struct* cmds)
{
	//example: PD10669,9079,35;

	int32_t xval = 0, yval = 0, zval = 0;
	uint8_t delimiterpos = 0;

	delimiterpos = eval_arg_in_cmd(cmdstr, 2, ',', len, &xval);
	if((delimiterpos == 0) && (xval == INT32_MIN))	{return -1;}//check if parameter is valid and converted to a value
	delimiterpos = eval_arg_in_cmd(cmdstr, delimiterpos+1, ',', len, &yval);
	if((delimiterpos == 0) && (yval == INT32_MIN))	{return -1;}
	delimiterpos = eval_arg_in_cmd(cmdstr, delimiterpos+1, ';', len, &zval);
	if((delimiterpos == 0) && (zval == INT32_MIN))	{return -1;}

	cmds->params = (int32_t*)calloc(3, sizeof(int32_t));
	((int32_t*)cmds->params)[0] = xval;
	((int32_t*)cmds->params)[1] = yval;
	((int32_t*)cmds->params)[2] = zval;

	return 0;
}

void execute_plot_cmd(cmd_struct *cmdstr, CP *currentpos)
{
	switch(currentpos->curr_state & (AbsoluteStep_MSK|RelativeStep_MSK))
	{
		case RelativeStep_MSK:	//not clipping to axis limits intentionally
								HPGL_PR(((int32_t*)cmdstr->params)[0], ((int32_t*)cmdstr->params)[1], ((int32_t*)cmdstr->params)[2], currentpos);
								break;

		case AbsoluteStep_MSK:
								((int32_t*)cmdstr->params)[0] += currentpos->origin_offset_x;
								((int32_t*)cmdstr->params)[1] += currentpos->origin_offset_y;
								((int32_t*)cmdstr->params)[2] += currentpos->origin_offset_z;

								//clip to axis limits
								if(((int32_t*)cmdstr->params)[0]>XaxisLen)	{ ((int32_t*)cmdstr->params)[0]=XaxisLen;}
								if(((int32_t*)cmdstr->params)[1]>YaxisLen)	{ ((int32_t*)cmdstr->params)[1]=YaxisLen;}
								if(((int32_t*)cmdstr->params)[2]>ZaxisLen)	{ ((int32_t*)cmdstr->params)[2]=ZaxisLen;}

								HPGL_PA(((int32_t*)cmdstr->params)[0], ((int32_t*)cmdstr->params)[1], ((int32_t*)cmdstr->params)[2], currentpos);
								break;
		default: break;
	}
}

uint8_t eval_arg_in_cmd(uint8_t* cmdstr, uint8_t startindx, char delimiter, uint8_t maxlen, int32_t* numarg)
{
	uint8_t indx = 0;
	uint8_t delimiter_pos = 0;
	uint8_t tmpstr[10] = {0};
	uint8_t pownum = 0;
	uint8_t	numofdigits = 0;

	for( indx=startindx; indx<=maxlen; indx++)//search for the end of the argument (delimiter) and copy the characters of the argument to a buffer while preparing them for processing
	{
		if(cmdstr[indx] == delimiter)
		{
			delimiter_pos = indx;
			break;
		}
		else
		{
			if(cmdstr[indx] != '-')
			{
				tmpstr[indx-startindx] = cmdstr[indx]-'0';
			}
			else
			{
				tmpstr[indx-startindx] = cmdstr[indx];
			}
		}
	}
	pownum = 0;
	numofdigits = delimiter_pos-startindx;
	indx = numofdigits-1;
	while(numofdigits > 0)
	{
		if( (numofdigits==1) && (tmpstr[indx]=='-') )
		{
			(*numarg) *= (-1);
		}
		else
		{
			if((tmpstr[indx] >= 0) && (tmpstr[indx] <= 9))//make sure only numbers are converted, else the command is wrong
			{
				(*numarg) += tmpstr[indx]*mypow10(pownum);
				pownum++;
			}
			else
			{
				(*numarg) = INT32_MIN;	//indicate error
				return 0;				//
			}
		}
		indx--;
		numofdigits--;
	}

	return delimiter_pos;
}

void ext_ctr_transmit_string(uint8_t printCmd, uint8_t* strbuff, uint8_t len)
{
	if((LL_GPIO_IsInputPinSet(SPI2_EXT_CTR_BSY_GPIO_Port, SPI2_EXT_CTR_BSY_Pin) == 0) || (printCmd == PrintFilenameTXT_cmd))//only if extension board is not busy, not to slow down cnc movements
	{
		/* TODO is this better?
		if(len > (EXTCTRLPRINTMAXLEN-2)) { len = (EXTCTRLPRINTMAXLEN-2);}

		uint8_t txframe[EXTCTRLPRINTMAXLEN] = {0};
		uint8_t dummyrx[EXTCTRLPRINTMAXLEN] = {0};

		txframe[0] = printCmd;
		txframe[1] = len+2;
		memcpy(&txframe[2], strbuff, len);

		LL_GPIO_SetOutputPin(SPI2_CS_GPIO_Port, SPI2_CS_Pin);
		while(LL_GPIO_IsInputPinSet(SPI2_EXT_CTR_BSY_GPIO_Port, SPI2_EXT_CTR_BSY_Pin) == 0)	{ __NOP();}
		HAL_SPI_TransmitReceive(&hspi2, txframe, dummyrx, len+2, 1000);
		LL_GPIO_ResetOutputPin(SPI2_CS_GPIO_Port, SPI2_CS_Pin);*/

		uint8_t* txframe = calloc(len+2,1);//+2 is for the command byte which is byte0, and len byte which is byte1
		uint8_t* dummyrx = calloc(len+2,1);
		txframe[0] = printCmd;
		txframe[1] = len+2;
		memcpy(&txframe[2], strbuff, len);

		LL_GPIO_SetOutputPin(SPI2_CS_GPIO_Port, SPI2_CS_Pin);
		while(LL_GPIO_IsInputPinSet(SPI2_EXT_CTR_BSY_GPIO_Port, SPI2_EXT_CTR_BSY_Pin) == 0)	{ __NOP();}
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

  gotohome(&stepper_pos);
  A4988_power_down();

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
