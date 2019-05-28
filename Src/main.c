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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "stm32f769i_discovery.h"
#include "stm32f769i_discovery_lcd.h"
#include "stm32f769i_discovery_ts.h"
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LAYER0_ADDRESS    (LCD_FB_START_ADDRESS)
#define TEMP_REFRESH_PERIOD   1000    /* Internal temperature refresh period */
#define MAX_CONVERTED_VALUE   4095    /* Max converted value */
#define AMBIENT_TEMP            25    /* Ambient Temperature */
#define VSENS_AT_AMBIENT_TEMP  760    /* VSENSE value (mv) at ambient temperature */
#define AVG_SLOPE               25    /* Avg_Solpe multiply by 10 */
#define VREF                  3300

bool flagTimer6_temperature=0;
bool flagTimer7_gametime=0;
bool flagTimer7_roundtimeleft=0;
uint8_t minute=0;
uint8_t second=0;
bool touchScreenFlag=0;
uint8_t jogador;
uint32_t ConvertedValue=0;
TS_StateTypeDef TS_State;
uint8_t colunaCelula;
uint8_t linhaCelula;
uint8_t timeLeft=20;
int   tabuleiroInicial[8][8]={{0,0,0,0,0,0,0,0},
							  {0,0,0,0,0,0,0,0},
							  {0,0,0,0,0,0,0,0},
							  {0,0,0,1,2,0,0,0},
							  {0,0,0,2,1,0,0,0},
							  {0,0,0,0,0,0,0,0},
							  {0,0,0,0,0,0,0,0},
							  {0,0,0,0,0,0,0,0}};


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DMA2D_HandleTypeDef hdma2d;

DSI_HandleTypeDef hdsi;

LTDC_HandleTypeDef hltdc;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_DMA2D_Init(void);
static void MX_DSIHOST_DSI_Init(void);
static void MX_FMC_Init(void);
static void MX_LTDC_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
static void LCD_Config(void);
static void displayTemperature();
static void displayGame();
void detectBoardTouch(uint8_t*);
void jogo();
void showGameTime();
void showRoundTimeLeft();
void detectMenuTouch();
void LoadInitialBoard();
void checkAvailable(uint8_t player);
void checkAdjacent(uint8_t player, uint8_t opponent,uint8_t i, uint8_t j,bool flagJogada);
void checkMoreOponentPieces(uint8_t player, uint8_t opponent,int16_t linha, int16_t coluna, int16_t incrLinha, int16_t incrColuna,bool flagJogada);
void turnPieces(uint8_t auxPlayer,uint8_t auxOpponent, int8_t linhaCelula, int8_t colunaCelula);
void drawPieces(uint8_t i, uint8_t j,uint8_t player);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM6)
	{
		flagTimer6_temperature=1;
	}

	if(htim->Instance == TIM7)
	{
		flagTimer7_gametime=1;
		flagTimer7_roundtimeleft=1;
	}


}



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
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_DMA2D_Init();
  MX_DSIHOST_DSI_Init();
  MX_FMC_Init();
  MX_LTDC_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

  //Inicializaçãoo dos LEDS e USER BUTTON
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_RED);
  BSP_PB_Init(BUTTON_WAKEUP,BUTTON_MODE_GPIO);

  //Inicialização do LCD e Touch Sensor
  LCD_Config();
  BSP_TS_Init(BSP_LCD_GetXSize(),BSP_LCD_GetYSize());
  BSP_TS_ITConfig();

  //Inicialização do ADC1
  HAL_ADC_Start_IT(&hadc1);

  //Inicialização dos Interrupts dos Timers 6 e 7
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);



  displayGame();
  LoadInitialBoard();
  jogador=1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  displayTemperature();
	  showGameTime();
	  showRoundTimeLeft();
	  detectMenuTouch();
	  //jogo();

	  detectBoardTouch(&jogador);








    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 196;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV2;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  hdma2d.LayerCfg[1].AlphaInverted = DMA2D_REGULAR_ALPHA;
  hdma2d.LayerCfg[1].RedBlueSwap = DMA2D_RB_REGULAR;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

}

/**
  * @brief DSIHOST Initialization Function
  * @param None
  * @retval None
  */
static void MX_DSIHOST_DSI_Init(void)
{

  /* USER CODE BEGIN DSIHOST_Init 0 */

  /* USER CODE END DSIHOST_Init 0 */

  DSI_PLLInitTypeDef PLLInit = {0};
  DSI_HOST_TimeoutTypeDef HostTimeouts = {0};
  DSI_PHY_TimerTypeDef PhyTimings = {0};
  DSI_LPCmdTypeDef LPCmd = {0};
  DSI_CmdCfgTypeDef CmdCfg = {0};

  /* USER CODE BEGIN DSIHOST_Init 1 */

  /* USER CODE END DSIHOST_Init 1 */
  hdsi.Instance = DSI;
  hdsi.Init.AutomaticClockLaneControl = DSI_AUTO_CLK_LANE_CTRL_DISABLE;
  hdsi.Init.TXEscapeCkdiv = 4;
  hdsi.Init.NumberOfLanes = DSI_ONE_DATA_LANE;
  PLLInit.PLLNDIV = 20;
  PLLInit.PLLIDF = DSI_PLL_IN_DIV1;
  PLLInit.PLLODF = DSI_PLL_OUT_DIV1;
  if (HAL_DSI_Init(&hdsi, &PLLInit) != HAL_OK)
  {
    Error_Handler();
  }
  HostTimeouts.TimeoutCkdiv = 1;
  HostTimeouts.HighSpeedTransmissionTimeout = 0;
  HostTimeouts.LowPowerReceptionTimeout = 0;
  HostTimeouts.HighSpeedReadTimeout = 0;
  HostTimeouts.LowPowerReadTimeout = 0;
  HostTimeouts.HighSpeedWriteTimeout = 0;
  HostTimeouts.HighSpeedWritePrespMode = DSI_HS_PM_DISABLE;
  HostTimeouts.LowPowerWriteTimeout = 0;
  HostTimeouts.BTATimeout = 0;
  if (HAL_DSI_ConfigHostTimeouts(&hdsi, &HostTimeouts) != HAL_OK)
  {
    Error_Handler();
  }
  PhyTimings.ClockLaneHS2LPTime = 28;
  PhyTimings.ClockLaneLP2HSTime = 33;
  PhyTimings.DataLaneHS2LPTime = 15;
  PhyTimings.DataLaneLP2HSTime = 25;
  PhyTimings.DataLaneMaxReadTime = 0;
  PhyTimings.StopWaitTime = 0;
  if (HAL_DSI_ConfigPhyTimer(&hdsi, &PhyTimings) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_ConfigFlowControl(&hdsi, DSI_FLOW_CONTROL_BTA) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_SetLowPowerRXFilter(&hdsi, 10000) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_ConfigErrorMonitor(&hdsi, HAL_DSI_ERROR_NONE) != HAL_OK)
  {
    Error_Handler();
  }
  LPCmd.LPGenShortWriteNoP = DSI_LP_GSW0P_DISABLE;
  LPCmd.LPGenShortWriteOneP = DSI_LP_GSW1P_DISABLE;
  LPCmd.LPGenShortWriteTwoP = DSI_LP_GSW2P_DISABLE;
  LPCmd.LPGenShortReadNoP = DSI_LP_GSR0P_DISABLE;
  LPCmd.LPGenShortReadOneP = DSI_LP_GSR1P_DISABLE;
  LPCmd.LPGenShortReadTwoP = DSI_LP_GSR2P_DISABLE;
  LPCmd.LPGenLongWrite = DSI_LP_GLW_DISABLE;
  LPCmd.LPDcsShortWriteNoP = DSI_LP_DSW0P_DISABLE;
  LPCmd.LPDcsShortWriteOneP = DSI_LP_DSW1P_DISABLE;
  LPCmd.LPDcsShortReadNoP = DSI_LP_DSR0P_DISABLE;
  LPCmd.LPDcsLongWrite = DSI_LP_DLW_DISABLE;
  LPCmd.LPMaxReadPacket = DSI_LP_MRDP_DISABLE;
  LPCmd.AcknowledgeRequest = DSI_ACKNOWLEDGE_DISABLE;
  if (HAL_DSI_ConfigCommand(&hdsi, &LPCmd) != HAL_OK)
  {
    Error_Handler();
  }
  CmdCfg.VirtualChannelID = 0;
  CmdCfg.ColorCoding = DSI_RGB888;
  CmdCfg.CommandSize = 640;
  CmdCfg.TearingEffectSource = DSI_TE_EXTERNAL;
  CmdCfg.TearingEffectPolarity = DSI_TE_RISING_EDGE;
  CmdCfg.HSPolarity = DSI_HSYNC_ACTIVE_LOW;
  CmdCfg.VSPolarity = DSI_VSYNC_ACTIVE_LOW;
  CmdCfg.DEPolarity = DSI_DATA_ENABLE_ACTIVE_HIGH;
  CmdCfg.VSyncPol = DSI_VSYNC_FALLING;
  CmdCfg.AutomaticRefresh = DSI_AR_ENABLE;
  CmdCfg.TEAcknowledgeRequest = DSI_TE_ACKNOWLEDGE_DISABLE;
  if (HAL_DSI_ConfigAdaptedCommandMode(&hdsi, &CmdCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_SetGenericVCID(&hdsi, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DSIHOST_Init 2 */

  /* USER CODE END DSIHOST_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};
  LTDC_LayerCfgTypeDef pLayerCfg1 = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 7;
  hltdc.Init.VerticalSync = 3;
  hltdc.Init.AccumulatedHBP = 14;
  hltdc.Init.AccumulatedVBP = 5;
  hltdc.Init.AccumulatedActiveW = 654;
  hltdc.Init.AccumulatedActiveH = 485;
  hltdc.Init.TotalWidth = 660;
  hltdc.Init.TotalHeigh = 487;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 0;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 0;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg.Alpha = 0;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = 0;
  pLayerCfg.ImageWidth = 0;
  pLayerCfg.ImageHeight = 0;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg1.WindowX0 = 0;
  pLayerCfg1.WindowX1 = 0;
  pLayerCfg1.WindowY0 = 0;
  pLayerCfg1.WindowY1 = 0;
  pLayerCfg1.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg1.Alpha = 0;
  pLayerCfg1.Alpha0 = 0;
  pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg1.FBStartAdress = 0;
  pLayerCfg1.ImageWidth = 0;
  pLayerCfg1.ImageHeight = 0;
  pLayerCfg1.Backcolor.Blue = 0;
  pLayerCfg1.Backcolor.Green = 0;
  pLayerCfg1.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg1, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */

  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 9999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 19999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 9999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 9999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK2;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_13;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_32;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_1;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_DISABLE;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 16;
  SdramTiming.ExitSelfRefreshDelay = 16;
  SdramTiming.SelfRefreshTime = 16;
  SdramTiming.RowCycleDelay = 16;
  SdramTiming.WriteRecoveryTime = 16;
  SdramTiming.RPDelay = 16;
  SdramTiming.RCDDelay = 16;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();

  /*Configure GPIO pin : PI13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_13)
	{
		touchScreenFlag=1;
		BSP_TS_GetState(&TS_State);
	}
}

static void LCD_Config(void)
{
	uint32_t lcd_status = LCD_OK;

	lcd_status = BSP_LCD_Init();
	while(lcd_status != LCD_OK);

	BSP_LCD_LayerDefaultInit(0,LAYER0_ADDRESS);
	BSP_LCD_Clear(LCD_COLOR_WHITE);
}

static void displayGame()
{
	char string[50];
	int i,j;

	 BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);
	 BSP_LCD_SetFont(&Font24);
	 sprintf(string, "PLAY REVERSI WITH YOUR ARM");
	 BSP_LCD_DisplayStringAt(0,LINE(0), (uint8_t *)string, CENTER_MODE);

	 sprintf(string, "MENU");
	 BSP_LCD_DisplayStringAt(205,LINE(9), (uint8_t *)string, CENTER_MODE);
	 BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
	 BSP_LCD_DrawRect(420,205,370,245);

	 BSP_LCD_DrawRect(455,250,300,50); //botão 1 do menu
	 sprintf(string, "Human vs. Human");
	 BSP_LCD_DisplayStringAt(205,LINE(11), (uint8_t *)string, CENTER_MODE);

	 BSP_LCD_DrawRect(455,310,300,50); //botão 2 do menu
	 sprintf(string, "Human vs. ARM");
	 BSP_LCD_DisplayStringAt(205,LINE(11)+60, (uint8_t *)string, CENTER_MODE);

	 BSP_LCD_DrawRect(455,370,300,50);//botão 3 do menu
	 sprintf(string, "Highscores");
	 BSP_LCD_DisplayStringAt(205,LINE(16), (uint8_t *)string, CENTER_MODE);



	 BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);
	 sprintf(string, "Game Information");
	 BSP_LCD_DisplayStringAt(200,LINE(3), (uint8_t *)string, CENTER_MODE);
	 BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
	 BSP_LCD_DrawRect(420,50,370,145);


	 BSP_LCD_SetFont(&Font16);
	 sprintf(string, "Game Mode: Human vs. ARM");
	 BSP_LCD_DisplayStringAt(200,LINE(7), (uint8_t *)string, CENTER_MODE);


	 //sprintf(string, "Game Time: 0 s");
	 //BSP_LCD_DisplayStringAt(200,LINE(8), (uint8_t *)string, CENTER_MODE);


	 sprintf(string, "Realizado por: Joao Gomes");
	 BSP_LCD_DisplayStringAt(10,LINE(29), (uint8_t *)string, LEFT_MODE);


	 //BSP_LCD_SetTextColor(LCD_COLOR_DARKRED);
	 //sprintf(string, "Player 1 round: 20s left");
	// BSP_LCD_DisplayStringAt(10,LINE(2), (uint8_t *)string, LEFT_MODE);





	 BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGRAY);
	 BSP_LCD_FillRect(10,50,400,400);



	 BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
	 for(i=0;i<8;i++)
	 {
		 for(j=0;j<8;j++)
		 {
			 BSP_LCD_DrawRect(10+50*j,50+50*i,50,50);
		 }
	 }
	 BSP_LCD_DrawRect(5,25,790,430);



}

static void displayTemperature()
{
	long int JTemp;
	char string[100];

	if(flagTimer6_temperature==1)
	{
			flagTimer6_temperature=0;
			BSP_LED_Toggle(LED_GREEN);
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	  		BSP_LCD_SetFont(&Font16);
	  		ConvertedValue=HAL_ADC_GetValue(&hadc1); //get value
	  		JTemp = ((((ConvertedValue * VREF)/MAX_CONVERTED_VALUE) - VSENS_AT_AMBIENT_TEMP) * 10 / AVG_SLOPE) + AMBIENT_TEMP;
	  		sprintf(string, "Int. Temp:%ld'C", JTemp);
	  		BSP_LCD_DisplayStringAt(0,LINE(29), (uint8_t *)string, RIGHT_MODE);
	  		BSP_LCD_ClearStringLine(30);
	}
}


void showGameTime()
{

	char string[100];

	if(flagTimer7_gametime==1)
	{
		flagTimer7_gametime=0;
		second++;
		if(second==60)
		{
			second=0;
			minute++;
		}
		BSP_LCD_SetFont(&Font16);
		BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
		sprintf(string, "Game Time: %im%2is",minute,second);
		BSP_LCD_DisplayStringAt(200,LINE(8), (uint8_t *)string, CENTER_MODE);

	}
}

void showRoundTimeLeft()
{

	char string[50];

	if(flagTimer7_roundtimeleft==1)
	{
		flagTimer7_roundtimeleft=0;
		timeLeft--;
		if(timeLeft==0)
			timeLeft=0;

			BSP_LCD_SetFont(&Font16);
			BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
			sprintf(string, "%2is left",timeLeft);
			BSP_LCD_DisplayStringAt(200,LINE(2), (uint8_t *)string, LEFT_MODE);

	}
}




void detectBoardTouch(uint8_t* jk)
{
	//Esta função detecta em que célula do tabuleiro carregámos e preenche a célula
	char string[20];
	uint16_t pos_x=0;
	uint16_t pos_y=0;
	uint8_t auxPlayer;
	uint8_t auxOpponent;

	if(*jk==1)
	{
		BSP_LCD_SetFont(&Font16);
		BSP_LCD_SetTextColor(LCD_COLOR_RED);
		sprintf(string, "Player 1 round:");
		BSP_LCD_DisplayStringAt(30,LINE(2), (uint8_t *)string, LEFT_MODE);
		BSP_LCD_FillCircle(20,37,5);
	}
	if(*jk==2)
	{
		BSP_LCD_SetFont(&Font16);
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		sprintf(string, "Player 2 round:");
		BSP_LCD_DisplayStringAt(30,LINE(2), (uint8_t *)string, LEFT_MODE);
		BSP_LCD_FillCircle(20,37,5);
	}


	if(touchScreenFlag==1)
	{
		touchScreenFlag=0;

		if(TS_State.touchX[0]>10 && TS_State.touchX[0]<410 && TS_State.touchY[0]>50 && TS_State.touchY[0]<450)
		{
			// TS_State.touchX[0]-10 distância até ao limite do lado esquerdo do tabuleiro
			// (TS_State.touchX[0]-10)/50) número de quadrados até ao lado esquerdo do tabuleiro
			// 10+(TS_State.touchX[0]-10)/50)*50) multiplicamos por 50 para dar a distância até à celula pretendida
			colunaCelula = (TS_State.touchX[0]-10)/50;
			linhaCelula  = (TS_State.touchY[0]-50)/50;

			if(tabuleiroInicial[linhaCelula][colunaCelula]==-(*jk))
			{
				tabuleiroInicial[linhaCelula][colunaCelula]= *jk; //coloca peça na variavel tabuleiro

				auxPlayer=(*jk);
				if(auxPlayer==1)
					auxOpponent=2;

				else if(auxPlayer==2)
					auxOpponent=1;

				pos_x=10+(colunaCelula)*50; //posição no LCD
				pos_y=50+(linhaCelula)*50;

				turnPieces(auxPlayer,auxOpponent,linhaCelula,colunaCelula);



				if(*jk==1)
				{
					BSP_LCD_SetTextColor(LCD_COLOR_RED);
					BSP_LCD_FillCircle(pos_x+25,pos_y+25,20);
					jogador=2;
					checkAvailable(jogador);
					return;
				}

				if(*jk==2)
				{
					BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
					BSP_LCD_FillCircle(pos_x+25,pos_y+25,20);
					jogador=1;
					checkAvailable(jogador);
					return;
				}
			}
		}
	}
}


void detectMenuTouch()
{
	// BSP_LCD_DrawRect(420,205,370,245);
	 if(touchScreenFlag==1)
	 {
		 touchScreenFlag=0;
		 if(TS_State.touchX[0]>455 && TS_State.touchX[0]<755 && TS_State.touchY[0]>250 && TS_State.touchY[0]<300)
		 {
			 BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
			 BSP_LCD_FillRect(455,250,300,50);
		 }

		 if(TS_State.touchX[0]>455 && TS_State.touchX[0]<755 && TS_State.touchY[0]>310 && TS_State.touchY[0]<360)
		 {
			 BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
			 BSP_LCD_FillRect(455,310,300,50);
		 }

		 if(TS_State.touchX[0]>455 && TS_State.touchX[0]<755 && TS_State.touchY[0]>370 && TS_State.touchY[0]<420)
		 {
			 BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
			 BSP_LCD_FillRect(455,370,300,50);
		 }

	 }
}


void LoadInitialBoard()
{
	uint8_t i,j;
	uint16_t linha=0;
	uint16_t coluna=0;

	for(i=0;i<8;i++)
	{
		for(j=0;j<8;j++)
		{

			linha  = 50+50*i;
			coluna = 10+50*j;
			if(tabuleiroInicial[i][j]==1)
			{
				BSP_LCD_SetTextColor(LCD_COLOR_RED);
				BSP_LCD_FillCircle(coluna+25,linha+25,20);
			}

			if(tabuleiroInicial[i][j]==2)
			{
				BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
				BSP_LCD_FillCircle(coluna+25,linha+25,20);
			}
		}
	}
	checkAvailable(1);
	checkAvailable(2);
}


void checkAvailable(uint8_t player)
{
	uint8_t i,j;
	uint8_t opponent;

	if(player==1)
		opponent=2;
	else if(player==2)
		opponent=1;



	for(i=0;i<8;i++)
	{
			for(j=0;j<8;j++)
			{
				if(tabuleiroInicial[i][j]==player)
				{
					checkAdjacent(player,opponent,i,j,0);
				}
			}
	}


}

void checkAdjacent(uint8_t player, uint8_t opponent,uint8_t i, uint8_t j,bool flagJogada)
{
	if(tabuleiroInicial[i+1][j]==opponent)
	{
		checkMoreOponentPieces(player,opponent,i+2,j,1,0,flagJogada);
	}
	if(tabuleiroInicial[i-1][j]==opponent)
	{
		checkMoreOponentPieces(player,opponent,i-2,j,-1,0,flagJogada);
	}
	if(tabuleiroInicial[i][j+1]==opponent)
	{
		checkMoreOponentPieces(player,opponent,i,j+2,0,1,flagJogada);
	}
	if(tabuleiroInicial[i][j-1]==opponent)
	{
		checkMoreOponentPieces(player,opponent,i,j-2,0,-1,flagJogada);
	}
	if(tabuleiroInicial[i+1][j+1]==opponent)
	{
		checkMoreOponentPieces(player,opponent,i+2,j+2,1,1,flagJogada);
	}
	if(tabuleiroInicial[i-1][j-1]==opponent)
	{
		checkMoreOponentPieces(player,opponent,i-2,j-2,-1,-1,flagJogada);
	}
	if(tabuleiroInicial[i-1][j+1]==opponent)
	{
		checkMoreOponentPieces(player,opponent,i-2,j+2,-1,1,flagJogada);
	}
	if(tabuleiroInicial[i+1][j-1]==opponent)
	{
		checkMoreOponentPieces(player,opponent,i+2,j-2,1,-1,flagJogada);
	}
}

void checkMoreOponentPieces(uint8_t player, uint8_t opponent,int16_t linha, int16_t coluna, int16_t incrLinha, int16_t incrColuna,bool flagJogada)
{
	int8_t i,j;


	i=linha, j=coluna;


	while(i<8 && i>=0 && j<8 && j>=0 && tabuleiroInicial[i][j]==opponent)
	{
		i=i+incrLinha;
		j=j+incrColuna;
	}

	if(tabuleiroInicial[i][j]==0 && flagJogada==0)
	{
		tabuleiroInicial[i][j]=-player;
	}



}


void drawPieces(uint8_t i, uint8_t j,uint8_t player)
{
	uint16_t pos_x=0;
	uint16_t pos_y=0;

	pos_x=10+(j)*50; //posição no LCD
	pos_y=50+(i)*50;
	if(player==1)
	{
		BSP_LCD_SetTextColor(LCD_COLOR_RED);
		BSP_LCD_FillCircle(pos_x+25,pos_y+25,20);
	}
	else if(player==2)
	{
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_FillCircle(pos_x+25,pos_y+25,20);
	}
}


void turnPieces(uint8_t auxPlayer,uint8_t auxOpponent, int8_t linhaCelula, int8_t colunaCelula)
{
	int8_t i;
	int8_t j;
	int8_t auxLinha=linhaCelula;
	int8_t auxColuna=colunaCelula;

	for(i=-1;i<=1;i++)
	{
	        for(j=-1;j<=1;j++)
	        {
	        	if(tabuleiroInicial[linhaCelula+i][colunaCelula+j]!=auxPlayer && tabuleiroInicial[linhaCelula+i][colunaCelula+j]==auxOpponent)
	        	{
	        		linhaCelula=linhaCelula + i;
	        		colunaCelula=colunaCelula + j;

	        		if(tabuleiroInicial[linhaCelula+i][colunaCelula+j]==auxPlayer && linhaCelula+i!=auxLinha && colunaCelula+j !=auxColuna)
	        		{
	        			  drawPieces(linhaCelula-i,colunaCelula-j,auxPlayer);
	        			  tabuleiroInicial[linhaCelula-i][colunaCelula-j]=auxPlayer;
	        			  linhaCelula=linhaCelula - i;
	        			  colunaCelula=colunaCelula - j;
	        		}



	        	}
	        /*	if(tabuleiroInicial[linhaCelula+i][colunaCelula+j]==auxPlayer)
	        	{
	        		drawPieces(linhaCelula+i,colunaCelula+j,auxPlayer);
	        		tabuleiroInicial[linhaCelula-i][colunaCelula-j]=auxPlayer;
	        		linhaCelula=linhaCelula - i;
	        		colunaCelula=colunaCelula - j;
	        	}*/

	        }

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
