/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "fdcan.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PVESC_UART.h"
#include "checksum.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Drive Feedback
typedef struct {
  float vel;              //!< velocity (rad/s)
  float pos;              //!< position (rad)
} DriveFeedback;

typedef struct {
  uint8_t TxCommand;
  uint8_t errorFlag;
  uint8_t RxDataBuff[2];
  uint8_t parity;
  uint16_t kneeOutputCountTemp;
  uint16_t kneeOutputCount; //(0-16383) 14bit quantization levels
  int32_t debug_kneeOutputDeg; //(0-360) degree
} AEAT9922;

struct ENCODER_Engine{
  int8_t direction;
  int16_t u16counter_ABI[2];
  int16_t encDelta_ABI;
  int32_t s32counter_ABI;
  float gearRatio;
}
incENCODER = {.direction = -1};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define IS_DRIVE0
#define IS_DRIVE1
// #define IS_DRIVE2
// #define IS_DRIVE3

#define DRIVE0_LEFT_ID 0
#define DRIVE1_LEFT_ID 1
#define DRIVE2_LEFT_ID 2
#define DRIVE3_LEFT_ID 3

#define DRIVE0_RIGHT_ID 0
#define DRIVE1_RIGHT_ID 1
#define DRIVE2_RIGHT_ID 2
#define DRIVE3_RIGHT_ID 3

/* SPI commands */

// #define AMT22_RESET     0x60
// #define AMT22_ZERO      0x70
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
DriveFeedback fb; // for now: fb[0] is DRIVE0_RIGHT_ID

// absolute encoder variable
AEAT9922 kneeAbs;
uint8_t abs_read_addr[2] = {0xC0, 0x3F};
// uint8_t abs_read_addr[2] = {0xC0, 0x21};
uint8_t errorAddr = 0x21;
uint8_t crcresult;

uint32_t errorCount;

//vesc variable
uint8_t  vesc_tx_buff[10];
uint16_t vesc_bufflength;
int32_t TMC_torque_command = 0;

/**BEGIN>> Controller vialable <<**/
float Current_U = 0;
/**END>> Controller vialable <<**/


// test variable
uint32_t RxcallbackReCeived = 0;
uint32_t RxcallbackDoned = 0;
uint8_t firstTimeTxFDCAN2 = 0;
volatile uint32_t rxfifofilllevel = 0;
uint32_t txfifofreelevelBefore = 0;
uint32_t txfifofreelevelAfter = 0;
uint32_t countTxError = 0;
uint32_t countRxError1 = 0;
uint32_t countRxError2 = 0;
uint32_t countRxError3 = 0;
uint32_t canRestartCounter = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//! @brief Initialize CAN bus
void CAN_Init();
void CAN_UpdateCommand(FDCAN_HandleTypeDef *hfdcan, float *Current);
void CAN2_SendIqRef(DriveFeedback *fb);
uint8_t isEvenParity(uint16_t data); // 1 is even parity


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t isEvenParity(uint16_t data){
    uint16_t y = data ^ (data >> 1);
    y = y ^ (y >> 2);
    y = y ^ (y >> 4);
    y = y ^ (y >> 8);
 
    // Rightmost bit of y holds the parity value
    // if (y&1) is 0 then parity is odd else even
    if(y & 1) {return 0;} else {return 1;};

}
void CAN_DISABLE_ALL(){
// LOW: normal mode, HIGH: stand by mode (TJA1042)
HAL_GPIO_WritePin(canfd2_stb_GPIO_Port, canfd2_stb_Pin, GPIO_PIN_SET);
}

void CAN_ENABLE_ALL(){
// LOW: normal mode, HIGH: stand by mode (TJA1042)
HAL_GPIO_WritePin(canfd2_stb_GPIO_Port, canfd2_stb_Pin, GPIO_PIN_RESET);
}


static void CAN_Start(FDCAN_HandleTypeDef *hfdcan,
                      FDCAN_FilterTypeDef *filter,
                      const uint32_t ActiveITs)
{
  if (HAL_FDCAN_ConfigFilter(hfdcan, filter) != HAL_OK) { while(1); }
  // Configure global filter
  if (HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) { while(1); }
  // Activate Rx FIFO new message notification
  if (HAL_FDCAN_ActivateNotification(hfdcan, ActiveITs, 0) != HAL_OK) { while(1); }
  // Configure and enable Tx Delay Compensation, required for BRS mode.
  // TdcOffset default recommended value: DataTimeSeg1 * DataPrescaler
  // TdcFilter default recommended value: 0
  if (HAL_FDCAN_ConfigTxDelayCompensation(hfdcan, hfdcan->Init.DataTimeSeg1 * hfdcan->Init.DataPrescaler, 0) != HAL_OK) { while(1); }
  // Enable Tx delay compensation
  if (HAL_FDCAN_EnableTxDelayCompensation(hfdcan) != HAL_OK) { while(1); }
  // Start the FDCAN module
  if (HAL_FDCAN_Start(hfdcan) != HAL_OK) { while(1); }
}

static void CAN_ConfigOverwriteMode(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo){
  if (HAL_FDCAN_ConfigRxFifoOverwrite(hfdcan, RxFifo, FDCAN_RX_FIFO_OVERWRITE) != HAL_OK) {
   while(1);
  }
}

void CAN_Init()
{
  // Disable CAN transceiver
  CAN_DISABLE_ALL();
  
  FDCAN_FilterTypeDef filter = {0};
  filter.IdType       = FDCAN_STANDARD_ID;
  filter.FilterIndex  = 0;
  filter.FilterType   = FDCAN_FILTER_MASK;

  // Start CAN2
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;

#ifdef IS_DRIVE0
  filter.FilterID1    = DRIVE0_RIGHT_ID;
#endif
#ifdef IS_DRIVE1
  filter.FilterID1    = DRIVE1_RIGHT_ID;
#endif
#ifdef IS_DRIVE2
  filter.FilterID1    = DRIVE2_RIGHT_ID;
#endif
#ifdef IS_DRIVE3
  filter.FilterID1    = DRIVE3_RIGHT_ID;
#endif

  filter.FilterID2    = 0x7FF;
  // CAN_ConfigOverwriteMode(&hfdcan2, FDCAN_RX_FIFO0);
  CAN_Start(&hfdcan2, &filter, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);

  // Enable CAN tranceiver
  CAN_ENABLE_ALL();
}


void canRestartPeriPheral(){

  // if we have two can instance, we will need to fix this later.

  // deinit the canfd peripheral
  if (HAL_FDCAN_DeInit(&hfdcan2) != HAL_OK) { while(1) ;};
  // init
  MX_FDCAN2_Init();
  // config both can instance and start both can instance
  CAN_Init();

  // track restart behaviour
  canRestartCounter++;


}



void CAN2_SendIqRef(DriveFeedback *fb)
{
  if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) < 2) { 
    countTxError++;
    if (countTxError >= 500) {countTxError = 0; canRestartPeriPheral();};
    HAL_GPIO_WritePin(led_red_GPIO_Port, led_red_Pin, GPIO_PIN_SET);
    return; 
  }
  HAL_GPIO_WritePin(led_red_GPIO_Port, led_red_Pin, GPIO_PIN_RESET);
  // Set header
  FDCAN_TxHeaderTypeDef header;
  header.IdType = FDCAN_STANDARD_ID;
  header.TxFrameType = FDCAN_DATA_FRAME;
  header.DataLength = FDCAN_DLC_BYTES_8;
  header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  header.BitRateSwitch = FDCAN_BRS_ON;
  header.FDFormat = FDCAN_FD_CAN;
  header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  header.MessageMarker = 0;
  // send feedback

#ifdef IS_DRIVE0
  header.Identifier = DRIVE0_RIGHT_ID;
#endif
#ifdef IS_DRIVE1
  header.Identifier = DRIVE1_RIGHT_ID;
#endif
#ifdef IS_DRIVE2
  header.Identifier = DRIVE2_RIGHT_ID;
#endif
#ifdef IS_DRIVE3
  header.Identifier = DRIVE3_RIGHT_ID;
#endif

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &header, (uint8_t*)fb) != HAL_OK) { return; }
}



void CAN_UpdateCommand(FDCAN_HandleTypeDef *hfdcan, float *Current)
{
  // Retrieve message
  FDCAN_RxHeaderTypeDef header;
  uint8_t rx_buf[4]; // receive current from master
  if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, rx_buf) != HAL_OK) { 
    countRxError2++;
    return;
  }
  // Check header
  if (header.IdType != FDCAN_STANDARD_ID || header.RxFrameType != FDCAN_DATA_FRAME || 
      header.FDFormat != FDCAN_FD_CAN || header.DataLength != FDCAN_DLC_BYTES_4) { 
        countRxError3++;
        return; 
      }
  // Update data from master
  *Current = *(float *)rx_buf;
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
  MX_FDCAN2_Init();
  MX_I2C4_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  // can initialize
  CAN_Init();
  // timer for encoder
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // encoder timer
  // timer initialize // loop 2000 hz
  HAL_TIM_Base_Start_IT(&htim6);

  // prepare data to send
  fb.vel = 56.0f;
  fb.pos = 25.0f;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    HAL_GPIO_WritePin(CS_KneeABS_GPIO_Port, CS_KneeABS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, abs_read_addr, 2, 100);
    HAL_GPIO_WritePin(CS_KneeABS_GPIO_Port, CS_KneeABS_Pin, GPIO_PIN_SET);


    HAL_GPIO_WritePin(CS_KneeABS_GPIO_Port, CS_KneeABS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Receive(&hspi2, kneeAbs.RxDataBuff, 2, 100);
    HAL_GPIO_WritePin(CS_KneeABS_GPIO_Port, CS_KneeABS_Pin, GPIO_PIN_SET);

    kneeAbs.errorFlag = (kneeAbs.RxDataBuff[0] >> 6) & 1U;
    kneeAbs.kneeOutputCountTemp = (uint16_t)( kneeAbs.RxDataBuff[0] << 8 | kneeAbs.RxDataBuff[1]);
    kneeAbs.parity = isEvenParity(kneeAbs.kneeOutputCountTemp);
    if(!kneeAbs.errorFlag && isEvenParity(kneeAbs.kneeOutputCountTemp)){
      kneeAbs.kneeOutputCount =  kneeAbs.kneeOutputCountTemp & 0x3FFF;
    }else{
      errorCount++;
    }


    HAL_Delay(2);

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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 75;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// FDCAN2 RXfifo 0
//! @brief Rx FIFO 0 callback
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  RxcallbackReCeived++;

  // Skip if no new message
  if(!(RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)) { 
    countRxError1++;
    return; 
  }

  // Update feedback data
  CAN_UpdateCommand(hfdcan, &Current_U);
  
  // send command back to master
  CAN2_SendIqRef(&fb);
  // test
  RxcallbackDoned++;

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if(htim->Instance == TIM6){ // loop 2000 hz

    // send data CANFD back to master
    // txfifofreelevelBefore = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2);
    // 
    // rxfifofilllevel += HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0);
    // txfifofreelevelAfter = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2);

    // firstTimeTxFDCAN2++;
    //read encoder
    incENCODER.u16counter_ABI[0] = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    incENCODER.encDelta_ABI = (incENCODER.u16counter_ABI[0] - incENCODER.u16counter_ABI[1]);
    incENCODER.u16counter_ABI[1] = incENCODER.u16counter_ABI[0];
    incENCODER.s32counter_ABI += incENCODER.direction * incENCODER.encDelta_ABI;

    // sending uart
    //TMC_torque_command = 1000 is 1 A (current).
    TMC_torque_command = Current_U;
    vesc_bufflength = VESC_UARTsetCurrent(vesc_tx_buff, TMC_torque_command);
    HAL_UART_Transmit_IT(&huart3, vesc_tx_buff, vesc_bufflength);
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
  __disable_irq();
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
