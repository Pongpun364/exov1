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
#include "UNVScontroller_V3.h"
#include "PVESC_UART.h"
#include "checksum.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Drive Feedback
typedef struct {
  float vel;              //!< velocity (rad/s)
  float pos;              //!< position (rad)
  float absPos; // !< Absolute position (rad)
} DriveFeedback;

typedef struct {
  uint8_t TxCommand;
  uint8_t errorFlag;
  volatile uint8_t RxDataBuff[2];
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
incENCODER = {.direction = 1};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// left side
// #define IS_DRIVE0 0
#define IS_DRIVE1 1
// right side
// #define IS_DRIVE2 2
// #define IS_DRIVE3 3


#ifndef PI
  #define PI   3.14159265358979323846264338327950 
#endif


#define RadToDeg(X) (X * (180.0 / PI))
#define DegToRad(X) (X * (PI / 180.0))
// not implement ratio
//#define CntToDeg(X) (X * (360.0/2000.0))
//#define DegToCnt(X) (X * (2000.0/360.0))

// implement gear ratio (aps 5055s)
//  #define CntToDeg(X) (X * ((360.0 * 360.0)/(2000.0*35400.0)))
//  #define DegToCnt(X) (X * ((2000.0*35400.0)/(360.0 * 360.0)))
// note
// 360 deg output = 35400 deg input

// implement gear ratio (flipsky 6354 motor)
 #define CntToDeg(X) (X * ((360.0 * 3.0)/(2000.0*284.0)))
 #define DegToCnt(X) (X * ((2000.0*284.0)/(360.0 * 3.0)))

/* SPI commands */

// #define AMT22_RESET     0x60
// #define AMT22_ZERO      0x70
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
DriveFeedback fb; // for now: fb[0] is DRIVE0_ID


/**BEGIN>> Controller vialable <<**/
// CONTROLLER
CTRL_TypeDef knee_left;
uint8_t controller_is_enable = 0;
float pos_setpoint, pos_feedback;
float vel_setpoint, vel_feedback;
float cur_setpoint, cur_feedback;

float pos_feedback_knee_L = 0;
float vel_feedback_knee_L = 0;

float Current_U = 0;

// user input to controller
float userPos, userVel; // Position is in degree (0-360), Velocity is in Deg/Second.
float u; // current value for debug
float posError;
// sine parameter
uint16_t t_max = 1000;
uint32_t t_run, lastTick;
int32_t debug_pos = 0;

/**END>> Controller vialable <<**/


// absolute encoder variable
AEAT9922 kneeAbs;
uint8_t abs_read_addr[2] = {0xC0, 0x3F};
//end absolute encoder variable


//vesc variable
uint8_t  vesc_tx_buff[10];
uint16_t vesc_bufflength;
int32_t TMC_torque_command = 0;
// end vesc variable


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
uint32_t errorCount;

uint8_t txcplt;
uint8_t rxcplt;
uint8_t txrxcplt;

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
  filter.FilterID1    = IS_DRIVE0;
#endif
#ifdef IS_DRIVE1
  filter.FilterID1    = IS_DRIVE1;
#endif
#ifdef IS_DRIVE2
  filter.FilterID1    = IS_DRIVE2;
#endif
#ifdef IS_DRIVE3
  filter.FilterID1    = IS_DRIVE3;
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
  HAL_Delay(100);
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
  header.DataLength = FDCAN_DLC_BYTES_12;
  header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  header.BitRateSwitch = FDCAN_BRS_ON;
  header.FDFormat = FDCAN_FD_CAN;
  header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  header.MessageMarker = 0;
  // send feedback

#ifdef IS_DRIVE0
  header.Identifier = IS_DRIVE0;
#endif
#ifdef IS_DRIVE1
  header.Identifier = IS_DRIVE1;
#endif
#ifdef IS_DRIVE2
  header.Identifier = IS_DRIVE2;
#endif
#ifdef IS_DRIVE3
  header.Identifier = IS_DRIVE3;
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

// CONTROLLER

  // float kUNVS_wheel[3] = {0.2f, 1.0f, 1.0f};       //controller gain wheel universal controller for 1st order system ka=0.6 kr=1 kb=2(could be increased)
  float kUNVS_kneeleft[3] = {5.0f, 0.15f, 0.0f};     // 5, 0.15 ,0  date09/03/23
  float intLimit = 10.0f;                          //saturation value of controller integral terms
  float uLimit   = 10.0f;                          //saturation value of controller 
  float dZone    = 0.001f;                         //deadzone of controller
  float dtCtrl   = 0.001f; 
  float velLimit = 100.0f;
  
  // InitCtrl(&motor_wheel_L, dtCtrl, kUNVS_wheel, dZone, intLimit, uLimit, velLimit);
  // InitCtrl(&motor_wheel_R, dtCtrl, kUNVS_wheel, dZone, intLimit, uLimit, velLimit);
  InitCtrl(&knee_left, dtCtrl, kUNVS_kneeleft, dZone, intLimit, uLimit, velLimit);
  // InitCtrl(&motor_bws_R, dtCtrl, kUNVS_bws, dZone, intLimit, uLimit, velLimit);

  // END INITIALIZE CONTROLLER //

  // CAN initialize
  CAN_Init();
  // timer for encoder
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // encoder timer
  // timer initialize // loop 1000 hz
  HAL_TIM_Base_Start_IT(&htim6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // sine wave position 1000 hz
    // if(uwTick - lastTick >= 1){
    //   lastTick = uwTick;
    //   // tmax = 1000 -> 1 sec to finish sine loop
    //   userPos = 2000.0f * sin(2.0f * PI * (t_run++ % t_max) / (t_max - 1));
    //   debug_pos = userPos;
    // }
    
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

  // // Update feedback data
  // CAN_UpdateCommand(hfdcan, &Current_U);

  // // 4.) Sending uart to control motor using current
  // //TMC_torque_command = 1000 is 1 A (current).
  // TMC_torque_command = Current_U * 1000;
  // vesc_bufflength = VESC_UARTsetCurrent(vesc_tx_buff, TMC_torque_command);
  // HAL_UART_Transmit_IT(&huart3, vesc_tx_buff, vesc_bufflength);

  // // send command back to master
  // CAN2_SendIqRef(&fb);



  // test
  RxcallbackDoned++;

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if(htim->Instance == TIM6){ // loop 1000 hz

    // 1.) read encoder

    //incremental encoder
    incENCODER.u16counter_ABI[0] = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    incENCODER.encDelta_ABI = (incENCODER.u16counter_ABI[0] - incENCODER.u16counter_ABI[1]);
    incENCODER.u16counter_ABI[1] = incENCODER.u16counter_ABI[0];
    incENCODER.s32counter_ABI += incENCODER.direction * incENCODER.encDelta_ABI;

    // //absolute encoder
    // HAL_GPIO_WritePin(CS_KneeABS_GPIO_Port, CS_KneeABS_Pin, GPIO_PIN_RESET);
    // HAL_SPI_Transmit_IT(&hspi2, abs_read_addr, 2);

    // // update the fb parameter
    // fb.pos = CntToDeg(incENCODER.s32counter_ABI);    //[degree] 
    // fb.vel = CntToDeg(incENCODER.encDelta_ABI) * 1000.0f;   //[deg/s]
    // // fb.absPos = kneeAbs.kneeOutputCount * (360.0/16383.0); //offset = 6.81 deg
    // fb.absPos = userPos; //offset = 6.81 deg


    // 3.) COMPUTE control parameter (position control)
    //3.1 knee_left

    pos_feedback =  CntToDeg(incENCODER.s32counter_ABI);    //[deg] 
    //vel_feedback = DegToRad(CntToDeg(incENCODER.encDelta_ABI)) * 1000.0f;   //[deg/s]
    // y=mm2rad*BWSComm_L.h_lever;
    posError=userPos - pos_feedback;
    knee_left.err=posError;
    // PosCtrlPID(&motor_bws_L, &u); //standard PID  // bws gain P=40 I=20 D=10
    PosCtrl(&knee_left, &u); //  bws gain Ka=150 Kr=0.25(quite 0.2) Kb=20


    // 4.) Sending uart to control motor using current
    //TMC_torque_command = 1000 is 1 A (current).
    TMC_torque_command = u * 1000 ;
    vesc_bufflength = VESC_UARTsetCurrent(vesc_tx_buff, TMC_torque_command);
    HAL_UART_Transmit_IT(&huart3, vesc_tx_buff, vesc_bufflength);


  }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if(hspi->Instance == SPI2){
    HAL_GPIO_WritePin(CS_KneeABS_GPIO_Port, CS_KneeABS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CS_KneeABS_GPIO_Port, CS_KneeABS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Receive_IT(&hspi2, (uint8_t *)kneeAbs.RxDataBuff, 2);
  }
}
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if(hspi->Instance == SPI2){
    HAL_GPIO_WritePin(CS_KneeABS_GPIO_Port, CS_KneeABS_Pin, GPIO_PIN_SET);

    // process the received data
    kneeAbs.errorFlag = (kneeAbs.RxDataBuff[0] >> 6) & 1U;
    kneeAbs.kneeOutputCountTemp = (uint16_t)( ((uint8_t)kneeAbs.RxDataBuff[0] << 8) | ((uint8_t)kneeAbs.RxDataBuff[1]));
    kneeAbs.parity = isEvenParity(kneeAbs.kneeOutputCountTemp);
    if(!kneeAbs.errorFlag && isEvenParity(kneeAbs.kneeOutputCountTemp)){
      kneeAbs.kneeOutputCount =  kneeAbs.kneeOutputCountTemp & 0x3FFF;
    }else{
      errorCount++;
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
