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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CoVAPSy_buzzer.h"
#include "CoVAPSy_bno055.h"
#include "CoVAPSy_moteurs.h"
#include "u8g.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BP_ENFONCE 0
#define BP_RELACHE 1
#define BUTEE_DROITE 1630
#define BUTEE_GAUCHE 1160
#define DISTANCE_1_TOUR_AXE_TRANSMISSION_MM 79
#define ADRES_SRF10 0x70
#define ADDRESS_TF051 0x3C

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float vitesse_mesuree_m_s = 0;
uint32_t vitesse_mesuree_mm_s = 0;
uint32_t lectures_ADC[3];
uint8_t SPI_TxBuffer[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}; 	// buffer pour la transmission SPI
uint8_t SPI_RxBuffer[6] ={}; 				// Buffer pour la réception SPI
int16_t roll, distance_US;
uint8_t temp;
uint16_t telemetre_gauche, telemetre_droit;
uint32_t drapeau = 0;
static u8g_t u8g;

// Custom flags
int errorFlag, SPIFlag = 0;
int SPIActive = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t control = 0;
void u8g_Delay(uint16_t val) {
    HAL_Delay(val);
}
void u8g_xMicroDelay(uint16_t val) {
    static uint32_t i, j;
    static uint32_t freq;
    freq = HAL_RCC_GetSysClockFreq() / 1000000;

    for (i = 0; i < val;) {
        for (j = 0; j < freq; ++j) {
            ++j;
        }
        ++i;
    }
}
void u8g_MicroDelay(void) {
    u8g_xMicroDelay(1);
}
void u8g_10MicroDelay(void) {
    u8g_xMicroDelay(10);
}

uint8_t u8g_com_arm_stm32_sh_i2c_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr) {
    switch (msg) {
    case U8G_COM_MSG_STOP:
        break;

    case U8G_COM_MSG_INIT:
        u8g_MicroDelay();
        break;

    case U8G_COM_MSG_ADDRESS:
        if (arg_val == 0) {
            control = 0;
        } else {
            control = 0x40;
        }
        u8g_10MicroDelay();
        break;

    case U8G_COM_MSG_WRITE_BYTE: {
        HAL_I2C_Mem_Write(&hi2c1, ADDRESS_TF051<<1, control, 1, &arg_val, 1, 10000);
    }
        break;

    case U8G_COM_MSG_WRITE_SEQ:
    case U8G_COM_MSG_WRITE_SEQ_P: {
        HAL_I2C_Mem_Write(&hi2c1, ADDRESS_TF051<<1, control, 1, arg_ptr, arg_val, 10000);
    }

        break;
    }
    return 1;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint32_t bp1,bp2,bp1_old=0,bp2_old=0;
	uint8_t donnees_Tx_i2c[8];
	uint8_t donnees_Rx_i2c[8];
	char text[50];
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  buzzer_start();
  buzzer_gamme();

  bno055_init();

  u8g_InitComFn(&u8g, &u8g_dev_sh1106_128x64_i2c, u8g_com_arm_stm32_sh_i2c_fn);
  u8g_Begin(&u8g);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Code par defaut : ---------------------------

	  //démarrage de la conversion ADC des 3 canaux
	  HAL_ADC_Start_DMA(&hadc1, lectures_ADC, 3);

	  //lecture des boutons
	  bp1 = HAL_GPIO_ReadPin(BP1_GPIO_Port, BP1_Pin);
	  bp2 = HAL_GPIO_ReadPin(BP2_GPIO_Port, BP2_Pin);

	  //détection front descendant sur bp1
	  if((bp1 == BP_ENFONCE) && (bp1_old == BP_RELACHE))
	  {
		  //buzzer_start_frequency_Hz(1760);
		  //HAL_Delay(500);
		  //buzzer_stop();
	  }

	  //détection front descendant sur bp2
	  if((bp2 == BP_ENFONCE) && (bp2_old == BP_RELACHE))
	  {
		  buzzer_start_frequency_Hz(440);
		  HAL_Delay(100);
		  buzzer_start_frequency_Hz(840);
		  HAL_Delay(100);
		  buzzer_start_frequency_Hz(440);
		  HAL_Delay(100);
		  buzzer_start_frequency_Hz(840);
		  HAL_Delay(100);
		  buzzer_stop();
		  HAL_SPI_TransmitReceive_IT(&hspi3, SPI_TxBuffer, SPI_RxBuffer, 6);
		  //changement d'état de la Led4
		  HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
		  SPIActive = 1;
	  }
	  //lecture ultrason
		donnees_Tx_i2c[0]=0x02;
		while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
		HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)ADRES_SRF10<<1, donnees_Tx_i2c, 1, 1000);
		while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
		HAL_I2C_Master_Receive(&hi2c1, (uint16_t)ADRES_SRF10<<1, donnees_Rx_i2c, 2, 1);
		distance_US = (uint16_t)(donnees_Rx_i2c[0]<<8) + donnees_Rx_i2c[1];

	  roll = bno055_lecture_16bits(EULER_ROLL_16bits);
	  temp = bno055_lecture_8bits(TEMPERATURE_8bits);

	  // si il n'y a pas eu de lecture de la vitesse récemment
	  if(drapeau==0)
	  {
		  vitesse_mesuree_mm_s = 0;
		  vitesse_mesuree_m_s = 0;
	  }
	  drapeau = 0;

	  u8g_FirstPage(&u8g);
		do {
			u8g_SetFont(&u8g, u8g_font_profont12);
			sprintf(text,"vitesse %5u mm/s", (unsigned int)vitesse_mesuree_mm_s);
			u8g_DrawStr(&u8g, 0, 12,  text);
			sprintf(text,"roulis_RAW : %5d", roll);
			u8g_DrawStr(&u8g, 0, 24,  text);
			sprintf(text,"dist_US : %5d cm", distance_US);
			u8g_DrawStr(&u8g, 0, 36,  text);
			if (SPIActive)
			{
				sprintf(text,"Receive %x %x %x", SPI_RxBuffer[0], SPI_RxBuffer[1], SPI_RxBuffer[2]);
				u8g_DrawStr(&u8g, 0, 48,  text);
				sprintf(text,"  SPI : %x %x %x", SPI_RxBuffer[3], SPI_RxBuffer[4], SPI_RxBuffer[5]);
				u8g_DrawStr(&u8g, 0, 60,  text);
			}
			else
			{
				sprintf(text,"  Press BP2 to  ");
				u8g_DrawStr(&u8g, 0, 48,  text);
				sprintf(text,"  activate SPI  ");
				u8g_DrawStr(&u8g, 0, 60,  text);
			}
			//sprintf(text,"telem_g_RAW : %5u", (unsigned int)lectures_ADC[0]);
			//u8g_DrawStr(&u8g, 0, 48,  text);
			//sprintf(text,"telem_d_RAW : %5u", (unsigned int)lectures_ADC[1]);
			//u8g_DrawStr(&u8g, 0, 60,  text);
		} while (u8g_NextPage(&u8g));

	  //Demande de lecture ultrason
	  donnees_Tx_i2c[0]=0x00;
	  donnees_Tx_i2c[1]=0x51;
	  while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
	  HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)ADRES_SRF10<<1, donnees_Tx_i2c, 2, 1000);

	  // 1,2,3 : distance
	  SPI_TxBuffer[0] = (char)distance_US;
	  SPI_TxBuffer[1] = (char)distance_US;
	  SPI_TxBuffer[2] = (char)distance_US;
	  // 255 placeholder
	  SPI_TxBuffer[3] = 0xFF;
	  // Fourche ENVOI EN DECIMETRE
	  // Valeurs : 4 entier
	  // 		   5 decimal
	  SPI_TxBuffer[4] = (char)(vitesse_mesuree_mm_s/100);
	  SPI_TxBuffer[5] = (char)(vitesse_mesuree_mm_s - (vitesse_mesuree_mm_s/100)*100);

	  // DELAI
	  HAL_Delay(100);
	  if (SPIFlag)
	  {
		  // Illustrateur
		  HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
		  SPIFlag = 0;
	  }
	  else
	  {
		  //HAL_SPI_Receive_IT(&hspi3, SPI_TxBuffer, 6);
	  }


	  //attente de la fin de la conversion ADC, si jamais ce n'est pas encore fini
	  HAL_ADC_PollForConversion(&hadc1, 1);

	  //sauvegarde des valeurs de bp1 et bp2 pour la détection des fronts
	  bp1_old = bp1;
	  bp2_old = bp2;
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim)
{
	static uint32_t mesure_precedente_us=0, mesure_us, indice=0, i;
	static uint32_t tableau_intervalles_us[16]={};
	static float coefficient_distance_par_intervalle_us = DISTANCE_1_TOUR_AXE_TRANSMISSION_MM *1000 / 16.0;
	static uint32_t somme_intervalles_us = 0;
	uint32_t nb_intervalles=0;
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin,1);
	mesure_us = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1); // ou TIM2->CCR1
	if((mesure_us-mesure_precedente_us) >= 300) //si mesure cohérente (pas un glitch):
	{
		if((mesure_us > (mesure_precedente_us+100000)) || ((mesure_us-100000) > mesure_precedente_us)) //cas d'un nouveau départ
		{
			for(indice=0;indice<16;indice++)
			{
				tableau_intervalles_us[indice] = 0;
			}
			indice=0;
		}
		else //cas où on tourne depuis plus d'un intervalle
		{
			drapeau = 1;
			tableau_intervalles_us[indice] = mesure_us - mesure_precedente_us; //on sauvegarde la nouvelle mesure dans le tableau
			//On fait une moyenne sur 10 ms au plus ou 16 valeurs.
			somme_intervalles_us = 0;
			i= indice;
			do{
				if(tableau_intervalles_us[i]==0)
						break;
				somme_intervalles_us += tableau_intervalles_us[i];
				i = (i - 1)%16;
				nb_intervalles++;
			}while ((somme_intervalles_us<100000) && (nb_intervalles < 16));
			indice = (indice+1)%16; // on incrémente l'indice avec retour à 0 pour indice = 16
			vitesse_mesuree_m_s = coefficient_distance_par_intervalle_us * nb_intervalles / somme_intervalles_us;
			vitesse_mesuree_mm_s = 1000*vitesse_mesuree_m_s;
		}
		mesure_precedente_us = mesure_us;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef * hspi)
{
	if (HAL_SPI_Receive_IT(&hspi3, SPI_RxBuffer, 6) != HAL_OK)
	{
		errorFlag = 99;
    }
	SPIFlag = 1;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef * hspi)
{
	if (HAL_SPI_Transmit_IT(&hspi3, SPI_TxBuffer, 6) != HAL_OK)
	{
		errorFlag = 99;
    }
	SPIFlag = 1;
}


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef * hspi)
{
	if (HAL_SPI_TransmitReceive_IT(&hspi3, SPI_TxBuffer, SPI_RxBuffer, 6) != HAL_OK)
	{
		errorFlag = 99;
    }
	SPIFlag = 1;
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
