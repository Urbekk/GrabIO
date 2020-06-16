/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stdio.h"
#include "stdlib.h"
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct point {
	float x;
	float y;
	float z;
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define L1 186
#define L2 339
#define TOLERANCE 1
#define BAZA 240
#define ALFA 0
#define BETA 85//90
#define GAMMA 95//90
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
int tmp, tmp2;
uint8_t Received[5];
volatile int flaga = 0;
volatile int i, j, k, l;
volatile int max, max1, max2, max0;
volatile int odczyt;
volatile int kalibracja = 0;
volatile int positions;
volatile int obrot = 0;
volatile int flagaF0, flagaF1, flagaF2;
volatile int goFABRIK = 0;
volatile int workspace = 0;
struct point pStart = { .x=0, .y=0, .z=BAZA };
struct point pMiddle = { .x = cosf(ALFA * M_PI / 180.0) * cosf(BETA * M_PI / 180.0) * L1,
						 .y = sinf(ALFA * M_PI / 180.0) * cosf(BETA * M_PI / 180.0) * L1,
						 .z = BAZA + cosf((90.0 - BETA) * M_PI / 180.0) * L1 };
struct point pEnd = { .x = cosf(ALFA * M_PI / 180.0) * (cosf(BETA * M_PI / 180.0) * L1 + sinf((BETA + GAMMA - 90.0) * M_PI / 180.0) * L2),
					  .y = sinf(ALFA * M_PI / 180.0) * (cosf(BETA * M_PI / 180.0) * L1 + sinf((BETA + GAMMA - 90.0) * M_PI / 180.0) * L2),
					  .z = BAZA + cosf((90.0 - BETA) * M_PI / 180.0) * L1 - cosf((BETA + GAMMA - 90.0) * M_PI / 180.0) * L2 };
struct point target;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM9_Init(void);
/* USER CODE BEGIN PFP */

void kierunek(int a, int b, int c, int d) {
	HAL_GPIO_WritePin(kierunkiL1_GPIO_Port, kierunkiL1_Pin, a);
	HAL_GPIO_WritePin(kierunkiL2_GPIO_Port, kierunkiL2_Pin, b);
	HAL_GPIO_WritePin(kierunkiP1_GPIO_Port, kierunkiP1_Pin, c);
	HAL_GPIO_WritePin(kierunkiP2_GPIO_Port, kierunkiP2_Pin, d);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim == &htim2) {
		if (flaga == 1) {
			kalibracja++;
			i++;
			j--;
			if (i == 75) {
				j = 80;
				kalibracja = 0;
			}
			if (kalibracja == 7) {
				j++;
				kalibracja = 0;
			}
			TIM4->CCR4 = i;
			TIM4->CCR3 = j;
			if (i == max) {
				flaga = 0;
				HAL_TIM_Base_Stop_IT(&htim2);
			}
		}
		if (flaga == -1) {
			kalibracja++;
			i--;
			j++;
			if (i == 75) {
				j = 80;
				kalibracja = 0;
			}
			if (kalibracja == 7) {
				j--;
				kalibracja = 0;
			}
			TIM4->CCR4 = i;
			TIM4->CCR3 = j;
			if (i == max) {
				flaga = 0;
				HAL_TIM_Base_Stop_IT(&htim2);
			}
		}
		if (flaga == 2) {
			k++;
			TIM4->CCR2 = k;
			if (k == max) {
				flaga = 0;
				HAL_TIM_Base_Stop_IT(&htim2);
			}
		}
		if (flaga == 3) {
			k--;
			TIM4->CCR2 = k;
			if (k == max) {
				flaga = 0;
				HAL_TIM_Base_Stop_IT(&htim2);
			}
		}
		if (flaga == 4) {
			l++;
			TIM4->CCR1 = l;
			if (l == max) {
				flaga = 0;
				HAL_TIM_Base_Stop_IT(&htim2);
			}
		}
		if (flaga == 5) {
			l--;
			TIM4->CCR1 = l;
			if (l == max) {
				flaga = 0;
				HAL_TIM_Base_Stop_IT(&htim2);
			}
		}

		/* do fabrika */
		if (flaga == 10) {
			if (flagaF1 == 1) {
				kalibracja++;
				i++;
				j--;
				if (i == 75) {
					j = 80;
					kalibracja = 0;
				}
				if (kalibracja == 7) {
					j++;
					kalibracja = 0;
				}
				TIM4->CCR4 = i;
				TIM4->CCR3 = j;
				if (i == max1) {
					flagaF1 = 0;
				}
			}
			if (flagaF1 == -1) {
				kalibracja++;
				i--;
				j++;
				if (i == 75) {
					j = 80;
					kalibracja = 0;
				}
				if (kalibracja == 7) {
					j--;
					kalibracja = 0;
				}
				TIM4->CCR4 = i;
				TIM4->CCR3 = j;
				if (i == max1) {
					flagaF1 = 0;
				}
			}
			if (flagaF2 == 1) {
				k++;
				TIM4->CCR2 = k;
				if (k == max2) {
					flagaF2 = 0;
				}
			}
			if (flagaF2 == -1) {
				k--;
				TIM4->CCR2 = k;
				if (k == max2) {
					flagaF2 = 0;
				}
			}
			if (flagaF1 == 0 && flagaF2 == 0) {
				flaga = 0;
				flagaF1=0;
				flagaF2=0;
				HAL_TIM_Base_Stop_IT(&htim2);
			}
		}
	}

	if (htim == &htim9) { //BEDAC W WHILE PRZESTAJE DZIALAC UART
		if (obrot == 1) {
			if (TIM5->CNT <= positions + 500) { //hamowanie & poprzednie 500
				kierunek(1, 1, 1, 1);
				obrot = 0;
				HAL_TIM_Base_Stop_IT(&htim9);
			}
		}
		if (obrot == -1) {
			if (TIM5->CNT >= positions - 500) { //poprzednie 500
				kierunek(1, 1, 1, 1);
				obrot = 0;
				HAL_TIM_Base_Stop_IT(&htim9);
			}
		}
	}
	if (obrot == 2) {
		if (TIM5->CNT <= positions) { //dla mniejszych katow
			kierunek(1, 1, 1, 1);
			obrot = 0;
			HAL_TIM_Base_Stop_IT(&htim9);
			if (flagaF1 != 0 || flagaF2 != 0) {
				flaga = 10;
				HAL_TIM_Base_Start_IT(&htim2);
			}
		}
	}
	if (obrot == -2) {
		if (TIM5->CNT >= positions) { //dla mniejszych katow
			kierunek(1, 1, 1, 1);
			obrot = 0;
			HAL_TIM_Base_Stop_IT(&htim9);
			if (flagaF1 != 0 || flagaF2 != 0) {
				flaga = 10;
				HAL_TIM_Base_Start_IT(&htim2);
			}
		}
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	uint8_t Data[50]; // Tablica przechowujaca wysylana wiadomosc.
	uint16_t size = 0; // Rozmiar wysylanej wiadomosci
	uint8_t tmp[3];
	tmp[0] = Received[1];
	tmp[1] = Received[2];
	tmp[2] = Received[3];

	switch (Received[0]) {
	case 'a':
		odczyt = atoi((char*) tmp);
		size = sprintf((char*) Data, "Odebrano znak: %c%c%c%c\n\r", Received[0],
				Received[1], Received[2], Received[3]);
		max = ((((float) odczyt / 180) * 90) + 30);
		i = TIM4->CCR4;
		j = TIM4->CCR3;
		if (max == i)
			break;
		if (max > i)
			flaga = 1;
		else
			flaga = -1;
		HAL_TIM_Base_Start_IT(&htim2);
		break; //
	case 'b':
		odczyt = atoi((char*) tmp);
		size = sprintf((char*) Data, "Odebrano znak: %c%c%c%c\n\r", Received[0],
				Received[1], Received[2], Received[3]);
		max = (((-0.53) * (float) odczyt) + (119.3));
		if (max > 98)
			max = 98;
		if (max < 25)
			max = 25;
		//OBSLUGA BLEDU
		k = TIM4->CCR2;
		if (max == k)
			break;
		if (max > k)
			flaga = 2;
		else
			flaga = 3;
		HAL_TIM_Base_Start_IT(&htim2);
		if(odczyt==90){
			workspace = 1;
		}
		break;
	case 'c':
		odczyt = atoi((char*) tmp);
		size = sprintf((char*) Data, "Odebrano znak: %c%c%c%c\n\r", Received[0],
				Received[1], Received[2], Received[3]);
		max = ((((float) odczyt / 180) * 90) + 30);
		l = TIM4->CCR1;
		if (max == l)
			break;
		if (max > l)
			flaga = 4;
		else
			flaga = 5;
		HAL_TIM_Base_Start_IT(&htim2);
		break;
	case 'd':
		odczyt = atoi((char*) tmp);
		size = sprintf((char*) Data, "Odebrano znak: %c%c%c%c\n\r", Received[0],
				Received[1], Received[2], Received[3]);
		max = odczyt + 25;
		TIM3->CCR2 = max;
		break;
	case 'e':
		//TIM5->CNT = 3875;
		odczyt = atoi((char*) tmp);
		size = sprintf((char*) Data, "Odebrano znak: %c%c%c%c\n\r", Received[0],
				Received[1], Received[2], Received[3]);
		positions = (odczyt * 32) + 1405;
		if (positions < TIM5->CNT) {
			kierunek(0, 1, 1, 0); //obrot w prawo
			obrot = 1;
		} else {
			kierunek(1, 0, 0, 1); //obrot w lewo
			obrot = -1;
		}
		TIM1->CCR2 = 60;
		TIM1->CCR4 = 40;
		HAL_TIM_Base_Start_IT(&htim9);
		break;
	case 'f': //jazda do przodu
		kierunek(1, 0, 1, 0);
		TIM1->CCR2 = 100;
		TIM1->CCR4 = 70;
		break;
	case 'g': //zatrzymanie
		kierunek(1, 1, 1, 1);
		TIM5->CNT = 7000;
		break;
	case 'V':
		HAL_UART_Transmit_IT(&huart6, Data, size);
		HAL_UART_Receive_IT(&huart6, (uint8_t*) &Received, 5);
		return;
		break;
	case 'x':
		odczyt = atoi((char*) &Received[1]);
		target.x = odczyt;
		HAL_UART_Transmit_IT(&huart6, Data, size);
		HAL_UART_Receive_IT(&huart6, (uint8_t*) &Received, 5);
		return;
		break;
	case 'y':
		odczyt = atoi((char*) &Received[1]);
		target.y = odczyt;
		HAL_UART_Transmit_IT(&huart6, Data, size);
		HAL_UART_Receive_IT(&huart6, (uint8_t*) &Received, 5);
		return;
		break;
	case 'z':
		odczyt = atoi((char*) &Received[1]);
		target.z = odczyt;
		goFABRIK = 1;
		break;
	case 'h':
		max1 = 120;
		i = TIM4->CCR4;
		j = TIM4->CCR3;
		if (max1 == i) {
			//nic nie rob
		} else if (max1 > i)
			flagaF1 = 1;
		else
			flagaF1 = -1;

		max2 = 98;
		if (max2 > 98)
			max2 = 98;
		if (max2 < 25)
			max2 = 25;
		k = TIM4->CCR2;
		if (max2 == k) {
			//nic nie rob
		}
		if (max2 > k)
			flagaF2 = 1;
		else
			flagaF2 = -1;
		flaga = 10;
		HAL_TIM_Base_Start_IT(&htim2);
		break;
	}

	HAL_UART_Transmit_IT(&huart6, Data, size); // Rozpoczecie nadawania danych z wykorzystaniem przerwan
	HAL_UART_Receive_IT(&huart6, (uint8_t*) &Received, 4); // Ponowne włączenie nasłuchiwania
}

int distanceBetween(struct point start, struct point end) {
	return sqrt(
			pow(end.x - start.x, 2) + pow(end.y - start.y, 2)
					+ pow(end.z - start.z, 2));
}

void fabrik(struct point pStart, struct point pMiddle, struct point pEnd,
		struct point target) {
	float distance = distanceBetween(pStart,target);
	float diffrence;
	float rSM, rME;
	float lambdaME, lambdaSM;
	float alfa, beta, gamma;
	struct point B = pStart;
	if (distance > L1 + L2) {
		//punkt poza obszarem roboczym
	} else {
		diffrence = distanceBetween(pStart, pEnd);
		while (diffrence > TOLERANCE) {

			pEnd = target;
			rME = distanceBetween(pEnd, pMiddle);
			lambdaME = L2 / rME;
			pMiddle.x = (1 - lambdaME) * pEnd.x + lambdaME * pMiddle.x;
			pMiddle.y = (1 - lambdaME) * pEnd.y + lambdaME * pMiddle.y;
			pMiddle.z = (1 - lambdaME) * pEnd.z + lambdaME * pMiddle.z;

			rSM = distanceBetween(pMiddle, pStart);
			lambdaSM = L1 / rSM;
			pStart.x = (1 - lambdaSM) * pMiddle.x + lambdaSM * pStart.x;
			pStart.y = (1 - lambdaSM) * pMiddle.y + lambdaSM * pStart.y;
			pStart.z = (1 - lambdaSM) * pMiddle.z + lambdaSM * pStart.z;

			pStart = B;
			rSM = distanceBetween(pMiddle, pStart);
			lambdaSM = L1 / rSM;
			pMiddle.x = (1 - lambdaSM) * pStart.x + lambdaSM * pMiddle.x;
			pMiddle.y = (1 - lambdaSM) * pStart.y + lambdaSM * pMiddle.y;
			pMiddle.z = (1 - lambdaSM) * pStart.z + lambdaSM * pMiddle.z;

			rME = distanceBetween(pEnd, pMiddle);
			lambdaME = L2 / rME;
			pEnd.x = (1 - lambdaME) * pMiddle.x + lambdaME * pEnd.x;
			pEnd.y = (1 - lambdaME) * pMiddle.y + lambdaME * pEnd.y;
			pEnd.z = (1 - lambdaME) * pMiddle.z + lambdaME * pEnd.z;

			diffrence = distanceBetween(target, pEnd);
		}
		alfa = atanf(pEnd.y / pEnd.x) * 180 / M_PI;
		beta = acosf(pMiddle.x / (cosf(alfa * M_PI / 180.0) * L1)) * 180 / M_PI;//1
		alfa = (int)alfa;
		beta = (int)beta;
		pEnd.x = (int)pEnd.x;
		gamma = asinf((pEnd.x / cosf(alfa * M_PI / 180.0)- cosf(beta * M_PI / 180.0) * L1) / L2) * 180 / M_PI- beta + 90.0;//1
		if(isnanf(gamma)){
			return;
		}
		max1 = ((((float) beta / 180) * 90) + 30);
		i = TIM4->CCR4;
		j = TIM4->CCR3;
		if (max1 == i) {
			//nic nie rob
		} else if (max1 > i)
			flagaF1 = 1;
		else
			flagaF1 = -1;

		max2 = (((-0.53) * (float) gamma) + (119.3));
		if (max2 > 98)
			max2 = 98;
		if (max2 < 25)
			max2 = 25;
		k = TIM4->CCR2;
		if (max2 == k) {
			//nic nie rob
		}
		if (max2 > k)
			flagaF2 = 1;
		else
			flagaF2 = -1;

		if(pEnd.x>0)
			positions = ((180-alfa) * 32) + 1405;
		if(pEnd.x<=0 && pEnd.y>=0)
			positions = ((360-alfa) * 32) + 1405;
		if(pEnd.x<0 && pEnd.y<0)
			positions = ((0+alfa) * 32) + 1405;
		if (positions < TIM5->CNT) {
			kierunek(0, 1, 1, 0); //obrot w prawo
			obrot = 2;
		} else {
			kierunek(1, 0, 0, 1); //obrot w lewo
			obrot = -2;
		}
		TIM1->CCR2 = 60;
		TIM1->CCR4 = 40;
		HAL_TIM_Base_Start_IT(&htim9);
	}
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_TIM4_Init();
	MX_TIM2_Init();
	MX_USART6_UART_Init();
	MX_TIM1_Init();
	MX_TIM3_Init();
	MX_TIM5_Init();
	MX_TIM9_Init();
	/* USER CODE BEGIN 2 */
	//HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL); //enkoder z 1 silnika
	TIM5->CNT = 7000; //pozycja poczatkowa podstawy odpowiadajaca 180stopni
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); // ten sam przy podstawie
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); // ten sam przy podstawie
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2); // srodkowy przegub
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); // obrot efektorem
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // chwytak
	/*
	 * KONFIGURACJA POCZATKOWA MANIPULATORA 120 - 41 - 98 - 75
	 */
	TIM4->CCR4 = 120; //(75 srodek) inkrementacja ruch w przod
	TIM4->CCR3 = 41; //(80 na srodku) inkremetacja odchyla w tyl
	TIM4->CCR2 = 98; //25-125 inkrementacja zamyka (66 kat prosty) (25 wyprostowane) (98 zamkniety)
	TIM4->CCR1 = 75; //25-125 inkrementacja skreca w lewo (75 srodek) (125 max w prawo) (25 max w lewo ale lekko wychodzi poza zakres)
	kierunek(1, 0, 1, 0); // jazda na wprost
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // PRAWY (2)  100 dobra predkosc
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // LEWY (1)    70 dobra predkosc
	TIM1->CCR2 = 0;
	TIM1->CCR4 = 0;
	HAL_UART_Receive_IT(&huart6, (uint8_t*) &Received, 4);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
//		if(workspace==1){
//			workspace=0;
//			pStart.x=0;
//			pStart.y=0;
//			pStart.z=BAZA;
//			pMiddle.x = cosf(ALFA * M_PI / 180.0) * cosf(BETA * M_PI / 180.0) * L1;
//			pMiddle.y = sinf(ALFA * M_PI / 180.0) * cosf(BETA * M_PI / 180.0) * L1;
//			pMiddle.z = BAZA + cosf((90.0 - BETA) * M_PI / 180.0) * L1;
//			pEnd.x = cosf(ALFA * M_PI / 180.0) * (cosf(BETA * M_PI / 180.0) * L1 + sinf((BETA + GAMMA - 90.0) * M_PI / 180.0) * L2);
//			pEnd.y = sinf(ALFA * M_PI / 180.0) * (cosf(BETA * M_PI / 180.0) * L1 + sinf((BETA + GAMMA - 90.0) * M_PI / 180.0) * L2);
//			pEnd.z = BAZA + cosf((90.0 - BETA) * M_PI / 180.0) * L1 - cosf((BETA + GAMMA - 90.0) * M_PI / 180.0) * L2;
//		}
		if(goFABRIK==1){
			goFABRIK=0;
			fabrik(pStart, pMiddle, pEnd, target);
		}
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 100;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 9999;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 99;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 3999;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 999;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 1999;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 999;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 1999;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 999;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */
	HAL_TIM_MspPostInit(&htim4);

}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void) {

	/* USER CODE BEGIN TIM5_Init 0 */

	/* USER CODE END TIM5_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 0;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 14000;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 15;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 15;
	if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM5_Init 2 */

	/* USER CODE END TIM5_Init 2 */

}

/**
 * @brief TIM9 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM9_Init(void) {

	/* USER CODE BEGIN TIM9_Init 0 */

	/* USER CODE END TIM9_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };

	/* USER CODE BEGIN TIM9_Init 1 */

	/* USER CODE END TIM9_Init 1 */
	htim9.Instance = TIM9;
	htim9.Init.Prescaler = 499;
	htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim9.Init.Period = 99;
	htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim9) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM9_Init 2 */

	/* USER CODE END TIM9_Init 2 */

}

/**
 * @brief USART6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART6_UART_Init(void) {

	/* USER CODE BEGIN USART6_Init 0 */

	/* USER CODE END USART6_Init 0 */

	/* USER CODE BEGIN USART6_Init 1 */

	/* USER CODE END USART6_Init 1 */
	huart6.Instance = USART6;
	huart6.Init.BaudRate = 115200;
	huart6.Init.WordLength = UART_WORDLENGTH_8B;
	huart6.Init.StopBits = UART_STOPBITS_1;
	huart6.Init.Parity = UART_PARITY_NONE;
	huart6.Init.Mode = UART_MODE_TX_RX;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart6) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART6_Init 2 */

	/* USER CODE END USART6_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, kierunkiP1_Pin | kierunkiP2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(kierunkiL1_GPIO_Port, kierunkiL1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(kierunkiL2_GPIO_Port, kierunkiL2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : kierunkiP1_Pin kierunkiP2_Pin */
	GPIO_InitStruct.Pin = kierunkiP1_Pin | kierunkiP2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : kierunkiL1_Pin */
	GPIO_InitStruct.Pin = kierunkiL1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(kierunkiL1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : kierunkiL2_Pin */
	GPIO_InitStruct.Pin = kierunkiL2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(kierunkiL2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
