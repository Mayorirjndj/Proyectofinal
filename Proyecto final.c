#include "main.h"
#include "i2c-lcd.h"
#include "ULTRASONICO.h"
#include "keypad_4x4.h"
#include "stdio.h"
#include "string.h"

#define TRUE 1
#define FALSE 0
#define TIEMPO_CONTADOR 1200
#define CLAVE	"1234"

#define ESTADO_INIT         0
#define ESTADO_ABRIENDO     1
#define ESTADO_CERRANDO     2
#define ESTADO_CERRADO      3
#define ESTADO_ABIERTO      4
#define ESTADO_EMERGENCIA   5
#define ESTADO_ERROR        6
#define ESTADO_ESPERA       7

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

//DEFINICION DE LOS ESTADOS A TRAVE
volatile int ESTADO_ACTUAL = ESTADO_INIT;
volatile int ESTADO_SIGUIENTE = ESTADO_INIT;
volatile int ESTADO_ANTERIOR = ESTADO_INIT;
volatile int CONTADOR = 0;
volatile int DISTANCIA = 0;


char clave[5];
char key;

volatile struct INPUTS{
    unsigned int LSA: 1;
    unsigned int LSC: 1;
    unsigned int CA:  1;
    unsigned int CC:  1;
    unsigned int FC:  1;
}inputs;
volatile struct OUTPUTS{
    unsigned int MC: 1;
    unsigned int MA: 1;
    unsigned int LED_ERROR: 	 1;
    unsigned int LED_EMERGENCIA: 1;
    unsigned int LED_MOVIMIENTO: 1;
}outputs;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM2_Init(void);
static void LCD_PRUEBA(void);

int Func_ESTADO_INIT(void);
int Func_ESTADO_ABRIENDO(void);
int Func_ESTADO_CERRANDO(void);
int Func_ESTADO_CERRADO(void);
int Func_ESTADO_ABIERTO(void);
int Func_ESTADO_EMERGENCIA(void);
int Func_ESTADO_ERROR(void);
int Func_ESTADO_ESPERA(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM15_Init();
  MX_TIM2_Init();
  lcd_init();
  ultrasonic_init();

  LCD_PRUEBA();
  lcd_clear();
  HAL_TIM_Base_Start_IT(&htim2);


  while (1)
  {


	  	  	  if (ESTADO_SIGUIENTE == ESTADO_INIT)
	          {
	              ESTADO_SIGUIENTE = Func_ESTADO_INIT();
	          }
	          if (ESTADO_SIGUIENTE == ESTADO_ESPERA)
	          {
	              ESTADO_SIGUIENTE = Func_ESTADO_ESPERA();
	          }
	          if (ESTADO_SIGUIENTE == ESTADO_ABRIENDO)
	          {
	              ESTADO_SIGUIENTE = Func_ESTADO_ABRIENDO();
	          }
	          if (ESTADO_SIGUIENTE == ESTADO_CERRANDO)
	          {
	              ESTADO_SIGUIENTE = Func_ESTADO_CERRANDO();
	          }
	          if (ESTADO_SIGUIENTE == ESTADO_CERRADO)
	          {
	              ESTADO_SIGUIENTE = Func_ESTADO_CERRADO();
	          }
	          if (ESTADO_SIGUIENTE == ESTADO_ABIERTO)
	          {
	              ESTADO_SIGUIENTE = Func_ESTADO_ABIERTO();
	          }
	          if (ESTADO_SIGUIENTE == ESTADO_EMERGENCIA)
	          {
	              ESTADO_SIGUIENTE = Func_ESTADO_EMERGENCIA();
	          }
	          if (ESTADO_SIGUIENTE == ESTADO_ERROR)
	          {
	              ESTADO_SIGUIENTE = Func_ESTADO_ERROR();
	          }

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM15;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim15ClockSelection = RCC_TIM15CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}


static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }

}

static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

static void MX_TIM15_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 71;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65535;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, R4_Pin|R3_Pin|TRIG_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOB, LERROR_Pin|LE_Pin|LD2_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOC, R2_Pin|R1_Pin|LM_Pin|MA_Pin
                          |MC_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = C4_Pin|C3_Pin|C2_Pin|C1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = R4_Pin|R3_Pin|TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LERROR_Pin LE_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LERROR_Pin|LE_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CA_Pin CC_Pin */
  GPIO_InitStruct.Pin = CA_Pin|CC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : R2_Pin R1_Pin LM_Pin MA_Pin
                           MC_Pin */
  GPIO_InitStruct.Pin = R2_Pin|R1_Pin|LM_Pin|MA_Pin
                          |MC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = LSA_Pin|LCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if(DISTANCIA < 10){

		inputs.FC = TRUE;
	}else{

		inputs.FC = FALSE;
	}
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
		inputs.LSA = (HAL_GPIO_ReadPin(LSA_GPIO_Port, LSA_Pin) == TRUE)?TRUE:FALSE;
		inputs.LSC = (HAL_GPIO_ReadPin(LCS_GPIO_Port, LCS_Pin) == TRUE)?TRUE:FALSE;
		inputs.CC = (HAL_GPIO_ReadPin(CC_GPIO_Port, CC_Pin) == TRUE)?TRUE:FALSE;
		inputs.CA = (HAL_GPIO_ReadPin(CA_GPIO_Port, CA_Pin) == TRUE)?TRUE:FALSE;

		if (outputs.LED_MOVIMIENTO == TRUE)
		    {
				HAL_GPIO_WritePin(LM_GPIO_Port, LM_Pin, GPIO_PIN_SET);
		    }else
		    {
		    	HAL_GPIO_WritePin(LM_GPIO_Port, LM_Pin, GPIO_PIN_RESET);
		    }

		if (outputs.LED_ERROR == TRUE)
			{
				HAL_GPIO_WritePin(LERROR_GPIO_Port, LERROR_Pin, GPIO_PIN_SET);
			}else
			{
				HAL_GPIO_WritePin(LERROR_GPIO_Port, LERROR_Pin, GPIO_PIN_RESET);
			}

		if (outputs.LED_EMERGENCIA == TRUE)
			{
				HAL_GPIO_WritePin(LE_GPIO_Port, LE_Pin, GPIO_PIN_SET);
			}else
			{
				HAL_GPIO_WritePin(LE_GPIO_Port, LE_Pin, GPIO_PIN_RESET);
			}

		if (outputs.MA == TRUE)
			{
				HAL_GPIO_WritePin(MA_GPIO_Port, MA_Pin, GPIO_PIN_SET);
			}else
			{
				HAL_GPIO_WritePin(MA_GPIO_Port, MA_Pin, GPIO_PIN_RESET);
			}
		if (outputs.MC == TRUE)
			{
				HAL_GPIO_WritePin(MC_GPIO_Port, MC_Pin, GPIO_PIN_SET);
			}else
			{
				HAL_GPIO_WritePin(MC_GPIO_Port, MC_Pin, GPIO_PIN_RESET);
			}


		switch (ESTADO_ACTUAL) {
			case ESTADO_ABRIENDO:
					CONTADOR++;
			break;
			case ESTADO_CERRANDO:
					CONTADOR++;
			break;
			default:
					CONTADOR = 0;
			break;
				}

}

int Func_ESTADO_INIT(){

	ESTADO_ANTERIOR = ESTADO_ACTUAL;
	ESTADO_ACTUAL = ESTADO_INIT;

	outputs.LED_MOVIMIENTO = FALSE;
	outputs.LED_EMERGENCIA = FALSE;
	outputs.MA = FALSE;
	outputs.MC = FALSE;
	outputs.LED_ERROR = FALSE;

	lcd_clear();
	lcd_enviar("INICIANDO", 0, 0);
	lcd_enviar("PROGRAMA", 1, 0);
	HAL_Delay(500);

	return ESTADO_ESPERA;

}

int Func_ESTADO_ABRIENDO(){

	ESTADO_ANTERIOR = ESTADO_ACTUAL;
	ESTADO_ACTUAL = ESTADO_ABRIENDO;

	outputs.LED_MOVIMIENTO = TRUE;
	outputs.LED_EMERGENCIA = FALSE;
	outputs.MA = TRUE;
	outputs.MC = FALSE;
	outputs.LED_ERROR = FALSE;



	lcd_clear();
	lcd_enviar("ABRIENDO", 0, 0);
	lcd_enviar("PUERTON", 1, 0);
	HAL_Delay(500);

	for (;;)
	    {
			DISTANCIA = ultrasonic_measure_distance();
	        if(inputs.LSA == TRUE){

	            return ESTADO_ABIERTO;
	        }
	        if (inputs.LSA == TRUE && inputs.LSC == TRUE)
	        {
	            return ESTADO_ERROR;
	        }
	        if (inputs.FC == TRUE)
	        {
	            return ESTADO_EMERGENCIA;
	        }
	        if(inputs.CC == TRUE ){

	            return ESTADO_CERRANDO;
	        }

	        if (CONTADOR == TIEMPO_CONTADOR)
	        {
	            return ESTADO_ERROR;
	        }
	    }

}

int Func_ESTADO_CERRANDO(){

	ESTADO_ANTERIOR = ESTADO_ACTUAL;
	ESTADO_ACTUAL = ESTADO_CERRANDO;

	outputs.LED_MOVIMIENTO = TRUE;
	outputs.LED_EMERGENCIA = FALSE;
	outputs.MA = FALSE;
	outputs.MC = TRUE;
	outputs.LED_ERROR = FALSE;

	lcd_clear();
	lcd_enviar("CERRANDO", 0, 0);
	lcd_enviar("PUERTON", 1, 0);
	HAL_Delay(500);

    for (;;)
    {
    	DISTANCIA = ultrasonic_measure_distance();
        if (inputs.LSC == TRUE )
        {
            return ESTADO_CERRADO;
        }

        if (inputs.LSA == TRUE && inputs.LSC == TRUE)
        {
            return ESTADO_ERROR;
        }
        if (inputs.FC == TRUE)
        {
            return ESTADO_EMERGENCIA;
        }
        if (inputs.CA == TRUE)
        {
            return ESTADO_ABRIENDO;
        }
        if (CONTADOR == 3600)
        {
            return ESTADO_ERROR;
        }


    }
}

int Func_ESTADO_CERRADO()
{
	ESTADO_ANTERIOR = ESTADO_ACTUAL;
	ESTADO_ACTUAL = ESTADO_CERRADO;

	outputs.LED_MOVIMIENTO = FALSE;
	outputs.LED_EMERGENCIA = FALSE;
	outputs.MA = FALSE;
	outputs.MC = FALSE;
	outputs.LED_ERROR = FALSE;

	lcd_clear();
	lcd_enviar("PUERTON", 0, 0);
	lcd_enviar("CERRADO", 1, 0);
	HAL_Delay(500);

	for (;;)
	{
		DISTANCIA = ultrasonic_measure_distance();
		HAL_Delay(1000);
	    return ESTADO_ESPERA;
	}


}

int Func_ESTADO_ABIERTO(){

	ESTADO_ANTERIOR = ESTADO_ACTUAL;
	ESTADO_ACTUAL = ESTADO_ABIERTO;

	outputs.LED_MOVIMIENTO = FALSE;
	outputs.LED_EMERGENCIA = FALSE;
	outputs.MA = FALSE;
	outputs.MC = FALSE;
	outputs.LED_ERROR = FALSE;

	lcd_clear();
	lcd_enviar("PUERTON", 0, 0);
	lcd_enviar("ABIERTO", 1, 0);
	HAL_Delay(500);

	for (;;)
	{
		HAL_Delay(500);
	    return ESTADO_ESPERA;
	}

}

int Func_ESTADO_EMERGENCIA(){

	ESTADO_ANTERIOR = ESTADO_ACTUAL;
	ESTADO_ACTUAL = ESTADO_EMERGENCIA;

	outputs.LED_MOVIMIENTO = FALSE;
	outputs.LED_EMERGENCIA = TRUE;
	outputs.MA = FALSE;
	outputs.MC = FALSE;
	outputs.LED_ERROR = FALSE;

	lcd_clear();
	lcd_enviar("EMERGENCIA!!", 0, 0);
	lcd_enviar("EMERGENCIA!!", 1, 0);
	HAL_Delay(500);

	for (;;)
	 {
		DISTANCIA = ultrasonic_measure_distance();

	    if (inputs.FC == FALSE)
	    {
	    	HAL_Delay(1500);
	    	return ESTADO_ANTERIOR;
	    }
	    HAL_Delay(400);
	    outputs.LED_EMERGENCIA = !outputs.LED_EMERGENCIA;

	 }


}

int Func_ESTADO_ERROR(){

	ESTADO_ANTERIOR = ESTADO_ACTUAL;
	ESTADO_ACTUAL = ESTADO_ERROR;

	outputs.LED_MOVIMIENTO = FALSE;
	outputs.LED_EMERGENCIA = FALSE;
	outputs.MA = FALSE;
	outputs.MC = FALSE;
	outputs.LED_ERROR = TRUE;

	int cont = 0;

	lcd_clear();
	lcd_enviar("ERROR!", 0, 0);
	lcd_enviar("SISTEMA!!", 1, 0);
	HAL_Delay(1500);
	lcd_clear();



	for(;;){
		DISTANCIA = ultrasonic_measure_distance();
		outputs.LED_ERROR = (!outputs.LED_ERROR);
		HAL_Delay(200);
		key = Keypad_Get_Char();
		lcd_enviar("CLAVE:", 1, 0);

			if(key != 0 && cont < 4){


				clave[cont] = key;
				cont++;
			}
			if(cont == 4){
				if(strcmp(clave, CLAVE) == 0){

					lcd_clear();
					lcd_enviar("CLAVE", 0, 0);
					lcd_enviar("CORRECTA", 1, 0);
					HAL_Delay(1500);

					return ESTADO_ESPERA;
				}else {

					lcd_clear();
					lcd_enviar("CLAVE", 0, 0);
					lcd_enviar("INCORRECTA", 1, 0);
					HAL_Delay(1500);
				}
			}
	}

}

int Func_ESTADO_ESPERA(){

	ESTADO_ANTERIOR = ESTADO_ACTUAL;
	ESTADO_ACTUAL = ESTADO_ESPERA;

	outputs.LED_MOVIMIENTO = FALSE;
	outputs.LED_EMERGENCIA = FALSE;
	outputs.MA = FALSE;
	outputs.MC = FALSE;
	outputs.LED_ERROR = FALSE;

	lcd_clear();
	lcd_enviar("ESPERANDO", 0, 0);
	HAL_Delay(500);

	for (;;)
	{
		DISTANCIA = ultrasonic_measure_distance();

	   if (inputs.LSA == FALSE && inputs.FC == FALSE && inputs.LSC == FALSE)
	    {
	       return ESTADO_CERRANDO;
	    }
	    if (inputs.CA == TRUE && inputs.FC == FALSE && inputs.LSA == FALSE)
	     {
	       return ESTADO_ABRIENDO;
	     }
	    if (inputs.CC == TRUE && inputs.FC == FALSE && inputs.LSC == FALSE)
	    {
	    	return ESTADO_CERRANDO;
	    }
	    if (inputs.CA == TRUE && inputs.FC == FALSE && inputs.LSA == FALSE)
	    {
	    	return ESTADO_ABRIENDO;
	    }

	    if (inputs.FC == TRUE)
	    {
	    	return ESTADO_EMERGENCIA;
	    }
	    if(inputs.LSA == TRUE && inputs.LSC == TRUE)
	    {
	    	return ESTADO_ERROR;
	    }

	    }

}




static void LCD_PRUEBA(void){
	lcd_clear();
	lcd_enviar("MAQUINA", 0, 3);
	lcd_enviar("ESTADO", 1, 3);
	HAL_Delay(2000);
	lcd_clear();
	lcd_enviar("ALBERT", 0, 0);
	lcd_enviar("MESA", 1, 0);
	HAL_Delay(2000);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }

}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif
