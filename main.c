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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "string.h" //-->>BIBLIOTECA PARA MANIPULAR STRINGS
#include "i2c-lcd.h" //-->>PARA MANIPULAR DISPLAYS VIA I2C
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// -->>DEFINES PARA VERIFICAÇÃO DE ESTADO
#define ESPERANDO_INTERRUPCAO_1   0
#define ESPERANDO_INTERRUPCAO_2   1
#define FALSO  0
#define VERDADEIRO 1

//-->>DEFINES PARA O MÓDULO MPU
#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

FATFS fs;  //-->>objeto com estrutura do tipo fatfs
FIL fil;	//-->>objeto com estrutura do tipo filer (arquivo)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

//--> VARIAVEIS PARA O ACELERÔMETRO DO MÓDULO MPU
int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;
float Ax, Ay, Az;

//-->VARIAVEIS DE ESTADO PARA VERIFICAÇÃO
volatile uint8_t exibir_mpu = FALSO;
volatile uint8_t inicializado = FALSO;
volatile uint8_t gravacao_inicializada = FALSO;
volatile uint8_t gravar_display = FALSO;
volatile uint8_t gravar_sd = FALSO;
volatile uint8_t mudar_arquivo = FALSO;
volatile uint8_t ler_mpu = FALSO;
volatile uint8_t ler_temp = FALSO;
volatile uint8_t enviar_para_esp8266 = FALSO;

GPIO_PinState estado_botao;
GPIO_PinState estado_anterior_botao = GPIO_PIN_RESET;


//-->>VARIAVEIS PARA O SENSOR IDUTIVO
volatile uint8_t estado_sensor_indutivo = ESPERANDO_INTERRUPCAO_1;
volatile uint32_t T1_IND = 0;
volatile uint32_t T2_IND = 0;
volatile uint32_t TICKS_IND = 0;
volatile uint32_t TIM2_C1_OVC = 0;
volatile uint32_t FREQUENCIA_IND = 0;

//-->> VARIAVEIS PARA O SINAL DE VELA
volatile uint8_t estado_sinal_de_vela = ESPERANDO_INTERRUPCAO_1;
volatile uint32_t T1_VELA = 0;
volatile uint32_t T2_VELA = 0;
volatile uint32_t TICKS_VELA = 0;
volatile uint32_t TIM2_C3_OVC = 0;
volatile uint32_t FREQUENCIA_VELA = 0;

volatile uint16_t TEMPERATURA = 0;

//-->> VARI�?VEIS PARA A VELOCIDADE
volatile float VELOCIDADE = 0;
volatile float VELOCIDADE_MAX = 0;

volatile uint8_t indice_arquivo = 1;

//-->> VARI�?VEIS PARA CONTADORES
int i;
volatile uint8_t contagem_vetor = 0;
volatile uint8_t contagem_sd = 0;
volatile uint8_t contagem_display = 1; //iniciar em 1 (1*20 = 20ms adiantado)
volatile uint8_t contagem_botao = 0;
volatile uint16_t contagem_esp8266 = 2; //iniciar em 2 (2*20 = 40ms adiantado)
volatile uint8_t tempo_pressionado = 0;
volatile uint8_t contagem_mpu = 3; //iniciar em 3 (3*20 = 60ms adiantado)
volatile uint8_t contagem_temp = 4; //iniciar em 4 (4*20 = 80ms adiantado)
volatile uint32_t tempo_entre_interrupcoes_ind = 0;
volatile uint32_t tempo_entre_interrupcoes_vela = 0;

//Os adiantamentos é para reduzir o número de tarefas simultâneas, mas mantendo os intervalos definidos

//-->> VARI�?VEIS PARA A MARCAÇÃO DO TEMPO
volatile uint16_t mseg = 0;
volatile uint8_t seg = 0;
volatile uint8_t min = 0;
volatile uint8_t horas = 0;

//-->> VARI�?VEIS PARA O DESENHO DA BARRA DE RPM
int unidades_da_barra = 0;
char simbolo_barra[2];
char barra_rpm[20];

//-->> STRING PARA NOME DO ARQUIVO SALVO
char nome_arquivo[50];

//-->> VARI�?VEIS DE STRING PARA BUFFERS
char dado_atual[50];
char dados[500] = {};
char dados_sd[500] = {};
char dados_display[21];
char uart_dados[30] = 	{};

// caracteres display
uint8_t termometro[8] = {0x06, 0x04, 0x06, 0x04, 0x06, 0x04, 0x0E, 0x0E};
uint8_t tracoEsq[8] = {0x10, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t tracoGrandeEsq[8] = {0x10, 0x10, 0x10, 0x10, 0x00, 0x00, 0x00, 0x00};
uint8_t espaco[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void MPU6050_Init(void){ //-->> FUNÇÃO PARA INICIALIZAR O MÓDULO MPU6050
	uint8_t check, Data;

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000);

	if (check == 104)  // 0x68 SERA RETORNADO PELO SENSOR SE TUDO OCORRER BEM
		{
			// power management register 0X6B, PODE-SE ESCREVER TUDO 0 PARA ATIVAR O SENSOR
			Data = 0;
			HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 1000);

			// SETA DATA RATE PARA 1KHz ESCREVENDO NO REGISTRADOR SMPLRT_DIV
			Data = 0x07;
			HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

			// SETA A CONFIGURAÇÕES DO ACELEROMTRO NO REGISTRADOR ACCEL_CONFIG Register
			// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 2g
			Data = 0x00;
			HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

			// SETA AS CONFIGURAÇÕES DO GIROSCÓPIO NO REGISTRADOR GYRO_CONFIG Register
			// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s
			Data = 0x00;
			HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
		}
}

////-->> FUNÇÃO PARA LEITURA DA ACELERAÇÃO
void MPU6050_Read_Accel (void)
{
	// -->> LÊ 6 BYTES DE DADOS DO REGISTRADOR ACCEL_XOUT_H register
	uint8_t Rec_Data[6];
	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** CONVERSÃO DOS VALORES RAW DA ACELERAÇÃO EM g
	     DIVIDI-SE DE ACORDO COM A ESCALA COMPLETA SETADA EM FS_SEL
	     SETOU-SE FS_SEL = 0, ENTÃO A DIVISÃO É POR 16384.0
	     PARA MAIS DETALHES CONFERIR O REGISTRDOR ACCEL_CONFIG ****/
	Ax = Accel_X_RAW/16384.0;
	Ay = Accel_Y_RAW/16384.0;
	Az = Accel_Z_RAW/16384.0;
}

//Função para criar characteres
void salvar_characteres(void){
	  // Salva chars no CGRAM
	  lcd_send_cmd(0x40);
	  for (int i=0; i<8; i++) lcd_send_data(termometro[i]);

	  lcd_send_cmd(0x40+8);
	  for (int i=0; i<8; i++) lcd_send_data(tracoEsq[i]);

	  lcd_send_cmd(0x40+16);
	  for (int i=0; i<8; i++) lcd_send_data(tracoGrandeEsq[i]);

	  lcd_send_cmd(0x40+24);
	  for (int i=0; i<8; i++) lcd_send_data(espaco[i]);

	  /*lcd_send_cmd(0x40+32);
	  for (int i=0; i<8; i++) lcd_send_data(cc5[i]);

	  lcd_send_cmd(0x40+40);
	  for (int i=0; i<8; i++) lcd_send_data(cc6[i]);

	  lcd_send_cmd(0x40+48);
	  for (int i=0; i<8; i++) lcd_send_data(cc7[i]);*/
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
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
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  //-->> DEFINE O SIMBOLO DA BARRA DE VELOCIDADE COMO UM QUADRADO PREENCHIDO
  sprintf(simbolo_barra,"%c",255);


  //-->> INICIALIZAÇÃO DO DISPLAY LCD DO BAJA
  lcd_init();

  salvar_characteres();

  lcd_send_cmd (0x80|0x00);
  lcd_send_string("VITORIA BAJA - 2023");

  lcd_send_cmd (0x80|0x40);
  lcd_send_string(" ");

  lcd_send_cmd (0x80|0x14);
  lcd_send_string("INICIALIZANDO...");

  lcd_send_cmd (0x80|0x54);
  lcd_send_string(" ");

  //-->> ATRASO
  HAL_Delay(2000);

  //-->> INICIALIZAÇÃO DO MÓDULO MPU6050
  MPU6050_Init ();

  //-->MONTAGEM DO CARTÃO SD
  f_mount(&fs, "", 0);

  //-->>CRIAÇÃO/ABERTURA DO ARQUIVO QUE RECEBERAO OS DADOS
  f_open(&fil, "dados_placa_central_1.txt", FA_OPEN_ALWAYS | FA_WRITE | FA_READ);
  f_lseek(&fil, fil.fsize);
  f_puts("Inicializando Gravação de Dados...\n\nTEMPO\t-->\tVELOCIDADE\tRPM_VELA\tTEMPERATURA\tAx\tAy\tAz\n\n", &fil);
  f_close(&fil);

  //-->>VERIFICAR SE ESTA VARI�?VEL AINDA É NECESS�?RIA
  inicializado = VERDADEIRO;

  //-->> LIMPAR DISPLAY
  lcd_clear();

  //-->> INICIALIZAÇÃO DOS TIM2 E SEUS IMPUT CAPTURES
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);

  //-->>INICIALIZAÇÃO DO TIM3
  HAL_TIM_Base_Start_IT(&htim3);

  //-->>INICIALIZAÇÃO DA CALIBRAÇÃO ADC
  HAL_ADCEx_Calibration_Start(&hadc1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  	if(ler_temp == VERDADEIRO){
	  		HAL_ADC_Start(&hadc1);
	  		HAL_ADC_PollForConversion(&hadc1, 1);
	  		TEMPERATURA = HAL_ADC_GetValue(&hadc1)*0.0807;
	  		ler_temp = FALSO;
	  	}

		//-->> GRAVAÇÃO DOS DADOS SALVOS NO BUFFER NO CARTÃO SD
		if(gravar_sd == VERDADEIRO && gravacao_inicializada == VERDADEIRO){ 	//só executa se gravação foi inicializada, a após cada intervalo de 100ms
		  sprintf(nome_arquivo, "dados_placa_central_%i.txt",indice_arquivo);
		  f_open(&fil, nome_arquivo, FA_OPEN_ALWAYS | FA_WRITE | FA_READ);
		  f_lseek(&fil, fil.fsize);
		  f_puts(dados_sd, &fil);
		  f_close(&fil);
		  gravar_sd = FALSO;

		  if(mudar_arquivo == VERDADEIRO){ // muda o indice para salvar nova gravação em novo arquivo
			  indice_arquivo++;
			  mudar_arquivo = FALSO;
		  }
		}

		//-->>ENVIA DADOS DO BUFFER PARA ESP8266 VIA COMUNICAÇÃO UART
		if(enviar_para_esp8266 == VERDADEIRO){
	    	sprintf(uart_dados, "%.2f,%lu,%i,%.2f,%.2f,%.2f",VELOCIDADE,FREQUENCIA_VELA,TEMPERATURA,Ax,Ay,Az); //Copia dado atual para telemetria
			HAL_UART_Transmit(&huart1, (uint8_t *)uart_dados, sizeof(uart_dados), 200);
			enviar_para_esp8266 = FALSO;
		}

		//-->> ENVIA STRINGS COM DADOS PARA O DISPLAY
		if(gravar_display == VERDADEIRO){
	    	unidades_da_barra = FREQUENCIA_VELA/220; //atualiza o tamanho da barra de velocidade
			if (unidades_da_barra > 20){ //Não pode passar de 20 barras
				unidades_da_barra = 20;
			}

			lcd_clear();
			if(exibir_mpu == VERDADEIRO){ // Exibir dados do mpu6050

				sprintf(dados_display, "  Ax     Ay     Az ");

				lcd_send_cmd (0x80|0x00);
				lcd_send_string(dados_display);

				sprintf(dados_display, "%.2f  %.2f  %.2f ", Ax,Ay,Az);
				lcd_send_cmd (0x80|0x40);
				lcd_send_string(dados_display);
			}

			else{ //Exibe os dados principais

				//Printa velocidade
				sprintf(dados_display, "%.2f", VELOCIDADE);
				lcd_send_cmd (0x80|0x1F);
				lcd_send_string(dados_display);
				lcd_send_cmd (0x80|0x24);
				lcd_send_string("Km/h");

				//printa RPM do motor
				sprintf(dados_display, "%lu", FREQUENCIA_VELA);
				lcd_send_cmd (0x80|0x50);
				lcd_send_string(dados_display);

				//Printa escala do RPM
				lcd_send_cmd (0x80|0x41);
				lcd_send_data(3);
				lcd_send_cmd (0x80|0x42);
				lcd_send_data(1);
				lcd_send_cmd (0x80|0x43);
				lcd_send_data(3);
				lcd_send_cmd (0x80|0x44);
				lcd_send_data(2);

				lcd_send_cmd (0x80|0x45);
				lcd_send_data(3);
				lcd_send_cmd (0x80|0x46);
				lcd_send_data(1);
				lcd_send_cmd (0x80|0x47);
				lcd_send_data(3);
				lcd_send_cmd (0x80|0x48);
				lcd_send_data(2);

				lcd_send_cmd (0x80|0x49);
				lcd_send_data(3);
				lcd_send_cmd (0x80|0x4A);
				lcd_send_data(1);
				lcd_send_cmd (0x80|0x4B);
				lcd_send_data(3);
				lcd_send_cmd (0x80|0x4C);
				lcd_send_data(2);

				lcd_send_cmd (0x80|0x4D);
				lcd_send_data(3);
				lcd_send_cmd (0x80|0x4E);
				lcd_send_data(1);
				lcd_send_cmd (0x80|0x4F);
				lcd_send_data(3);


				//Printa o tacometro em barra de rpm
				i = 0;
				while(i<unidades_da_barra){
					strcat(barra_rpm,simbolo_barra);
					i++;
				}
				lcd_send_cmd (0x80|0x00);
				lcd_send_string(barra_rpm);
				memset(barra_rpm, 0, sizeof(barra_rpm));
				unidades_da_barra = 0;

				//Printa a temperatura
				sprintf(dados_display, "%d%cC",TEMPERATURA,223);
				lcd_send_cmd (0x80|0x14);  //printa símbolo do termômetro
				lcd_send_data(0);
				lcd_send_cmd (0x80|0x15); //printa a temperatura
				lcd_send_string(dados_display);

			}

			//Printa o cronômetro
			if(gravacao_inicializada == VERDADEIRO){
				sprintf(dados_display, "%i:%i:%i:%i",horas,min,seg,mseg);
				lcd_send_cmd (0x80|0x54);
				lcd_send_string(dados_display);
				sprintf(dados_display, "REC%i",indice_arquivo);
				lcd_send_cmd (0x80|0x64);
				lcd_send_string(dados_display);
			}
			else if (gravacao_inicializada == FALSO){
				sprintf(dados_display, "%i:%i:%i:%i", horas,min,seg,mseg);
				lcd_send_cmd (0x80|0x54);
				lcd_send_string(dados_display);
			}

			gravar_display = FALSO;
		}

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 39;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 35999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
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
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 35999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//-->>INTERRUPÇÕES DO SENSOR INDUTIVO E DO SINAL DE VELA POR IMPUT CAPTURE
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim)
{
	if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) { 	//Se houver subida de nivel no sinal do indutivo
		tempo_entre_interrupcoes_ind = 0; //Zera o contagem de tempo entre interrupções do sensor indutivo

		if(estado_sensor_indutivo == ESPERANDO_INTERRUPCAO_1) //se primeira borda de subida
		{
			T1_IND = TIM2->CCR1; //registrar tempo (contador) da primeira interrupção
			TIM2_C1_OVC = 0; //reseta os over clocks contados
			estado_sensor_indutivo = ESPERANDO_INTERRUPCAO_2; //Agora vai esperar a segunda interrupção

		}
		else if(estado_sensor_indutivo == ESPERANDO_INTERRUPCAO_2) //Segunda interrupção
		{
			T2_IND = TIM2->CCR1; //registro do tempo(contador) da segunda interrupção
			FREQUENCIA_IND = (uint32_t)(6000000/((T2_IND + (TIM2_C1_OVC * 36000)) - T1_IND)); //RPM = (1,8MHZ * (1/18 rasgos) * 60 min) /  (T2 + (TIM2_OVC * 36000)) - T1)
			VELOCIDADE = FREQUENCIA_IND * 0.106; //Calculo da velocidade em Km/h, equação = (F*(1/60)*2*PI*R*3.6)    2R = 0,561m
			estado_sensor_indutivo = ESPERANDO_INTERRUPCAO_1; //Reiniciar espera de interrupções
		}
	}

	else if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) { //se houver borda de subida no sinal de vela
		tempo_entre_interrupcoes_vela = 0; //Zera contagem de tempo entre interrupções do sinal de vela

		if(estado_sinal_de_vela == ESPERANDO_INTERRUPCAO_1)// se primeira interrupção
		{
			T1_VELA = TIM2->CCR3; //registrar o tempo(contador) da primeira interrupção
			TIM2_C3_OVC = 0; //zerar os over clocks
			estado_sinal_de_vela = ESPERANDO_INTERRUPCAO_2; //aguardar segunda interrupção
		}
		else if(estado_sinal_de_vela == ESPERANDO_INTERRUPCAO_2)//se segunda interrupção
		{
			T2_VELA = TIM2->CCR3; //registrar tempo(contador) da segunda interrupção
			FREQUENCIA_VELA = (uint32_t)(108000000/((T2_VELA + (TIM2_C3_OVC * 36000)) - T1_VELA)); //RPM = (1,8MHZ*60min)  / [(T2 + (TIM3_OVC * 36000)) - T1)]
			estado_sinal_de_vela = ESPERANDO_INTERRUPCAO_1; //Reiniciar espera de interrupções
		}

	}

}

//-->>INTERRUPÇÕES PERIÓDICAS POR OVERFLOW DOS TIMERS
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{

	if (htim->Instance == TIM2 && inicializado == VERDADEIRO) { // a cada 20 ms
		//Contadores para regular eventos:
	    TIM2_C1_OVC++;
		TIM2_C3_OVC++;
	    contagem_vetor++;
	    contagem_display++;
	    contagem_botao++;
	    contagem_mpu++;
	    contagem_esp8266++;
	    contagem_temp++;

	    //String com dados atualizados a cada 20ms (para gravar no cartão sd)
	    sprintf(dado_atual, "%i:%i:%i:%i\t-->\t%.2f\t%lu\t%i\t%.2f\t%.2f\t%.2f\n",horas,min,seg,mseg, VELOCIDADE,FREQUENCIA_VELA,TEMPERATURA,Ax,Ay,Az); // @suppress("Float formatting support")

	    //Se a gravação foi inicializada (botão acionado)
	    if(gravacao_inicializada == VERDADEIRO){
		    contagem_sd++; //contador a cada 20ms
			strcat(dados,dado_atual); //Concatena o dado atual em um buffer

		    if(contagem_sd == 10){ // a cada 200ms (juntou 10 linhas de dados)
				sprintf(dados_sd,dados); // Cópia do buffer
				gravar_sd = VERDADEIRO; // Liberar gravação no cartão sd
				memset(dados, 0, sizeof(dados)); //Limpa o buffer original
		    	contagem_sd = 0; //reseta a contagem de tempo entre gravações
		    }
	    }

	    if(contagem_esp8266 == 50){ //a cada 1s

	    	enviar_para_esp8266 = VERDADEIRO; //Libera o envio para o esp
	    	contagem_esp8266 = 0; //reseta a contagem

	    }

	    if(contagem_temp == 25){ // a cada 500ms
		  ler_temp = VERDADEIRO; // Leitura da temperaturra
	      contagem_temp = 0; //reseta a contagem
	    }

	    if(contagem_mpu == 15){ // a cada 300ms
		  MPU6050_Read_Accel(); // Leitura do acelerômetro
	      contagem_mpu = 0; //reseta a contagem
	    }

	    if(contagem_botao == 10){ //a cada 200ms
	    	estado_botao = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5); //Salva estado do botão
	    	if(estado_botao == GPIO_PIN_SET){ //Se o botão está pressionado
				tempo_pressionado++; //Conta o tempo em que permanece pressionado
			}

			if(estado_anterior_botao != estado_botao){ //Se o estado do botão mudou
				estado_anterior_botao = estado_botao; //cópia do estado atual

				if(estado_botao == GPIO_PIN_RESET){ //Se o botão foi liberado

					if(tempo_pressionado >= 2 && tempo_pressionado <= 10 ){ // se pressionado por menos que 2s
						if(gravacao_inicializada == FALSO){ //Se a gravação está desativada
							gravacao_inicializada = VERDADEIRO;  //Gravação é ativada
							HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
						}
						else if(gravacao_inicializada == VERDADEIRO){ //Se gravação está ativada
							gravacao_inicializada = FALSO; //gravação é desativada
							mudar_arquivo = VERDADEIRO; //Para mudar o arquivo na próxima gravação
							HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
						}
						//Reseta o cronômetro
						mseg = 0;
						seg = 0;
						min = 0;
						horas = 0;
					}

					else if(tempo_pressionado >= 20){ //pressionado por 4s
						exibir_mpu = !exibir_mpu; //Exibe no display dados do acelerômetro
					}

					tempo_pressionado = 0;


				}
			}

	    	contagem_botao = 0;
	    }

	    if(contagem_display == 20){ //a cada 400ms

	    	gravar_display = VERDADEIRO; //Libera gravação no display
	  		contagem_display = 0;
	    }
	}

	else if(htim->Instance == TIM3 && inicializado == VERDADEIRO){ // a cada 5ms
			//Inicia cronômetro
			mseg = mseg + 5;
			if(mseg == 1000){
				seg++;
				mseg = 0;
			}
			if(seg == 60){
				min++;
				seg = 0;
			}
			if(min == 60){
				horas++;
				min = 0;
			}

		//Conta o tempo em milissegundos entre a interrupções
		tempo_entre_interrupcoes_ind= tempo_entre_interrupcoes_ind + 5;
		tempo_entre_interrupcoes_vela = tempo_entre_interrupcoes_vela + 5;

		//Se passa de 0.5 segundo, zera as velocidades
		if(tempo_entre_interrupcoes_ind >= 500){
			tempo_entre_interrupcoes_ind = 500;
			FREQUENCIA_IND = 0;
			VELOCIDADE = 0;
		}
		if(tempo_entre_interrupcoes_vela >= 500){
			tempo_entre_interrupcoes_vela = 500;
			FREQUENCIA_VELA = 0;
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
