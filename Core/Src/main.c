/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ESP8266_Chelo.h"
#include "ModBUS_Chelo.h"
#include "STR_Chelo.h"
#include "string.h"
#include "stdio.h"
#include "http.h"
#include "RYLR896.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define TOK 1
#define FIND 0
#define SERVIDOR 1
#define CLIENTE 0
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
struct LoRa lr;
struct WIFI wf;
struct MBUS mb_lr;		// Instancia Ethernet
//struct MBUS mb_wf;		// Instancia Wi-Fi

char post[512];
char body[512];
char ENDPOINT[]="/logdata",//ENDPOINT[]="/tepelco",
     SERVER_IP[]="192.168.0.91",
     PORT[]="8000";

uint8_t decimal[17],
		error=0,
		ESP_REinit=0,			//Conteo de intentos de incializacion
		ESP_InitF=0,			//Flag de error por no encontrar la sentencia
		ESP_HW_Init=0,
		EN_UART1_TMR=0,
		EN_UART2_TMR=0,
		EN_USART1_TMR=0,
		FLAG_TIMEOUT=0,
		FLAG_UART1=0,
		FLAG_UART2=0,
		resultado=0,
		error_rxdata=0,
		esp_restart=0,
		conexion,
		WF_SND_FLAG=0;

uint16_t datos[]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

uint32_t ms_ticks=0,
		 min_ticks=0;


char	UART_RX_vect[1024],
		UART2_RX_vect[512],
		UART_RX_vect_hld[1024],
		WIFI_NET[]="PLC_DEV",//WIFI_NET[]="Fibertel WiFi967 2.4GHz",//WIFI_NET[]="PLC_DEV",//
		WIFI_PASS[]="12345678",//WIFI_PASS[]="0042880756",//WIFI_PASS[]="12345678",//
		TCP_SERVER[]="192.168.0.91",//TCP_SERVER[]="192.168.0.65",//TCP_SERVER[]="192.168.0.102",//TCP_SERVER[]="192.168.0.47",
		TCP_PORT[]="8000",//TCP_PORT[]="502",
		TCP_SERVER_LOCAL[]="192.168.0.33",//TCP_SERVER[]="192.168.0.47",
		TCP_SERVER_LOCAL_GWY[]="192.168.0.99",//TCP_SERVER[]="192.168.0.47",
		TCP_SERVER_LOCAL_MSK[]="255.255.255.0",//TCP_SERVER[]="192.168.0.47",
		TCP_PORT_LOCAL[]="502",
		CMP_VECT[]="\0",
		UART_RX_byte[2],
		UART2_RX_byte[2];

int 	wf_snd_flag_ticks=0,
		UART_RX_items=0,
		UART2_RX_items=0,
		ESP_ticks=0,
		MB_TOUT_ticks=0,
		ticks=0,
		ntestc=0,
		uart1pass=0,
		USART1_ticks=0,
		FLAG_USART1=0,
		items_rx=0,
		UART_RX_pos=0;
		UART2_RX_pos=0;

enum {
	TEPELCO,
	TEST_1,
	TEST_2,
	TEST_3,
	TEST_4,
	TEST_5,
	TEST_6,
	PM710
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
uint8_t ESP8266_HW_Init(UART_HandleTypeDef *);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	  //----------------------- LoRa ------------------------//

	  //----------------------- LoRa ------------------------//

	  //----------------------- WIFI ------------------------//
 	  	Inicializar(&wf); 									//Borra todos los registros de la estructura
 	  	wf.RESET_PORT=GPIOA;
 	  	wf.RESET_PIN=GPIO_PIN_8;
		strcpy(wf._WF_Net, WIFI_NET);						//Nombre de la red WIFI  a conectar Fibertel WiFi967 2.4GHz
		strcpy(wf._WF_Pass, WIFI_PASS);						//Password de la red WIFI
		strcpy(wf._TCP_Remote_Server_IP, TCP_SERVER);		//char _TCP_Remote_Server_IP[16];		//IP del Servidor TCP
		strcpy(wf._TCP_Remote_Server_Port, TCP_PORT);		//char _TCP_Remote_Server_Port[16];			//Puerto del Servidor TCP
		strcpy(wf._TCP_Local_Server_IP, TCP_SERVER_LOCAL);
		strcpy(wf._TCP_Local_Server_GWY, TCP_SERVER_LOCAL_GWY);
		strcpy(wf._TCP_Local_Server_MSK, TCP_SERVER_LOCAL_MSK);
		strcpy(wf._TCP_Local_Server_Port, TCP_PORT_LOCAL);
		wf._TCP_Local_Server_EN=0;							//Habilito el Servidor Local
		wf._estado_conexion=100;//Si no se define no arranca	//wf._estado_conexion=1;					//Arranco en WiFi Desconectado
		wf._automatizacion=WF_CONNECT_TCP;//wf._automatizacion=WF_SEND;
		wf._NO_IP=1;
		wf._DBG_EN=1;
     //----------------------- WIFI ------------------------//

		for(uint8_t i=0;i<=16;i++)
		{
			decimal[i]=1;
		}
	 //----------------------- WIFI ------------------------//

	 //---------------------- ModBUS -----------------------//

		ModBUS_Config(&mb_lr);		//ETHERNET como cliente TCP envía  ModBUS
		mb_lr._mode = CLIENTE;

	 //---------------------- ModBUS -----------------------//
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  SysTick_Config(SystemCoreClock/1000);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
  ITM0_Write("\r\n INICIO OK\r\n",strlen("\r\n INICIO OK\r\n"));
     HW_RESET(&wf);
     if (wf._DBG_EN) ITM0_Write("\r\n RESET ESP8266 \r\n",strlen("\r\n RESET ESP8266 \r\n"));
     HAL_UART_Receive_IT(&huart1,(uint8_t *)UART_RX_byte,1);
     HAL_UART_Receive_IT(&huart2,(uint8_t *)UART2_RX_byte,1);

     if(ESP8266_HW_Init(&huart1)==1)
     {
   	  ESP_HW_Init=1;
   	  if (wf._DBG_EN) ITM0_Write("\r\n ESP HW Init OK\r\n",strlen("\r\n ESP HW Init OK\r\n"));
     }
     else
		 {
		   HW_RESET(&wf);
		  if(ESP8266_HW_Init(&huart1)==1)
			  {
				  ESP_HW_Init=1;
				  if (wf._DBG_EN) ITM0_Write("\r\n ESP HW Init OK\r\n",strlen("\r\n ESP HW Init OK\r\n"));
			  }
			  else
				  {
					  ESP_HW_Init=0;
					  if (wf._DBG_EN)  ITM0_Write("\r\n ESP HW Init Fail\r\n",strlen("\r\n ESP HW Init Fail\r\n"));
				  }
		 }

     HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  /**************[ INICIO PIDO ENVIAR DATOS ]**************/
  if (ESP_HW_Init==1)
	  	  {
			if((WF_SND_FLAG==1)&&(wf._TCP_Local_Server_EN==0)&&(wf._estado_conexion>=609)&&(lr._data_available))
	  			{	lr._data_available=0;
	  				wf_snd_flag_ticks=0;
	  				WF_SND_FLAG=0;
		  			for(uint8_t i=0;i<=16;i++)
		  				{
		  					datos[i]=ModBUS_F03_Read(&mb_lr,i);//datos[i]=ModBUS_F03_Read(&mb_lr,i);
		  				}

		  				if(httpPOST2(ENDPOINT, SERVER_IP,PORT,&datos,&decimal,16,TEST_1,post,body, 512))
							{
							CopiaVector(wf._data2SND,post,strlen(post),0,'A');
							wf._n_D2SND=strlen(post);
								if(wf._automatizacion < WF_SEND)		// Send only with automation sent diasabled
								{
									EnviarDatos(&wf);
									wf._estado_conexion=TCP_SND_EN_CURSO;
								}
							}
	  			}
	  	  }
	  /**************[ FIN PIDO ENVIAR DATOS ]**************/

	  		if ((FLAG_UART1==1)||(FLAG_TIMEOUT==1))  //Si recibí datos o me fui por TimeOUT
	  		{
	  			if(FLAG_UART1==1)
	  				{
	  					CopiaVector(wf._uartRCVD,UART_RX_vect_hld,UART_RX_items,1,CMP_VECT);
	  					FLAG_UART1=0;

	  						if (error_rxdata==3)
	  						{
	  							error_rxdata=0;
	  						}
	  						if (error_rxdata==1)
	  						{
	  							error_rxdata=5;
	  							error_rxdata=0;
	  						}
	  				}
	  			if(FLAG_TIMEOUT==1)
	  					{
	  						FLAG_TIMEOUT=0;
	  					}

	  			if (ESP_HW_Init==1) //Si el módulo se inició correctamente
	  				{
	  					/*************** Copio y proceso info recibida ***************/
	  					wf._n_orig=UART_RX_items;
	  					CopiaVector(wf._uartRCVD,UART_RX_vect_hld,UART_RX_items,1,CMP_VECT);
	  					resultado=AT_ESP8266_ND(&wf);
	  					}

	  		}
// AGREGAR TIMER EN MS TICKS PARA HABILITAR ESTADO Y CUENTA TODOS EN EL STRUCT
	  		if((FLAG_UART2 == 1)||(lr.tmr_dly_ON==1))  //Evento de dato recibido LoRA debo verificar que es
	  		{
	  			if(FLAG_UART2==1)
	  				{
	  				FLAG_UART2=0;
	  				LoRa_decode(&lr);
	  				if(lr._data_available)
	  				{
	  					CopiaVector(lr.dataRCV_hld,lr.dataRCV,lr._n_dataRCV,1,"D");
	  					char num[6];
	  					int i=0;
	  					int n=0;
	  					int lnn=0;
	  					lnn=strlen(lr.dataRCV_hld);
	  					while(i < lnn-1)
	  					{
							if( i!=0) i++;
							int j=0;

								while(lr.dataRCV_hld[i] != ';')
								{
									if(lr.dataRCV_hld[i] != '.')
									{
										num[j]=lr.dataRCV_hld[i];
										j++;
									}
									i++;
								}
							num[j]='\0';
							ModBUS_F03_Assign(&mb_lr,n,atoi(num,10));
							n++;//Incremento posición  a almacenar
	  					}
	  				}
	  				}

	  			if(lr.tmr_dly_ON==1)
	  				{
	  					lr.tmr_dly_ON=0;
	  					LoRa_reset_sndTIMER(&lr,3000);
						lr.dest_address[0]='\0';
						lr.txbuff[0]='\0';
			  			strncat(lr.dest_address,"1",1);
			  			strncat(lr.txbuff,"prueba de envio de mensaje de texto",strlen("prueba de envio de mensaje de texto"));
			  			lr.txitems=strlen("prueba de envio de mensaje de texto");
		  				lr.estado=_SENT;										//
			  			error=LoRa_Send(&lr,&huart2);
	  				}
	  		}

	  		if (ESP_HW_Init==1) //Si el módulo se inició correctamente
	  			{
	  				conexion=WiFi_Conn_ND(&wf,&huart1,1);	//Tiene que ir en el main el chequeo es constante
	  			}
	  		if (esp_restart==1) //WRNNG Hardcoded RESET WIFI
	  			{

	  				HW_RESET(&wf);
	  				HAL_Delay(5000);//210419
	  				esp_restart=0;
	  				wf._estado=0;
	  				wf._estado_conexion=100;
	  				ConectarWIFI(&wf);
	  			}
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000;
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
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim2, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_INACTIVE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 100;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 150;
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
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim3, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_INACTIVE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  huart1.Init.BaudRate = 115200;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

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
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PCB_LED_GPIO_Port, PCB_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, WIFI_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PCB_LED_Pin */
  GPIO_InitStruct.Pin = PCB_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PCB_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY_BTN_Pin */
  GPIO_InitStruct.Pin = KEY_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KEY_BTN_GPIO_Port, &GPIO_InitStruct);



  /*Configure GPIO pin : WIFI_EN_Pin */
  GPIO_InitStruct.Pin = WIFI_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(WIFI_EN_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


int ITM0_Write( char *ptr, int len)
{
 int DataIdx;

  for(DataIdx=0; DataIdx<len; DataIdx++)
  {
    ITM_SendChar(*ptr++);
  }
  return len;
}

void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

	ms_ticks++;	//100 ms

	ESP_ticks++;
	if(mb_lr._w_answer) MB_TOUT_ticks++;
	if ( mb_lr._w_answer && (mb_lr._timeout < MB_TOUT_ticks))
		{
			mb_lr._w_answer=0;
			MB_TOUT_ticks=0;
		}
// ENVIO DATOS LoRa ---------------------------------------------------------------//

	if(lr.tmr_dly_en==1)
	{
		lr.tmr_dlyCNT++;
		if(lr.tmr_dlyCNT > lr.tmr_dly)
		{
			lr.tmr_dly_ON=1;
			lr.tmr_dly_en=0;
		}
	}
// ENVIO DATOS LoRa ---------------------------------------------------------------//
// ENVIO DATOS WF ---------------------------------------------------------------//

	if((wf._estado_conexion==609 || wf._estado_conexion==700)&&(wf._TCP_Local_Server_EN==0))  wf_snd_flag_ticks++;

	if(wf_snd_flag_ticks>= 2000 && wf._ejecucion!=1 && wf._TCP_Local_Server_EN==0)		 	  WF_SND_FLAG=1;

// ENVIO DATOS WF ----------------------------------- ---------------------------//

if (ms_ticks==100)//(ms_ticks==250)//(ms_ticks==50)
  {

	  ms_ticks=0;
	  min_ticks++;

	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

	  if(min_ticks==2)//if(min_ticks==10)
		  {
		  	  min_ticks=0;  /* SETEO CADA 2 min*/
		  }
  }

	if(EN_USART1_TMR==1) USART1_ticks++;

	if(USART1_ticks>=2)//if(USART1_ticks>=10)
	{
		USART1_ticks=0;
		FLAG_USART1=1;
		EN_USART1_TMR=0;
		items_rx=uart1pass;
		uart1pass=0;
	}

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
	if(wf._estado_conexion==4)//if((wf._estado_conexion!=1)&&(wf._estado_conexion!=2)&&(resultado!=20)&&(resultado!=24)) //Solo cuento cuando no estahaciendo otra cosa
	{
		ticks++;
	}
	else
	{
		ticks=0;
	}

    if((wf._estado_conexion==TCP_CONN_EN_CURSO) || (wf._estado_conexion==CONEXION_EN_CURSO)) wf._ticks2++;	//Conteo

 	if((wf._estado_conexion==CONEXION_EN_CURSO) && (wf._ticks2 >10000))
 	{
 		wf._ticks2=0;
 		esp_restart=1;
 	}
 	if(wf._estado_conexion==TCP_CONN_ERROR || wf._estado_conexion==CONEXION_ERROR) wf._ticks2++;

if(wf._ejecucion==1)
	{
		if (FLAG_TIMEOUT!=1)
		{
			if(wf._instruccion!=2) wf._ticks++;//-----------------------Solo cuento una vez reconcido el timeout, cuando entro al timeout no cuento
			if(wf._instruccion==2) wf._ticks2++;
		}
		if ((wf._instruccion!=2)&&(wf._ticks > 5500))
		{
			FLAG_TIMEOUT=1;
			if(huart1.Instance->CR1 == 0x200C)  //--------------------Evito error UART colgado
			{
				HAL_UART_Receive_IT(&huart1,(uint8_t *)UART_RX_byte,1);
				EN_UART1_TMR=0; //OBS-VER Para que me vuelva a habilitar el timer
			}
		}
		if ((wf._instruccion==2)&&(wf._ticks2 > 20500)) //if (wf._ticks > 5000)
		{
			FLAG_TIMEOUT=1;
			if(huart1.Instance->CR1 == 0x200C)  //--------------------Evito error UART colgado
			{
				HAL_UART_Receive_IT(&huart1,(uint8_t *)UART_RX_byte,1);
				EN_UART1_TMR=0; //OBS-VER Para que me vuelva a habilitar el timer
			}
		}
	}
	else
	{
		wf._ticks=0;
	}
  /* USER CODE END SysTick_IRQn 1 */
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *ERRUART)

{
	if(ERRUART->Instance==USART1)
	{
		 volatile int aore=0;
		 volatile int bore=0;
		//Al leer los registros de esta forma SR y luego DR se resetean los errores de Framing Noise y Overrun FE NE ORE
		 wf._debug_count9++;
		 aore=ERRUART->Instance->SR;
		 bore=ERRUART->Instance->DR;
		 HAL_UART_DeInit(ERRUART);
		 MX_USART1_UART_Init();
		 HAL_UART_Receive_IT(ERRUART,(uint8_t *)UART_RX_byte,1);
	}
	if(ERRUART->Instance==USART2)
	{
		 volatile int aore=0;
		 volatile int bore=0;

		//Al leer los registros de esta forma SR y luego DR se resetean los errores de Framing Noise y Overrun FE NE ORE
		 wf._debug_count9++;
		 aore=ERRUART->Instance->SR;
		 bore=ERRUART->Instance->DR;
		 HAL_UART_DeInit(ERRUART);
		 MX_USART2_UART_Init();
		 HAL_UART_Receive_IT(ERRUART,(uint8_t *)UART2_RX_byte,1);
	}
}

void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim2)
{

		wf._debug_count10++;

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *INTSERIE)
{

// WiFi	USART 1 TIMER2
	if(INTSERIE->Instance==USART1)
		 {
			UART_RX_vect[UART_RX_pos]=UART_RX_byte[0];
			UART_RX_pos++;
			if(UART_RX_pos>=1022) UART_RX_pos=1022;
			HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);//HAL_TIM_Base_Start_IT(&htim7);	//Habilito el timer
			TIM2->CNT=1;
			EN_UART1_TMR=1;	//Habilito Timeout de software
			HAL_UART_Receive_IT(INTSERIE,(uint8_t *)UART_RX_byte,1);
		 }
// LoRa USART2 TIMER3
	if(INTSERIE->Instance==USART2)
		 {
			UART2_RX_vect[UART2_RX_pos]=UART2_RX_byte[0];
			UART2_RX_pos++;
			if(UART2_RX_pos>=512) UART2_RX_pos=512;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);
			HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);//HAL_TIM_Base_Start_IT(&htim7);	//Habilito el timer
			TIM3->CNT=1;
			EN_UART2_TMR=1;	//Habilito Timeout de software
			HAL_UART_Receive_IT(INTSERIE,(uint8_t *)UART2_RX_byte,1);
		 }
 }

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *TIMER)
{
// WiFi	USART 1 TIMER2
		if(TIMER->Instance==TIM2)
			{
				 HAL_TIM_OC_Stop_IT(TIMER, TIM_CHANNEL_1); //Paro el timer
				 FLAG_UART1=1;
				 EN_UART1_TMR=0;
				 UART_RX_items=UART_RX_pos;
				 UART_RX_pos=0;
				 UART_RX_vect[1022]='\0'; //Finalizo el vector a la fuerza ya que recibo hasta 124
				 CopiaVector(UART_RX_vect_hld,UART_RX_vect,UART_RX_items,1,CMP_VECT);
				 HAL_UART_Receive_IT(&huart1,(uint8_t *)UART_RX_byte,1); //Habilito le recepcón de puerto serie al terminar
				 if (wf._DBG_EN==1)
				 {
					 ITM0_Write((uint8_t *)UART_RX_vect_hld,UART_RX_items);
				 }
		}
// LoRa USART2 TIMER3
		if(TIMER->Instance==TIM3)
			{
				 HAL_TIM_OC_Stop_IT(TIMER, TIM_CHANNEL_1); //Paro el timer
				 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
				 FLAG_UART2=1;
				 EN_UART2_TMR=0;
				 UART2_RX_items=UART2_RX_pos;
				 UART2_RX_pos=0;
				 UART2_RX_vect[512]='\0'; //Finalizo el vector a la fuerza ya que recibo hasta 124
				 CopiaVector(lr.rxbuff,UART2_RX_vect,UART2_RX_items,1,CMP_VECT);
				 lr.rxitems=UART2_RX_items;
				 HAL_UART_Receive_IT(&huart2,(uint8_t *)UART2_RX_byte,1); //Habilito le recepcón de puerto serie al terminar
				 if (wf._DBG_EN==1)
				 {
					 ITM0_Write("\r\nData LoRa recibida = ",strlen("\r\nData LoRa recibida = "));
					 ITM0_Write((uint8_t *)UART2_RX_vect,UART2_RX_items);
					 ITM0_Write("\r\n",strlen("\r\n"));
				 }
		}
}

uint8_t ESP8266_HW_Init(UART_HandleTypeDef *SerialPort) //Devuelve 1 si reinició OK, y 0 si no
{
	  do{
		  HAL_UART_Transmit(SerialPort, "AT+RESTORE\r\n",strlen("AT+RESTORE\r\n"),100);
		  HAL_Delay(500);

		  wf._n_fcomp=strlen("ready");
		  wf._n_orig=UART_RX_items;

		  while(FT_String_ND(UART_RX_vect_hld,&wf._n_orig,"ready",&wf._n_fcomp,wf._uartRCVD_tok,&wf._n_tok,&ntestc,&wf._id_conn,&wf._overflowVector,FIND)!=1)
		  {
			  	  wf._n_orig=UART_RX_items;
			  	  if (ESP_ticks>=5000)
			  		 {
			  		 ESP_InitF=1;
			  		 break;
			  		 }
		  }

		  if(ESP_InitF==0)	//Si encontró la sentencia anterior analizo la siguiente
		  {
			  wf._n_fcomp=strlen("ready");
			  wf._n_orig=UART_RX_items;
			  while(FT_String_ND(UART_RX_vect_hld,&wf._n_orig,"ready",&wf._n_fcomp,wf._uartRCVD_tok,&wf._n_tok,&ntestc,&wf._id_conn,&wf._overflowVector,FIND)!=1)
			  {
				  wf._n_orig=UART_RX_items;
				  if (ESP_ticks>=5000)
					 {
					 break;
					 }
			  }
		  }

		  if (ESP_ticks<5000)
		  {
			  ESP_REinit=10;
			  ESP_ticks=0;
		  }
		  else
		  {
			  ESP_REinit++;
			  ESP_ticks=0;
		  }

	  } while (ESP_REinit<=5);

	  if(ESP_REinit==10)
	  {
		  return(1);
	  }
	  else
	  {
		  return(0);
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
