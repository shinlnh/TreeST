/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdio.h>
#include <math.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SHT40_ADDR 0x44 << 1   // �?ịa chỉ I2C của SHT40 (cần dịch trái 1 bit)
#define BH1750_ADDR 0x23 << 1  // �?ịa chỉ I2C của BH1750 (cần dịch trái 1 bit)
//----
// Lệnh đ�?c nhiệt độ và độ ẩm từ SHT40
#define SHT40_MEASURE_CMD 0xFD

// Lệnh đ�?c cư�?ng độ ánh sáng từ BH1750
#define BH1750_MEASURE_CMD 0x10
#define LCD_CS_GPIO_Port GPIOA    // GPIO port của chân CS (ch�?n port phù hợp)
#define LCD_CS_Pin GPIO_PIN_7    // Pin của chân CS (ch�?n pin phù hợp)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t Presence=0;
uint8_t Rh_byte1=0;
uint8_t Rh_byte2=0;
uint8_t Temp_byte1=0;
uint8_t Temp_byte2=0;
uint8_t SUM=0;
uint16_t TEMP=0;
uint16_t RH=0;
float TemperatureDHT=0;//SHT22
float HumidityDHT=0;//SHT22
uint16_t i=1;
float temperature=0;//SHT22
float humidity=0;//SHT22
float light_intensity=0;
uint16_t light=0;

char buffer[500];
const char *stop = "Tam dung lay mau\n";

const char *C1;
const char *C2;
const char *C3;
const char *Hello = "Chuong trinh khao sat, doi tuong: SEN DA\n\n";
//const char *data="Lan do thu: ";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Chế độ Output Push-Pull
    GPIO_InitStruct.Pull = GPIO_NOPULL;         // Không sử dụng Pull-Up/Down
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Tốc độ thấp
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;     // Chế độ Input
    GPIO_InitStruct.Pull = GPIO_NOPULL;         // Không sử dụng Pull-Up/Down
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}
void Delay_us(uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim2, 0);//Reset timer
	while ((__HAL_TIM_GET_COUNTER(&htim2))<us);//Doi
}
void DHT22_Start(void)
{
	Set_Pin_Output(GPIOB, GPIO_PIN_5); // set the pin as output
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	Delay_us(1200);//Delay 1.2ms for SHT reset
	HAL_GPIO_WritePin (GPIOB, GPIO_PIN_5, GPIO_PIN_SET);   // pull the pin high
	Delay_us(30);   // wait for 30us
	Set_Pin_Input(GPIOB, GPIO_PIN_5);   // set as input
	//Start receive data from SHT22
}
uint8_t DHT22_Check_Response (void)//Receive data <1bit>
{
	Set_Pin_Input(GPIOB, GPIO_PIN_5);   // set as input
	uint8_t Response = 0;
	Delay_us(40);  // wait for 40us
	if (!(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5))) // if the pin is low
	{
		Delay_us(80);   // wait for 80us
		if ((HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_5))) Response = 1;
		// if the pin is high, response is ok
		else Response = -1;
		//we need 40+80 = 120us to wait for sensor response that it readys
	}
	while ((HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_5)));   // wait for the pin to go low
	return Response;
}
uint8_t DHT22_Read (void)// We need 40bit for one-wire, so in loop, we use i from 0->7 for 1byte
{
	uint8_t i,j;
	for (j=0;j<8;j++)
	{
		//after response, sht22 start transmit 1bit, sht22 put the pin high (by res 10k-vcc)
		while (!(HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_5)));   // wait for the pin to go high
		Delay_us(40);   // wait for 40 us

		if (!(HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_5)))   // if the pin is low
		{
			i&= ~(1<<(7-j));   // write 0

		}
		else i|= (1<<(7-j));  // if the pin is high, write 1
		while ((HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_5)));  // wait for the pin to go low
	}

	return i;
}
void DHT22()
{
	DHT22_Start();
	Presence = DHT22_Check_Response();
	Rh_byte1 = DHT22_Read ();
	Rh_byte2 = DHT22_Read ();
	Temp_byte1 = DHT22_Read ();
	Temp_byte2 = DHT22_Read ();
	SUM = DHT22_Read();

	TEMP = ((Temp_byte1<<8)|Temp_byte2);
	RH = ((Rh_byte1<<8)|Rh_byte2);

	TemperatureDHT = (float) (TEMP/10.2);
	HumidityDHT = (float) (RH/10.5);

	HAL_Delay(1000);
}



void SHT40_ReadTemperatureHumidity(float *temperature, float *humidity) {
    uint8_t cmd = SHT40_MEASURE_CMD;
    uint8_t data[6]; // 6 byte dữ liệu trả v�? (2 byte nhiệt độ, 1 byte CRC, 2 byte độ ẩm, 1 byte CRC)

    // Gửi lệnh đo
    HAL_I2C_Master_Transmit(&hi2c1, SHT40_ADDR, &cmd, 1, HAL_MAX_DELAY);

    // �?ợi 10ms để cảm biến đo và trả v�? dữ liệu
    HAL_Delay(10);

    // �?�?c dữ liệu từ SHT40
    if (HAL_I2C_Master_Receive(&hi2c1, SHT40_ADDR, data, 6, HAL_MAX_DELAY) == HAL_OK) {
        // Chuyển đổi dữ liệu nhiệt độ
        uint16_t temp_raw = (data[0] << 8) | data[1];
        *temperature = -45.0 + 175.0 * (float)temp_raw / 65535.0;

        // Chuyển đổi dữ liệu độ ẩm
        uint16_t hum_raw = (data[3] << 8) | data[4];
        *humidity = -6.0 + 125.0 * (float)hum_raw / 65535.0;
    } else {
        // Xử lý lỗi đ�?c dữ liệu nếu có
        *temperature = -999.0; // Báo lỗi với giá trị bất thư�?ng
        *humidity = -999.0;    // Báo lỗi với giá trị bất thư�?ng
    }
}

void BH1750_ReadLightIntensity(float *light_intensity)
{
    uint8_t cmd = BH1750_MEASURE_CMD;
    uint8_t data[2]; // 2 byte dữ liệu trả v�?

    // Gửi lệnh đo cư�?ng độ ánh sáng
    HAL_I2C_Master_Transmit(&hi2c1, BH1750_ADDR, &cmd, 1, HAL_MAX_DELAY);

    // �?ợi th�?i gian để cảm biến đo và trả v�? dữ liệu (khoảng 180ms)
    HAL_Delay(180);

    // �?�?c dữ liệu từ BH1750 (2 byte)
    if (HAL_I2C_Master_Receive(&hi2c1, BH1750_ADDR, data, 2, HAL_MAX_DELAY) == HAL_OK)
    {
        // Chuyển đổi giá trị ánh sáng (Lux) từ 2 byte dữ liệu
        light = (data[0] << 8) | data[1];
        *light_intensity = 1.0*(float) light/1.2;
    }
    else
    {
        // Xử lý lỗi đ�?c dữ liệu nếu có
        *light_intensity = 0xFFFF; // Báo lỗi với giá trị bất thư�?ng
    }
}
/*void SendData(UART_HandleTypeDef *huart) {


    // Tạo chuỗi chứa các giá trị
    snprintf(buffer, sizeof(buffer),
    		 "Lan do thu: %d"
             "\nNhiet do dat: %.2f degC\n"
             "Do am dat: %.2f%%\n"
             "Nhiet do khong khi: %.2fdegC\n"
             "Do am khong khi: %.2f%%\n"
             "Cuong do anh sang: %f Lux\n\n",
             i, TemperatureDHT, HumidityDHT, temperature, HumidityDHT, light_intensity);

    // Gửi chuỗi qua USART
    HAL_UART_Transmit(huart, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);

    if(CheckTem() == 1)
    	C1 = "Nhiet do Kha thap, Nguy co hu hai cao. Can duy chuyen den noi am ap !\n ";
    else if (CheckTem() == 2)
    	C1 = "Nhiet do ly tuong, Cay phat trien binh thuong.!\n ";
    else if (CheckTem() == 3)
    	C1 = "nhiet do cao, Nguy co chay nang. Can lam mat ngay lap tuc.!\n ";
    if(CheckHum() == 1 )
    	C2 = "Do am thap, so luong hoi nuoc qua it. De xuat phun suong, tuoi nuoc cap am.!\n";
    else if(CheckHum() == 2 )
    	C2 = "Do am ly tuong, Thich hop cho su phat trien cua cay.!\n";
    else if	(CheckHum() == 3 )
        C2 = "Do am cao, Canh bao cay de benh va chet.!\n";
    if(CheckLight() == 1)
    	C3 = "Anh sang yeu, Cay kem phat trien. Canh bao than dai la nhat mau!\n";
    else if(CheckLight() ==2 )
    	C3 = "Anh sang phu hop, Cay phat trien on dinh!\n";
    else if(CheckLight() ==3)
    	C3 = "Anh sang tot, Cay phat trien manh me, mau sac dep, chat luong tot!\n";
    else
    	C3 = "Anh sang manh, Cay quang hop lien tuc, Can han che anh sang.!\n";
    HAL_UART_Transmit(huart, (uint8_t *)C1, strlen(C1), HAL_MAX_DELAY);
    HAL_UART_Transmit(huart, (uint8_t *)C2, strlen(C2), HAL_MAX_DELAY);
    HAL_UART_Transmit(huart, (uint8_t *)C3, strlen(C3), HAL_MAX_DELAY);

    }*/
int CheckTem()
{
	if(temperature < 10 && TemperatureDHT <10)
	{
		return 1;
	}
	else if (temperature > 16 && TemperatureDHT > 16 &&
			 temperature < 32 && TemperatureDHT < 32
			)
		return 2;
	else
		return 1;
}
int CheckHum()
{
	if(HumidityDHT < 30 && HumidityDHT < 30)
	{
		return 1;
	}
	else if (HumidityDHT > 35 && HumidityDHT > 35 &&
		     HumidityDHT < 60 && HumidityDHT < 60
			)
		return 2;
	else
		return 3;
}
int CheckLight()
{
	if(light_intensity < 1000)
	{
		return 1;
	}
	else if (light_intensity > 1000 && light_intensity < 3000
			)
		return 2;
	else if (light_intensity >3000 && light_intensity < 5000)
		return 3;
	else
		return 4;
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
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);  // for us Delay
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_UART_Transmit(&huart1, (uint8_t *)Hello, strlen(Hello), HAL_MAX_DELAY);
  while (1)
  {
	  DHT22();
	  SHT40_ReadTemperatureHumidity(&temperature, &humidity);
	  BH1750_ReadLightIntensity(&light_intensity);
	  if(CheckTem() == 1)
	  	  		  	    	C1 = "Nhiet do Kha thap, Nguy co hu hai cao. Can duy chuyen den noi am ap !\n ";
	  	  		  	    else if (CheckTem() == 2)
	  	  		  	    	C1 = "Nhiet do ly tuong, Cay phat trien binh thuong.!\n ";
	  	  		  	    else if (CheckTem() == 3)
	  	  		  	    	C1 = "nhiet do cao, Nguy co chay nang. Can lam mat ngay lap tuc.!\n ";
	  	  		  	    if(CheckHum() == 1 )
	  	  		  	    	C2 = "Do am thap, so luong hoi nuoc qua it. De xuat phun suong, tuoi nuoc cap am.!\n";
	  	  		  	    else if(CheckHum() == 2 )
	  	  		  	    	C2 = "Do am ly tuong, Thich hop cho su phat trien cua cay.!\n";
	  	  		  	    else if	(CheckHum() == 3 )
	  	  		  	        C2 = "Do am cao, Canh bao cay de benh va chet.!\n";
	  	  		  	    if(CheckLight() == 1)
	  	  		  	    	C3 = "Anh sang yeu, Cay kem phat trien. Canh bao than dai la nhat mau!\n";
	  	  		  	    else if(CheckLight() ==2 )
	  	  		  	    	C3 = "Anh sang phu hop, Cay phat trien on dinh!\n";
	  	  		  	    else if(CheckLight() ==3)
	  	  		  	    	C3 = "Anh sang tot, Cay phat trien manh me, mau sac dep, chat luong tot!\n";
	  	  		  	    else
	  	  		  	    	C3 = "Anh sang manh, Cay quang hop lien tuc, Can han che anh sang.!\n";
	  	  	snprintf(buffer, sizeof(buffer),
	  	  	    		 "Lan do thu: %d"
	  	  	             "\nNhiet do dat: %.2f degC\n"
	  	  	             "Do am dat: %.2f%%\n"
	  	  	             "Nhiet do khong khi: %.2fdegC\n"
	  	  	             "Do am khong khi: %.2f%%\n"
	  	  	             "Cuong do anh sang: %f Lux\n\n",
	  	  	             i, TemperatureDHT, HumidityDHT, temperature, HumidityDHT, light_intensity);
	  	  	    // Gửi chuỗi qua USART
	  	  		strcat(buffer,C1);
	  	  		strcat(buffer,C2);
	  	  		strcat(buffer,C3);
	  	  	    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
	  	  	    Delay_us(100);

	  	  	    //HAL_UART_Transmit(&huart1, (uint8_t *)C1, strlen(C1), HAL_MAX_DELAY);
	  	  	    //HAL_UART_Transmit(&huart1, (uint8_t *)C2, strlen(C2), HAL_MAX_DELAY);
	  	  	    //HAL_UART_Transmit(&huart1, (uint8_t *)C3, strlen(C3), HAL_MAX_DELAY);

	  while(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0));

	  i++;
	 // HAL_Delay(500);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.ClockSpeed = 100000;
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

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 47;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
