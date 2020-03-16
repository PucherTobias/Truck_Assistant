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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "MY_FLASH.h"
#include "pid_controller.h"
#include "pid_regler.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc3;

ETH_HandleTypeDef heth;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
uint32_t adcvaluek;					// Distanzsensor ADC value
uint32_t adc_steering;			// Winkelsensor ADC value
uint32_t adcval[2] ;				// Joysticks adc Wert
uint32_t lenken = 0;
uint32_t gas = 0;						// gas & lenk wert
int i = 0;

float sensork;						// Sensorwert in cm
float steering_conv;			// Winkelsensorwert in °
float steering_trailer;
float steering_average[10] = {0};
float steering_conv_avg = 0;
int av =0 ;

// Georg
uint16_t velocity[10000] = {0} ;
int16_t angle[10000] = {0} ;					//Speicherfelder
uint16_t thrust[10000]	= {0};
uint16_t steering[10000] = {0};
uint8_t bluebuffer[4] = {0} ;
uint8_t bluetransbuffer[14];
int statuscount = 0;
int buzzer_count = 0;
int statustest = 0;
int freigabe = 0 ;
int angle_error = 0;
int distance_error = 0;




//unsigned char velocityASCII[10000] ;		// ASCII Felder wegen UART
//char steeringASCII[10000] ;
int iw=0 ;
int itrans = 0 ;
int icom = 0 ;
int b_count = 0;
int b_sync = 0;
int bluebufferval = 0;
int length = 0 ;
int count_10ms=0 ;
int count_100ms=0 ;							// Verzögerungen
int count_1s=0 ;
int count_100ms_freigabe = 0;
int count_1s_freigabe = 0;
float countspin=0 ;							// Messung von Sensorausgang
float spins = 0 ;								// Umdrehungen Inkrementaldrehgeber
uint8_t spinstrans = 0 ;				// Umdrehungen in 8bit (UART-8bit)					
int setval_memory_storage	 = 0 ;	
int memory_start = 0 ;
int bluebutton = 0 ;
int memorystorebutton = 0 ;
int autobetrieb=0;							// Fuzzy
int handbetrieb=0;
int angle_sync=0;
int angleconv_sync= 237;
int deleted=0;
int adcvaluekavg[10];
int adcvalue_del =0;
int adcvalue_avg =0;
//Pucher
uint16_t w_velocity[10000] = {0};
int16_t w_angle[10000] = {0};
uint16_t w_thrust[10000] = {0};
uint16_t w_steering[10000] = {0};
uint16_t flasharray_length[1];
uint32_t flash_index = 0;

int curve_was_taken = 0;

int auto_steering = 90; //sollwert
int auto_thrust = 0; //sollwert
int auto_steering_pwm = 0;
int auto_thrust_pwm = 0;
float auto_angle_w = 0; //sollwert
float auto_angle_y = 0; //istwert
int auto_velocity_w = 0; //sollwert
int auto_velocity_y = 0; //istwert


pid_regler_struct pid1;
float Ti = 5;

int photodiode1 = 0;
int photodiode2 = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM10_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */
long map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


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
	int setval = 0;
	bluebuffer[1] = 128 ;
	static int32_t uwtick_Hold10ms;
  static int32_t uwtick_Hold100ms; 
  static int32_t uwtick_Hold1s;
	static int32_t uwtick_Hold100ms_freigabe; 
	static int32_t uwtick_Hold1s_freigabe;
	int uw_count = 0;
	int uw_result = 0;
	
	
	uwtick_Hold10ms=0;
  uwtick_Hold100ms=0;
  uwtick_Hold1s=0;
	uwtick_Hold100ms_freigabe = 0; 
	uwtick_Hold1s_freigabe = 0;
	
	
	bluetransbuffer[1] = 1;
	bluetransbuffer[2] = 2;
	bluetransbuffer[3] = 3;
	bluetransbuffer[4] = 4;
	bluetransbuffer[5] = 5;
	bluetransbuffer[6] = 6;
	bluetransbuffer[7] = 7;
	bluetransbuffer[8] = 8;
	bluetransbuffer[9] = 9;
	bluetransbuffer[10] = 10;
	bluetransbuffer[11] = 11;
	bluetransbuffer[12] = 12;
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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USB_DEVICE_Init();
  MX_ADC3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_TIM10_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); //Start the Motor PWM
	HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1) ;
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); //Start the servo PWM
	HAL_TIM_Base_Start_IT(&htim2) ;
	
	HAL_TIM_Base_Start(&htim1) ;					// Timer für ADC Abfrage 
	HAL_ADC_Start_DMA(&hadc3,adcval,2) ;	// DMA Abfrage von ADC value, Speicherung in adcval0 und adcval 1
	
	HAL_UART_Receive_DMA(&huart4,bluebuffer,sizeof(bluebuffer)) ;
	
	//MOTOR / SERVO DEFAULT BEGIN
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0); //default kein Gas
	htim4.Instance->CCR1 = 750; //default 90°
	//MOTOR / SERVO DEFAULT END
	
	//FLASH READ BEGIN
	//Flash auslesen -> Anzahl gespeicherter Werte, Sollwertkurve |
	//Flasharray Length
//	MY_FLASH_SetSectorAddrs(7, 0x080C0000);
//	MY_FLASH_ReadN(0, flasharray_length, 1, DATA_TYPE_16);
//	
//	//Velocity
//	MY_FLASH_SetSectorAddrs(8, 0x08100000);
//	MY_FLASH_ReadN(0, w_velocity, flasharray_length[0], DATA_TYPE_16);

//	//Angle
//	MY_FLASH_SetSectorAddrs(9, 0x08140000);
//	MY_FLASH_ReadN(0, w_angle, flasharray_length[0], DATA_TYPE_16);

//	//Thrust
//	MY_FLASH_SetSectorAddrs(10, 0x08180000);
//	MY_FLASH_ReadN(0, w_thrust, flasharray_length[0], DATA_TYPE_16);
//	
//	//Steering
//	MY_FLASH_SetSectorAddrs(11, 0x081C0000);
//	MY_FLASH_ReadN(0, w_steering, flasharray_length[0], DATA_TYPE_16);
	//FLASH READ END
	
//REGLER INITIALISIEREN

pid1.kp = 1.5;
pid1.ki = 1/Ti;
pid1.kd = 0;
pid1.T = 0.01;
pid1.u_min = -30;
pid1.u_max = 30;

pid_init(&pid1);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		
//		if(HAL_GPIO_ReadPin(Flash_start_GPIO_Port, Flash_start_Pin))			// Übertragung Start
//		{	memory_start = 1 ;
//			
//		}
//		if(HAL_GPIO_ReadPin(Flash_stop_GPIO_Port,Flash_stop_Pin))
//		{	memory_start = 2 ;
//			memorystorebutton++;
//			curve_was_taken = 2;
//		}
		
	

		
		while(freigabe==0)	{
			
			HAL_GPIO_WritePin(Status_LED1_GPIO_Port,Status_LED1_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(Status_LED2_GPIO_Port,Status_LED2_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(Status_LED_3_GPIO_Port,Status_LED_3_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(Status_LED_4_GPIO_Port,Status_LED_4_Pin,GPIO_PIN_SET);
			
			if(HAL_GPIO_ReadPin(Freigabe_GPIO_Port,Freigabe_Pin))	{
				freigabe=1;
		}
			uw_count=uwTick ;
	}
	
	while(freigabe==1)	{
		
//		if(HAL_GPIO_ReadPin(Freigabe_GPIO_Port,Freigabe_Pin))	{
//				freigabe=0 ;
//			}
		
		uw_result = uwTick - uw_count ;
		
		if((bluebuffer[3]==16) || (bluebuffer[3]==24) ){						//Abfrage ob Handbetrieb 
				autobetrieb= 1;																					// oder Autobetrieb
				handbetrieb= 0;
		}
		else	{															
				autobetrieb= 0;
				handbetrieb= 1;
		}
		if((bluebuffer[3] == 8))	{
				angle_sync = 1;
		}
		
	
		HAL_UART_Transmit(&huart4,bluetransbuffer,sizeof(bluetransbuffer),20);

		//PHOTODIODE
		if(HAL_GPIO_ReadPin(photodiode1_GPIO_Port, photodiode1_Pin)) {
			photodiode1	= 1;
		} else { 
			photodiode1 = 0;
		}
		
		if(HAL_GPIO_ReadPin(photodiode2_GPIO_Port, photodiode2_Pin)) {
			photodiode2	= 1;
		} else { 
			photodiode2 = 0;
		}
		
		if(sensork==0){
			distance_error=1 ;
		}
		if(adc_steering==0){
			angle_error==1;
			}
		
			
		if(icom>=9999)
			icom=9999 ;
		if(icom<0)
			icom=0;
		
		if(angle_sync==1)	{
			angleconv_sync = steering_conv_avg;
			angle_sync=0 ;
		}
		
	
		if( uw_result - uwtick_Hold10ms >= 10 ) {			// 10ms Zykluszeit 
			uwtick_Hold10ms += 10;
			count_10ms++;
			
		HAL_ADC_Start(&hadc1);		
		if(HAL_ADC_PollForConversion(&hadc1,10) == HAL_OK)	{			// Single conversion für Distanz
			adcvaluek = HAL_ADC_GetValue(&hadc1);
			sensork=2*(2076.0/(adcvaluek-11));
			if(adcvaluek<=1){
				adcvaluek=35;
			}
			adcvalue_del=adcvaluekavg[9];
			adcvaluekavg[9]=adcvaluekavg[8];
			adcvaluekavg[8]=adcvaluekavg[7];
			adcvaluekavg[7]=adcvaluekavg[6];
			adcvaluekavg[6]=adcvaluekavg[5];
			adcvaluekavg[5]=adcvaluekavg[4];
			adcvaluekavg[4]=adcvaluekavg[3];
			adcvaluekavg[3]=adcvaluekavg[2];
			adcvaluekavg[2]=adcvaluekavg[1];
			adcvaluekavg[1]=adcvaluekavg[0];
			adcvaluekavg[0]=adcvaluek;
			
			adcvalue_avg = (adcvaluekavg[9]+adcvaluekavg[8]+adcvaluekavg[7]+adcvaluekavg[6]+adcvaluekavg[5]+adcvaluekavg[4]+adcvaluekavg[3]+adcvaluekavg[2]+adcvaluekavg[1]+adcvaluekavg[0])/10 ;

			
		}	
		
		HAL_ADC_Start(&hadc2);		
		if(HAL_ADC_PollForConversion(&hadc2,10) == HAL_OK)	{			// Single conversion für Winkel
				adc_steering = HAL_ADC_GetValue(&hadc2);
				steering_conv = map(adc_steering,500,4000,0,360) ;
					if(steering_conv<0){
						steering_conv=0;
					}
//		if(av<=10){
//			steering_average = steering_average+steering_conv;
//			av++;
//		}
//		
//		if(av==11){
//			steering_conv_avg = steering_average/10;
//			av=0;
//			steering_average=0;
//		}
		
		
		if(steering_conv_avg >= angleconv_sync){
			steering_trailer = map(steering_conv_avg, angleconv_sync, angleconv_sync-90, 0, 90);					// Adcvals werden mit gas und lenken gemapt, sprich, umgewandelt in gewünschte werte
		}
		
		if(steering_conv_avg < angleconv_sync){
			steering_trailer = map(steering_conv_avg, angleconv_sync, angleconv_sync+90, 0 , -90);
		}
		
		if(steering_trailer==0){
			HAL_GPIO_WritePin(angle_0_GPIO_Port,angle_0_Pin,GPIO_PIN_SET) ;
		}
		
		if(steering_trailer!=0){
				HAL_GPIO_WritePin(angle_0_GPIO_Port,angle_0_Pin,GPIO_PIN_RESET) ;
				}					


		}
		
		// REGLER 10ms
		pid1.freigabe = ON;
		pid1.w = auto_angle_w;
		pid1.x = auto_angle_y;
		pid_calc(&pid1);
		
		
		if(count_10ms%2==0)	{ //Pucher 20ms begin
			if(setval_memory_storage==1){
				if(memory_start==1){
					curve_was_taken = 1;
					velocity[icom]=	spinstrans ;		// Übergabe der Sensorwerte
					angle[icom]= steering_trailer ;
					steering[icom] = lenken ;
					thrust[icom] = gas ;
//					snprintf(velocityASCII,10000,"v %d\r",velocity[icom]) ;
//					snprintf(steeringASCII,10000,"\ts %d\n\r",steering[icom]) ;	// Umwandlung in ASCII
//					HAL_UART_Transmit(&huart3,velocityASCII,sizeof(velocityASCII),1);		// Übertragung über UART
//					HAL_UART_Transmit(&huart3,steeringASCII,sizeof(steeringASCII),1);
					icom++ ;
				}
				if((memory_start==2)||(icom>=9999)){
					velocity[icom]=	0 ;		
					angle[icom]= steering_trailer ;
					steering[icom] = lenken ;
					thrust[icom] = 0 ;
					flasharray_length[0] = icom+1;	//memory_start= 0 einfügen?
					flash_index = icom;
				}
			}
			
				
				//SOLLWERTKURVE BEGIN
//				if(autobetrieb){
//					if(i_w >= 168){
//						i_w = 168;
//						auto_start_selfcontrol = 0;
//					}
//					if(i_w <= 0)
//						i_w = 0;
//				}
//				if((autobetrieb) && (auto_start_selfcontrol)){
//					auto_steering = w_steering[i_w];
//					auto_thrust = w_thrust[i_w];
//					auto_velocity_w = w_velocity[i_w];
//					auto_angle_w = w_angle[i_w];
//					i_w++;
//				}
			
				//SOLLWERTKURVE END
			}//Pucher 20ms end
		}	// 10ms Ende
		
		deleted=steering_average[9];
		steering_average[9]=steering_average[8];
		steering_average[8]=steering_average[7];
		steering_average[7]=steering_average[6];
		steering_average[6]=steering_average[5];
		steering_average[5]=steering_average[4];
		steering_average[4]=steering_average[3];
		steering_average[3]=steering_average[2];
		steering_average[2]=steering_average[1];
		steering_average[1]=steering_average[0];
		steering_average[0]=steering_conv;
		
		steering_conv_avg = (steering_average[9] + steering_average[8] + steering_average[7] + steering_average[6] + steering_average[5] + steering_average[4] + steering_average[3] + steering_average[2] + steering_average[1] + steering_average[0])/10 ;
	
		
		
		if( uw_result - uwtick_Hold100ms >= 100 ) {																			// 100ms Zykluszeit
			uwtick_Hold100ms += 100;
			count_100ms++;
			
			//ISTWERT 				
				auto_angle_y = steering_trailer;	
			
			
				if((count_100ms%7)==0){
					if(statuscount<=8)	{
						HAL_GPIO_TogglePin(Status_LED1_GPIO_Port,Status_LED1_Pin);
						HAL_GPIO_TogglePin(Status_LED2_GPIO_Port,Status_LED2_Pin);
						HAL_GPIO_TogglePin(Status_LED_3_GPIO_Port,Status_LED_3_Pin);
						HAL_GPIO_TogglePin(Status_LED_4_GPIO_Port,Status_LED_4_Pin);
						statuscount++;
						}
					else	{
						HAL_GPIO_WritePin(Status_LED1_GPIO_Port,Status_LED1_Pin,GPIO_PIN_RESET);
						HAL_GPIO_WritePin(Status_LED2_GPIO_Port,Status_LED2_Pin,GPIO_PIN_RESET);
						HAL_GPIO_WritePin(Status_LED_3_GPIO_Port,Status_LED_3_Pin,GPIO_PIN_RESET);
						HAL_GPIO_WritePin(Status_LED_4_GPIO_Port,Status_LED_4_Pin,GPIO_PIN_RESET);
					}
				}
				
//			if(adcvalue_avg<=30){
//				if((count_100ms%7)==0){
//						HAL_GPIO_TogglePin(Status_LED1_GPIO_Port,Status_LED1_Pin);
//						HAL_GPIO_TogglePin(Status_LED2_GPIO_Port,Status_LED2_Pin);
//						HAL_GPIO_TogglePin(Status_LED_3_GPIO_Port,Status_LED_3_Pin);
//						HAL_GPIO_TogglePin(Status_LED_4_GPIO_Port,Status_LED_4_Pin);
//				}
//			}
			
			if(adc_steering<=50){
				if((count_100ms%14)==0){
					HAL_GPIO_TogglePin(Status_LED1_GPIO_Port,Status_LED1_Pin);
					HAL_GPIO_TogglePin(Status_LED2_GPIO_Port,Status_LED2_Pin);
					HAL_GPIO_TogglePin(Status_LED_3_GPIO_Port,Status_LED_3_Pin);
					HAL_GPIO_TogglePin(Status_LED_4_GPIO_Port,Status_LED_4_Pin);
				}
			}
			
		}	// 100ms Ende
			
		
		if( uw_result - uwtick_Hold1s >= 1000 ) {																				// 1s Zykluszeit
			uwtick_Hold1s += 1000;
			count_1s++;
			
				if(buzzer_count<=3)	{
					HAL_GPIO_WritePin(Status_buzzer_GPIO_Port,Status_buzzer_Pin,GPIO_PIN_SET);
					buzzer_count++;
				}
				else	{
					HAL_GPIO_WritePin(Status_buzzer_GPIO_Port,Status_buzzer_Pin,GPIO_PIN_RESET);
				}
						
		}		// 1s Ende		
		
		if(HAL_GPIO_ReadPin(NotAus_GPIO_Port, NotAus_Pin)) {								// NOT-Aus
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
			__HAL_TIM_DISABLE(&htim3);
			__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0) ;
			__HAL_TIM_DISABLE(&htim10) ;
			while(1){}
		}
		
		/* Fuzzy 
		*/
		if((autobetrieb==1)&&(handbetrieb == 0)){
		//Pucher autobetrieb BEGINN///////////////////////////////
		setval_memory_storage = 0 ;

		//Berechnung der Lenk-,Gas-Werte und Schalten der zugehörigen PWM-GPIOs
		//Lenkung - Steering
			
		if(bluebuffer[2] >= 90){
			auto_angle_w = map(bluebuffer[2], 90, 120, 0, 25);
		}
		
		if(bluebuffer[2] < 90){
			auto_angle_w = map(bluebuffer[2], 60, 90, -25 , 0);
		}	
		//auto_angle_w = Sollwert
		
		//werte von regler übernehmen, limitieren, und auf den servo schreiben.
		auto_steering = 90-(pid1.u);	
		
		if(auto_steering > 135)
			auto_steering = 135;
		if(auto_steering < 45)
			auto_steering = 45;
		auto_steering_pwm = map(auto_steering, 0, 180, 250, 1250);
			
		htim4.Instance->CCR1 = auto_steering_pwm;
			
		//Motor - Thrust
			if(bluebuffer[1]<=128)	{
			if(bluebuffer[1]<=0){
				bluebuffer[1] = 0;
			}
			auto_thrust = map(bluebuffer[1],128,0,0,31);
			
			if(auto_thrust < 7)
				auto_thrust = 0;
			if(auto_thrust >= 30)
				auto_thrust = 30;
			
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, auto_thrust);
		}
		
//		if(adcval[0]>=509)	{
//			if(adcval[0] >= 754)
//				adcval[0] = 754;
//			auto_thrust = map(adcval[0], 509, 754 , 0, 31);
//		}	
//			
//		if(auto_thrust < 7)
//			auto_thrust = 0;
//		if(auto_thrust >= 30)
//			auto_thrust = 30;
//		
//		
//		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, auto_thrust);
		
		
		}//autobetrieb ENDE///////////////////////////////////////

		if((handbetrieb == 1)&&(autobetrieb == 0)){
			setval_memory_storage=1 ;
		
			if(curve_was_taken == 2){ //wenn eine kurvenaufnahme gestartet und beendet wurde, dann werden die Werte der Sollkurve in den Flashspeicher geschrieben
			//Flash schreiben -> Anzahl gespeicherter Werte, Sollwertkurve
			//Flasharray Length
			MY_FLASH_SetSectorAddrs(7, 0x080C0000);
			MY_FLASH_WriteN(0, &flash_index, 1, DATA_TYPE_16);

			//Velocity
			MY_FLASH_SetSectorAddrs(8, 0x08100000);
			MY_FLASH_WriteN(0, velocity, flash_index, DATA_TYPE_16);

			//Angle
			MY_FLASH_SetSectorAddrs(9, 0x08140000);
			MY_FLASH_WriteN(0, angle, flash_index, DATA_TYPE_16);

			//Thrust
			MY_FLASH_SetSectorAddrs(10, 0x08180000);
			MY_FLASH_WriteN(0, thrust, flash_index, DATA_TYPE_16);

			//Steering
			MY_FLASH_SetSectorAddrs(11, 0x081C0000);
			MY_FLASH_WriteN(0, steering, flash_index, DATA_TYPE_16);

			curve_was_taken = 0;


			HAL_Delay(5000);
			}

			
		lenken = bluebuffer[2] ;
		
		i = map(lenken, 0, 180, 250, 1250);
		
		htim4.Instance->CCR1 = i;
		
		if(bluebuffer[1]>=128)	{
			if(bluebuffer[1] >= 255)
				bluebuffer[1] = 255;
		
			gas = map(bluebuffer[1], 128, 255 , 0, 31);  
		
			if(gas < 7)
				gas = 0;
		
			
			__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, gas);
		}
		if(bluebuffer[1]<=128)	{
			if(bluebuffer[1]<=0){
				bluebuffer[1] = 0;
			}
			gas = map(bluebuffer[1],128,0,0,31);
			
			if(gas < 7)
				gas = 0;
			
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, gas);
		}
  }		

  }		

	}	// while end
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_UART4|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
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
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
	
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_10B;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_TRGO;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 2;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  heth.Init.AutoNegotiation = ETH_AUTONEGOTIATION_ENABLE;
  heth.Init.PhyAddress = LAN8742A_PHY_ADDRESS;
  heth.Init.MACAddr[0] =   0x00;
  heth.Init.MACAddr[1] =   0x80;
  heth.Init.MACAddr[2] =   0xE1;
  heth.Init.MACAddr[3] =   0x00;
  heth.Init.MACAddr[4] =   0x00;
  heth.Init.MACAddr[5] =   0x00;
  heth.Init.RxMode = ETH_RXPOLLING_MODE;
  heth.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
  heth.Init.MediaInterface = ETH_MEDIA_INTERFACE_RMII;

  /* USER CODE BEGIN MACADDRESS */
    
  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 32000;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 50;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 32000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 200;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
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
  htim3.Init.Prescaler = 6400;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
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
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 64;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 6400;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 100;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */
  HAL_TIM_MspPostInit(&htim10);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, Status_LED1_Pin|Status_buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, angle_0_Pin|Status_LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, Status_LED_3_Pin|Status_LED_4_Pin|USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Status_LED1_Pin Status_buzzer_Pin */
  GPIO_InitStruct.Pin = Status_LED1_Pin|Status_buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : NotAus_Pin free_Button_2_Pin */
  GPIO_InitStruct.Pin = NotAus_Pin|free_Button_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Counterrisingedge_Pin */
  GPIO_InitStruct.Pin = Counterrisingedge_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Counterrisingedge_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : angle_0_Pin Status_LED2_Pin */
  GPIO_InitStruct.Pin = angle_0_Pin|Status_LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : Freigabe_Pin */
  GPIO_InitStruct.Pin = Freigabe_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Freigabe_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : angle_sync_Pin */
  GPIO_InitStruct.Pin = angle_sync_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(angle_sync_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : photodiode1_Pin photodiode2_Pin */
  GPIO_InitStruct.Pin = photodiode1_Pin|photodiode2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Status_LED_3_Pin Status_LED_4_Pin */
  GPIO_InitStruct.Pin = Status_LED_3_Pin|Status_LED_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
	void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
	/* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
  countspin++ ;
	}
	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);
	spins = 1/((256*0.2)/countspin) ;
	spinstrans = spins*100 ;
	countspin=0;
	
  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_PeriodElapsedCallback could be implemented in the user file
   */
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
	bluebufferval++;
	
  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_RxCpltCallback can be implemented in the user file.
   */
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
