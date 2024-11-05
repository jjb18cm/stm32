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
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "stdio.h"
#include "motor.h"
#include "niming.h"
#include "pid.h"
#include "bsp_dht11.h"

#include "cJSON.h"
#include <string.h>
#include "HC_SR04.h"



#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern float Motor1Speed ;//ÁîµÊú∫‰∏?ÈÄüÂ∫¶
extern float Motor2Speed ;//

extern tPid pidMotor1Speed;//
extern tPid pidMotor2Speed;
extern tPid pidFollow;    //∂®æ‡¿Î∏˙ÀÊPID
extern tPid pidMPU6050YawMovement;  //¿˚”√6050∆´∫ΩΩ« Ω¯––◊ÀÃ¨øÿ÷∆µƒPID
extern uint8_t Usart1_ReadBuf[255];	//‰∏≤Âè£1 ÁºìÂÜ≤Êï∞ÁªÑ
float p,i,d,a,b;//‰∏≤Âè£1 Êé•Êî∂Â≠óËäÇËÆ°Êï∞
uint8_t OledString[50];//OLEDœ‘ æ π”√µƒ◊÷∑˚¥Æ ˝◊È
extern float Mileage;//Ë°åÈ©∂Ë∑ùÁ¶ª
//Ê∏©ÊπøÂ∫¶
/* USER CODE BEGIN PD */
uint16_t temperature;
uint16_t humidity;
/* USER CODE END PD */
extern tPid pidHW_Tracking;//Á∫¢Â§ñÂØªËøπpid
uint8_t g_ucaHW_Read[4] = {0};//‰øùÂ≠òÁ∫¢Â§ñÂØπÁÆ°ÁîµÂπ≥Êï∞ÁªÑ
int8_t g_cThisState = 0;//ËøôÊ¨°Áä∂Ê??
int8_t g_cLastState = 0; //‰∏äÊ¨°Áä∂Ê??
float g_fHW_PID_Out;//Á∫¢Â§ñËÆ°ÁÆópidËæìÂá∫ÈÄüÂ∫¶
float g_fHW_PID_Out1;//ÁîµÊú∫‰∏?pidÊéßÂà∂ÈÄüÂ∫¶
float g_fHW_PID_Out2;//ÁîµÊú∫‰∫åpidÊéßÂà∂ÈÄüÂ∫¶

uint8_t g_ucUsart3ReceiveData;  //‰øùÂ≠ò‰∏≤Âè£‰∏âÊé•ÂèóÁöÑÊï∞ÊçÆ

uint8_t Usart3String[50];//¥Æø⁄»˝ ‰≥ˆ◊÷∑˚¥Æ π”√µƒ◊÷∑˚¥Æ ˝◊È
float g_fHC_SR04_Read;//Ë∂ÖÂ£∞Ê≥¢ËØªÂèñÁöÑÊï∞ÊçÆ
float g_fFollow_PID_Out;//∂®æ‡¿Î∏˙ÀÊPIDº∆À„ ‰≥ˆÀŸ∂»


float pitch,roll,yaw; //‰øØ‰ª∞Ëß? Ê®™ÊªöËß? Ëà™ÂêëËß?

float  g_fMPU6050YawMovePidOut = 0.00f; //pidËæìÂá∫ËøêÁÆó
float  g_fMPU6050YawMovePidOut1 = 0.00f; //ÁîµÊú∫ÊéßÂà∂ËæìÂá∫
float  g_fMPU6050YawMovePidOut2 = 0.00f; //ÁîµÊú∫ÊéßÂà∂ËæìÂá∫

uint8_t g_ucMode = 0; 
//Â∞èËΩ¶ËøêÂä®Ê®°ÂºèÊ†áÂøó‰Ω? 0:ÊòæÁ§∫ÂäüËÉΩ„Ä?1:PIDÂæ™ËøπÊ®°Âºè„Ä?2:ÊâãÊú∫ÈÅ•ÊéßÊôÆÈ?öËøêÂä®Ê®°Âºè„??3.Ë∂ÖÂ£∞Ê≥¢ÈÅøÈöúÊ®°Âºè„??4:PIDË∑üÈöèÊ®°Âºè„Ä?5:ÈÅ•ÊéßËßíÂ∫¶Èó≠ÁéØ
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_ADC2_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();			//≥ı ºªØOLED  
  OLED_Clear()  	; 
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);//ø™∆Ù∂® ±∆˜1 Õ®µ¿1 PWM ‰≥ˆ
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);//ø™∆Ù∂® ±∆˜1 Õ®µ¿4 PWM ‰≥ˆ
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);//ø™∆Ù∂® ±∆˜2
  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);//ø™∆Ù∂® ±∆˜4
  HAL_TIM_Base_Start_IT(&htim2);				//ø™∆Ù∂® ±∆˜2 ÷–∂œ
  HAL_TIM_Base_Start_IT(&htim4);                //ø™∆Ù∂® ±∆˜4 ÷–∂œ
  
  HAL_TIM_Base_Start_IT(&htim1);                //ø™∆Ù∂® ±∆˜1 ÷–∂œ
  __HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);	//ø™∆Ù¥Æø⁄1Ω” ’÷–∂œ
  PID_init();//PID≤Œ ˝≥ı ºªØ
  HAL_UART_Receive_IT(&huart3,&g_ucUsart3ReceiveData,1);  //¥Æø⁄»˝Ω” ’ ˝æ›
  
	//Ê∏©ÊπøÂ∫¶
//	DHT11_Data_TypeDef DHT11_Data;
	
  HAL_Delay(500);//Âª∂Êó∂
  MPU_Init(); //ÂàùÂßãÂå?6050
  while(MPU_Init()!=0);// 
  while(mpu_dmp_init()!=0);//mpu6050,

//  cJSON *cJsonData ,*cJsonVlaue;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
	{
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	sprintf((char *)OledString," g_ucMode:%d",g_ucMode);// ÊòæÁ§∫ÂΩìÂâçÊ®°Âºè
	OLED_ShowString(0,6,OledString,12);	//OLEDÊòæÁ§∫
	
	sprintf((char *)Usart3String," g_ucMode:%d",g_ucMode);//ËìùÁâôÊòæÁ§∫
	HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//◊Ë»˚ Ω∑¢ÀÕÕ®π˝¥Æø⁄»˝ ‰≥ˆ◊÷∑˚ strlen:º∆À„◊÷∑˚¥Æ¥Û–°
	
	if(g_ucMode == 0)
	{
	//OLEDÊòæÁ§∫ÂäüËÉΩ
		sprintf((char*)OledString, "V1:%.2fV2:%.2f", Motor1Speed,Motor2Speed);//ÈÄüÂ∫¶
		OLED_ShowString(0,0,OledString,12);//
		
		sprintf((char*)OledString, "Mileage:%.2f", Mileage);//ËøêÂä®ÈáåÁ®ã
		OLED_ShowString(0,1,OledString,12);
		
		sprintf((char*)OledString, "U:%.2fV", adcGetBatteryVoltage());//ÁîµÂéã
		OLED_ShowString(0,2,OledString,12);
		
		sprintf((char *)OledString,"HC_SR04:%.2fcm\r\n",HC_SR04_Read());//Ë∑ùÁ¶ªË∂ÖÂ£∞Ê≥?
		OLED_ShowString(0,3,OledString,12);
		
		sprintf((char *)OledString,"p:%.2f r:%.2f \r\n",pitch,roll);//Èô?Ëû∫‰ª™6050‰øØ‰ª∞Ëß? Ê®™ÊªöËß?
		OLED_ShowString(0,4,OledString,12);
		
		sprintf((char *)OledString,"y:%.2f  \r\n",yaw);//Èô?Ëû∫‰ª™6050‰øØ‰ª∞Ëß? Ê®™ÊªöËß?
		OLED_ShowString(0,5,OledString,12);
		DHT11_Read_Data(&temperature,&humidity);
		sprintf((char *)OledString,"t= %d.%d,h=%d.%d%%\r\n",temperature>>8,temperature&0xff,humidity>>8,humidity&0xff);//ÈôøËû∫‰ª™6050‰øØ‰ª∞Ëßø Ê®™ÊªöËßø
		OLED_ShowString(0,7,OledString,11);


		
	//ÁªàÁ´ØÊòæÁ§∫
		sprintf((char*)Usart3String, "V1:%.2fV2:%.2f", Motor1Speed,Motor2Speed);//ÈÄüÂ∫¶
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//ÈòªÂ°ûÂºèÂèëÈÄÅÈ?öËøá‰∏≤Âè£‰∏âËæìÂá∫Â≠óÁ¨? strlen:ËÆ°ÁÆóÂ≠óÁ¨¶‰∏≤Â§ßÂ∞?
		//ÈòªÂ°ûÊñπÂºèÂèëÈ?ÅÂèØ‰ª•‰øùËØÅÊï∞ÊçÆÂèëÈÄÅÂÆåÊØïÔºå‰∏≠Êñ≠ÂèëÈ?Å‰∏ç‰∏?ÂÆöÂèØ‰ª•‰øùËØÅÊï∞ÊçÆÂ∑≤ÁªèÂèëÈÄÅÂÆåÊØïÊâçÂêØÂä®‰∏ã‰∏ÄÊ¨°ÂèëÈÄ?
		sprintf((char*)Usart3String, "Mileage:%.2f", Mileage);//juË∑ùÁ¶ª
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);
		
		sprintf((char*)Usart3String, "U:%.2fV", adcGetBatteryVoltage());//ÁîµÂéã
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);
		
		sprintf((char *)Usart3String,"HC_SR04:%.2fcm\r\n",HC_SR04_Read());//Ë∂ÖÂ£∞Ê≥?
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);
		
		sprintf((char *)Usart3String,"p:%.2f r:%.2f \r\n",pitch,roll);//Èô?Ëû∫‰ª™6050‰øØ‰ª∞Ëß? Ê®™ÊªöËß?  
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);
		
		sprintf((char *)Usart3String,"y:%.2f  \r\n",yaw);//Ëà™ÂêëËß? 
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);
		DHT11_Read_Data(&temperature,&humidity);
	sprintf((char*)Usart3String, "t= %d.%d,h=%d.%d%%\r\n",temperature>>8,temperature&0xff,humidity>>8,humidity&0xff);//ÈÄüÂ∫¶
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);
		//Èô?Ëû∫‰ª™6050Êï∞ÊçÆ
		while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){}//Ëøô‰∏™ÂèØ‰ª•Ëß£ÂÜ≥ÁªèÂ∏∏ËØª‰∏çÂá∫Êï∞ÊçÆÁöÑÈóÆÈ¢ò
		
		//ÁîµÊú∫ÂÅúÊ≠¢
		motorPidSetSpeed(0,0);
	}
	if(g_ucMode == 1)
	{
	///****    Á∫¢Â§ñÂØªËøπpid******************/
	g_ucaHW_Read[0] = READ_HW_OUT_1;//Áõ¥Êé•ËØªÂèñÁ∫¢Â§ñÁä∂Ê?ÅÊõ¥Âø?
	g_ucaHW_Read[1] = READ_HW_OUT_2;
	g_ucaHW_Read[2] = READ_HW_OUT_3;
	g_ucaHW_Read[3] = READ_HW_OUT_4;

	if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )
	{
//		printf("1\r\n");//
		g_cThisState = 0;
	}
	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 1&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )
	{
//		printf("\r\n");
		g_cThisState = -1;
	}
	else if(g_ucaHW_Read[0] == 1&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )
	{
//		printf("\r\n");
		g_cThisState = -2;
	}
	else if(g_ucaHW_Read[0] == 1&&g_ucaHW_Read[1] == 1&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0)
	{
//		printf("\r\n");
		g_cThisState = -3;
	}
	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 1&&g_ucaHW_Read[3] == 0 )
	{
//		printf("\r\n");
		g_cThisState = 1;	
	}
	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 1 )
	{
//		printf("\r\n");
		g_cThisState = 2;
	}
	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 1&&g_ucaHW_Read[3] == 1)
	{
//	    printf("\r\n");
		g_cThisState = 3;//
	}
	g_fHW_PID_Out = PID_realize(&pidHW_Tracking,g_cThisState);///PIDËÆ°ÁÆóËæìÂá∫ÁõÆÊ†áÈÄüÂ∫¶Ëøô‰∏™ÈÄüÂ∫¶Ôºå‰ºöÂíåÂü∫Á°?ÈÄüÂ∫¶Âä†Âáè 

	g_fHW_PID_Out1 = 3 + g_fHW_PID_Out;//ÁîµÊú∫1ÈÄüÂ∫¶=Âü∫Á°ÄÈÄüÂ∫¶+Âæ™ËøπPIDËæìÂá∫ÈÄüÂ∫¶
	g_fHW_PID_Out2 = 3 - g_fHW_PID_Out;//ÁîµÊú∫1ÈÄüÂ∫¶=Âü∫Á°ÄÈÄüÂ∫¶-Âæ™ËøπPIDËæìÂá∫ÈÄüÂ∫¶
	if(g_fHW_PID_Out1 >5) g_fHW_PID_Out1 =5;///ËøõË°åÈôêÂπÖ ÈôêÂπÖÈÄüÂ∫¶Âú?0-5‰πãÈó¥
	if(g_fHW_PID_Out1 <0) g_fHW_PID_Out1 =0;
	if(g_fHW_PID_Out2 >5) g_fHW_PID_Out2 =5;//
	if(g_fHW_PID_Out2 <0) g_fHW_PID_Out2 =0;
	if(g_cThisState != g_cLastState)//Â¶Ç‰ΩïËøôÊ¨°Áä∂Ê?Å‰∏çÁ≠â‰∫é‰∏äÊ¨°Áä∂Ê?Å„?ÅÂ∞±ËøõË°åÊîπÂèòÁõÆÊ†áÈÄüÂ∫¶ÂíåÊéßÂà∂ÁîµÊú∫„?ÅÂú®ÂÆöÊó∂Âô®‰∏≠‰æùÊóßÂÆöÊó∂ÊéßÂà∂ÁîµÊú∫
	{
		motorPidSetSpeed(g_fHW_PID_Out1,g_fHW_PID_Out2);///ÈÄöËøáËÆ°ÁÆóÁöÑÈ?üÂ∫¶ÊéßÂà∂ÁîµÊú∫
	}
	
	g_cLastState = g_cThisState;///‰øùÂ≠ò‰∏äÊ¨°Á∫¢Â§ñÂØπÁÆ°Áä∂Ê??	

	}
	if(g_ucMode == 2)
	{
		//***************ÈÅ•Êéß***********************//
		//ÈÅ•ÊéßÊ®°ÂºèÁöÑÊéßÂà∂Âú®‰∏≤Âè£‰∏âÁöÑ‰∏≠Êñ≠ÈáåÈù¢
}
	}
	if(g_ucMode == 3)
	{
		//************Ë∂ÖÂ£∞Ê≥¢***************//
//ÈÅøÈöúÈÄªËæë
		if(HC_SR04_Read() > 25)//25ÂéòÁ±≥ÂâçÊ≤°Êúâ‰∏ú
		{
			motorPidSetSpeed(1,1);//ÂâçËøõ
			HAL_Delay(100);
		}
		else{	//Êúâ?
			motorPidSetSpeed(-1,1);//Â∑¶ ÂéüÂú∞	
			HAL_Delay(500);
			if(HC_SR04_Read() > 25)//Êó?
			{
				motorPidSetSpeed(1,1);//Ââ?
				HAL_Delay(100);
			}
			else{//Êú?
				motorPidSetSpeed(1,-1);//Â∑? ÂéüÂú∞
				HAL_Delay(1000);
				if(HC_SR04_Read() >25)//Êó?
				{
					 motorPidSetSpeed(1,1);//Ââ?
					HAL_Delay(100);
				}
				else{
					motorPidSetSpeed(-1,-1);//ÂêéÈ??
					HAL_Delay(1000);
					motorPidSetSpeed(-1,1);//Âè? ÂéüÂú∞
					HAL_Delay(50);
				}
			}
		}
	}
	if(g_ucMode == 4)
	{
	//**********pidË∑üÈöè***********//
		g_fHC_SR04_Read=HC_SR04_Read();//ËØªÂèñÂâçÊñπË∑ùÁ¶ª
		if(g_fHC_SR04_Read < 60){  // 60ÂéòÁ±≥Êúâ‰∏úË•?
			g_fFollow_PID_Out = PID_realize(&pidFollow,g_fHC_SR04_Read);//pid ËÆ°ÁÆóÈÄüÂ∫¶ÂíåÂü∫Á°?ÈÄüÂ∫¶Âä†Âáè
			if(g_fFollow_PID_Out > 3) g_fFollow_PID_Out = 3;//Êú?Â§ßÈ?üÂ∫¶Â∞è‰∫éÂÖ?
			if(g_fFollow_PID_Out < -3) g_fFollow_PID_Out = -3;
			motorPidSetSpeed(g_fFollow_PID_Out,g_fFollow_PID_Out);//ËøêÂä®
		}
		else motorPidSetSpeed(0,0);//60ÂéòÁ±≥Êó? ÂÅ?
		HAL_Delay(10);//Âä†ÂÖ•Âª∂Êó∂
	}
	if(g_ucMode == 5)
	{
	//*************Ëû∫‰ª™ pidËΩ¨Âêë*****************//

		sprintf((char *)Usart3String,"pitch:%.2f roll:%.2f yaw:%.2f\r\n",pitch,roll,yaw);//ÊòæÁ§∫Èô?Ëû∫‰ª™Êï∞ÊçÆ
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),0xFFFF);//	

		while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){}  //ËØª‰∏çÂà∞Êï∞Êç?
		
		
		g_fMPU6050YawMovePidOut = PID_realize(&pidMPU6050YawMovement,yaw);//ËÆ°ÁÆóËæìÂá∫ÈÄüÂ∫¶

		g_fMPU6050YawMovePidOut1 = 1.5 + g_fMPU6050YawMovePidOut;//
		g_fMPU6050YawMovePidOut2 = 1.5 - g_fMPU6050YawMovePidOut;
		if(g_fMPU6050YawMovePidOut1 >3.5) g_fMPU6050YawMovePidOut1 =3.5;//ÈôêÈ??
		if(g_fMPU6050YawMovePidOut1 <0) g_fMPU6050YawMovePidOut1 =0;
		if(g_fMPU6050YawMovePidOut2 >3.5) g_fMPU6050YawMovePidOut2 =3.5;//
		if(g_fMPU6050YawMovePidOut2 <0) g_fMPU6050YawMovePidOut2 =0;
		motorPidSetSpeed(g_fMPU6050YawMovePidOut1,g_fMPU6050YawMovePidOut2); 
	
	}
	
///****    ∫ÏÕ‚PID—≠º£π¶ƒ‹******************/
//	g_ucaHW_Read[0] = READ_HW_OUT_1;//∂¡»°∫ÏÕ‚∂‘π‹◊¥Ã¨°¢’‚—˘œ‡±»”⁄–¥‘⁄if¿Ô√Ê∏¸∏ﬂ–ß
//	g_ucaHW_Read[1] = READ_HW_OUT_2;
//	g_ucaHW_Read[2] = READ_HW_OUT_3;
//	g_ucaHW_Read[3] = READ_HW_OUT_4;

//	if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )
//	{
////		printf("”¶∏√«∞Ω¯\r\n");//◊¢ ÕµÙ∏¸º”∏ﬂ–ß£¨ºı…ŸŒﬁ±ÿ“™≥Ã–Ú÷¥––
//		g_cThisState = 0;//«∞Ω¯
//	}
//	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 1&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )// π”√else if∏¸º”∫œ¿Ì∏ﬂ–ß
//	{
////		printf("”¶∏√”“◊™\r\n");
//		g_cThisState = -1;//”¶∏√”“◊™
//	}
//	else if(g_ucaHW_Read[0] == 1&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )
//	{
////		printf("øÏÀŸ”“◊™\r\n");
//		g_cThisState = -2;//øÏÀŸ”“◊™
//	}
//	else if(g_ucaHW_Read[0] == 1&&g_ucaHW_Read[1] == 1&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0)
//	{
////		printf("øÏÀŸ”“◊™\r\n");
//		g_cThisState = -3;//øÏÀŸ”“◊™
//	}
//	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 1&&g_ucaHW_Read[3] == 0 )
//	{
////		printf("”¶∏√◊Û◊™\r\n");
//		g_cThisState = 1;//”¶∏√◊Û◊™	
//	}
//	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 1 )
//	{
////		printf("øÏÀŸ◊Û◊™\r\n");
//		g_cThisState = 2;//øÏÀŸ◊Û◊™
//	}
//	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 1&&g_ucaHW_Read[3] == 1)
//	{
////	    printf("øÏÀŸ◊Û◊™\r\n");
//		g_cThisState = 3;//øÏÀŸ◊Û◊™
//	}
//	g_fHW_PID_Out = PID_realize(&pidHW_Tracking,g_cThisState);//PIDº∆À„ ‰≥ˆƒø±ÍÀŸ∂» ’‚∏ˆÀŸ∂»£¨ª·∫Õª˘¥°ÀŸ∂»º”ºı

//	g_fHW_PID_Out1 = 3 + g_fHW_PID_Out;//µÁª˙1ÀŸ∂»=ª˘¥°ÀŸ∂»+—≠º£PID ‰≥ˆÀŸ∂»
//	g_fHW_PID_Out2 = 3 - g_fHW_PID_Out;//µÁª˙1ÀŸ∂»=ª˘¥°ÀŸ∂»-—≠º£PID ‰≥ˆÀŸ∂»
//	if(g_fHW_PID_Out1 >5) g_fHW_PID_Out1 =5;//Ω¯––œﬁ∑˘ œﬁ∑˘ÀŸ∂»‘⁄0-5÷Æº‰
//	if(g_fHW_PID_Out1 <0) g_fHW_PID_Out1 =0;
//	if(g_fHW_PID_Out2 >5) g_fHW_PID_Out2 =5;
//	if(g_fHW_PID_Out2 <0) g_fHW_PID_Out2 =0;
//	if(g_cThisState != g_cLastState)//»Á∫Œ’‚¥Œ◊¥Ã¨≤ªµ»”⁄…œ¥Œ◊¥Ã¨°¢æÕΩ¯––∏ƒ±‰ƒø±ÍÀŸ∂»∫Õøÿ÷∆µÁª˙°¢‘⁄∂® ±∆˜÷–“¿æ…∂® ±øÿ÷∆µÁª˙
//	{
//		motorPidSetSpeed(g_fHW_PID_Out1,g_fHW_PID_Out2);//Õ®π˝º∆À„µƒÀŸ∂»øÿ÷∆µÁª˙
//	}
//	
//	g_cLastState = g_cThisState;//±£¥Ê…œ¥Œ∫ÏÕ‚∂‘π‹◊¥Ã¨	



////Õ®π˝¥Æø⁄»˝(¿∂—¿) ‰≥ˆ–≈œ¢
////***************¥Æø⁄»˝ ‰≥ˆ****************************//
//	sprintf((char *)Usart3String,"V1:%.2fV2:%.2f\r\n",Motor1Speed,Motor2Speed);//œ‘ æ¡Ω∏ˆµÁª˙◊™ÀŸ µ•Œª£∫◊™/√Î
//	HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//◊Ë»˚ Ω∑¢ÀÕÕ®π˝¥Æø⁄»˝ ‰≥ˆ◊÷∑˚ strlen:º∆À„◊÷∑˚¥Æ¥Û–°
//	
//	sprintf((char *)Usart3String,"Mileage%.2f\r\n",Mileage);//º∆À„–°≥µ¿Ô≥Ã µ•Œªcm
//	HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//◊Ë»˚ Ω∑¢ÀÕÕ®π˝¥Æø⁄»˝ ‰≥ˆ◊÷∑˚ strlen:º∆À„◊÷∑˚¥Æ¥Û–°
//	
//	sprintf((char *)Usart3String,"U:%.2fV\r\n",adcGetBatteryVoltage());//œ‘ æµÁ≥ÿµÁ—π
//	HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//◊Ë»˚ Ω∑¢ÀÕÕ®π˝¥Æø⁄»˝ ‰≥ˆ◊÷∑˚ strlen:º∆À„◊÷∑˚¥Æ¥Û–°	
//	
//	sprintf((char *)Usart3String,"HC_SR04:%.2fcm\r\n",HC_SR04_Read());//œ‘ æ≥¨…˘≤® ˝æ›
//	HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),0xFFFF);//Õ®π˝¥Æø⁄»˝ ‰≥ˆ◊÷∑˚ strlen:º∆À„◊÷∑˚¥Æ¥Û–°	
//	
//   	sprintf((char *)Usart3String,"pitch:%.2f roll:%.2f yaw:%.2f\r\n",pitch,roll,yaw);//œ‘ æ6050 ˝æ› ∏©—ˆΩ« ∫·πˆΩ« ∫ΩœÚΩ«
//	HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),0xFFFF);//Õ®π˝¥Æø⁄»˝ ‰≥ˆ◊÷∑˚ strlen:º∆À„◊÷∑˚¥Æ¥Û–°	
//   
//   //mpu_dmp_get_data(&pitch,&roll,&yaw);//∑µªÿ÷µ:0,DMP≥…π¶Ω‚≥ˆ≈∑¿≠Ω«
//    while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){}  //’‚∏ˆø…“‘Ω‚æˆæ≠≥£∂¡≤ª≥ˆ ˝æ›µƒŒ Ã‚
//	
//	
//	HAL_Delay(5);//◊¢“‚µ˜”√≤ªø…“‘π˝”⁄∆µ∑±HC_SR04_Read()

////*************MPU6050∫ΩœÚΩ« PID◊™œÚøÿ÷∆*****************//

//   	sprintf((char *)Usart3String,"pitch:%.2f roll:%.2f yaw:%.2f\r\n",pitch,roll,yaw);//œ‘ æ6050 ˝æ› ∏©—ˆΩ« ∫·πˆΩ« ∫ΩœÚΩ«
//	HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),0xFFFF);//Õ®π˝¥Æø⁄»˝ ‰≥ˆ◊÷∑˚ strlen:º∆À„◊÷∑˚¥Æ¥Û–°	
//   
//   //mpu_dmp_get_data(&pitch,&roll,&yaw);//∑µªÿ÷µ:0,DMP≥…π¶Ω‚≥ˆ≈∑¿≠Ω«
//    while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){}  //’‚∏ˆø…“‘Ω‚æˆæ≠≥£∂¡≤ª≥ˆ ˝æ›µƒŒ Ã‚
//	
//	
//	g_fMPU6050YawMovePidOut = PID_realize(&pidMPU6050YawMovement,yaw);//PIDº∆À„ ‰≥ˆƒø±ÍÀŸ∂» ’‚∏ˆÀŸ∂»£¨ª·∫Õª˘¥°ÀŸ∂»º”ºı

//	g_fMPU6050YawMovePidOut1 = 1.5 + g_fMPU6050YawMovePidOut;//ª˘¥°ÀŸ∂»º”ºıPID ‰≥ˆÀŸ∂»
//	g_fMPU6050YawMovePidOut2 = 1.5 - g_fMPU6050YawMovePidOut;
//	if(g_fMPU6050YawMovePidOut1 >3.5) g_fMPU6050YawMovePidOut1 =3.5;//Ω¯––œﬁ∑˘
//	if(g_fMPU6050YawMovePidOut1 <0) g_fMPU6050YawMovePidOut1 =0;
//	if(g_fMPU6050YawMovePidOut2 >3.5) g_fMPU6050YawMovePidOut2 =3.5;
//	if(g_fMPU6050YawMovePidOut2 <0) g_fMPU6050YawMovePidOut2 =0;
//	motorPidSetSpeed(g_fMPU6050YawMovePidOut1,g_fMPU6050YawMovePidOut2);


////**************±‹’œπ¶ƒ‹********************//
////±‹’œ¬ﬂº≠
//	if(HC_SR04_Read() > 25)//«∞∑ΩŒﬁ’œ∞≠ŒÔ
//	{
//		motorPidSetSpeed(1,1);//«∞‘À∂Ø
//		HAL_Delay(100);
//	}
//	else{	//«∞∑Ω”–’œ∞≠ŒÔ
//		motorPidSetSpeed(-1,1);//”“±ﬂ‘À∂Ø ‘≠µÿ	
//		HAL_Delay(500);
//		if(HC_SR04_Read() > 25)//”“±ﬂŒﬁ’œ∞≠ŒÔ
//		{
//			motorPidSetSpeed(1,1);//«∞‘À∂Ø
//			HAL_Delay(100);
//		}
//		else{//”“±ﬂ”–’œ∞≠ŒÔ
//			motorPidSetSpeed(1,-1);//◊Û±ﬂ‘À∂Ø ‘≠µÿ
//			HAL_Delay(1000);
//			if(HC_SR04_Read() >25)//◊Û±ﬂŒﬁ’œ∞≠ŒÔ
//			{
//				 motorPidSetSpeed(1,1);//«∞‘À∂Ø
//				HAL_Delay(100);
//			}
//			else{
//				motorPidSetSpeed(-1,-1);//∫Û‘À∂Ø
//				HAL_Delay(1000);
//				motorPidSetSpeed(-1,1);//”“±ﬂ‘À∂Ø
//				HAL_Delay(50);
//			}
//		}
//	}


////*************ŒﬁPID∏˙ÀÊπ¶ƒ‹************//
//	if(HC_SR04_Read() > 25)
//	{
//		motorPidSetSpeed(1,1);//«∞‘À∂Ø
//		HAL_Delay(100);
//	}
//	if(HC_SR04_Read() < 20)
//	{
//		motorPidSetSpeed(-1,-1);//∫Û‘À∂Ø
//		HAL_Delay(100);
//	}

////**********PID∏˙ÀÊπ¶ƒ‹***********//
//    g_fHC_SR04_Read=HC_SR04_Read();//∂¡»°«∞∑Ω’œ∞≠ŒÔæ‡¿Î
//	if(g_fHC_SR04_Read < 60){  //»Áπ˚«∞60cm ”–∂´Œ˜æÕ∆Ù∂Ø∏˙ÀÊ
//		g_fFollow_PID_Out = PID_realize(&pidFollow,g_fHC_SR04_Read);//PIDº∆À„ ‰≥ˆƒø±ÍÀŸ∂» ’‚∏ˆÀŸ∂»£¨ª·∫Õª˘¥°ÀŸ∂»º”ºı
//		if(g_fFollow_PID_Out > 6) g_fFollow_PID_Out = 6;//∂‘ ‰≥ˆÀŸ∂»œﬁ∑˘
//		if(g_fFollow_PID_Out < -6) g_fFollow_PID_Out = -6;
//		motorPidSetSpeed(g_fFollow_PID_Out,g_fFollow_PID_Out);//ÀŸ∂»◊˜”√”ÎµÁª˙…œ
//	}
//	else motorPidSetSpeed(0,0);//»Áπ˚«∞√Ê60cm √ª”–∂´Œ˜æÕÕ£÷π
//	HAL_Delay(10);//∂¡»°≥¨…˘≤®¥´∏–∆˜≤ªƒ‹π˝øÏ

//	ANO_DT_Send_F2(Motor1Speed*100, 3.0*100,Motor2Speed*100,3.0*100);//œÚƒ‰√˚…œŒªª˙Õ®π˝F2÷°∑¢ÀÕ4∏ˆint16¿‡–Õµƒ ˝æ› «˙œﬂœ‘ æ
//	if(Usart_WaitReasFinish() == 0)// «∑ÒΩ” ’ÕÍ±œ
//	{
//		cJsonData  = cJSON_Parse((const char *)Usart1_ReadBuf);
//		if(cJSON_GetObjectItem(cJsonData,"p") !=NULL)
//		{
//			cJsonVlaue = cJSON_GetObjectItem(cJsonData,"p");	
//		    p = cJsonVlaue->valuedouble;
//			pidMotor1Speed.Kp = p;
//		}
//		if(cJSON_GetObjectItem(cJsonData,"i") !=NULL)
//		{
//			cJsonVlaue = cJSON_GetObjectItem(cJsonData,"i");	
//		    i = cJsonVlaue->valuedouble;
//			pidMotor1Speed.Ki = i;
//		}
//		if(cJSON_GetObjectItem(cJsonData,"d") !=NULL)
//		{
//			cJsonVlaue = cJSON_GetObjectItem(cJsonData,"d");	
//		    d = cJsonVlaue->valuedouble;
//			pidMotor1Speed.Kd = d;
//		}
//		if(cJSON_GetObjectItem(cJsonData,"a") !=NULL)
//		{
//		
//			cJsonVlaue = cJSON_GetObjectItem(cJsonData,"a");	
//		    a = cJsonVlaue->valuedouble;
//			pidMotor1Speed.target_val =a;
//		}
//		if(cJSON_GetObjectItem(cJsonData,"b") !=NULL)
//		{
//		
//			cJsonVlaue = cJSON_GetObjectItem(cJsonData,"b");	
//		    b = cJsonVlaue->valuedouble;
//			pidMotor2Speed.target_val =b;
//		}
//		if(cJsonData != NULL){
//		  cJSON_Delete(cJsonData);// Õ∑≈ø’º‰°¢µ´ «≤ªƒ‹…æ≥˝cJsonVlaue≤ª»ªª· ≥ˆœ÷“Ï≥£¥ÌŒÛ
//		}
//		memset(Usart1_ReadBuf,0,255);//«Âø’Ω” ’buf£¨◊¢“‚’‚¿Ô≤ªƒ‹ π”√strlen	
//	}
//	printf("P:%.3f  I:%.3f  D:%.3f A:%.3f\r\n",p,i,d,a);
	
	
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 10);
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 10);
//	HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_SET);
//	HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_SET);
	
	
//	uint8_t c_Data[] = "¥Æø⁄ ‰≥ˆ≤‚ ‘:∫√º“ªÔVCC\r\n";
//	HAL_UART_Transmit(&huart1,c_Data,sizeof(c_Data),0xFFFF);
//	HAL_Delay(1000);
//	printf("printf:∫√º“ªÔVCC≤‚ ‘\r\n");
//	HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
//	HAL_Delay(500);
	
	
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
