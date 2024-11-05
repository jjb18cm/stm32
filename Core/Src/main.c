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
extern float Motor1Speed ;//电机�?速度
extern float Motor2Speed ;//

extern tPid pidMotor1Speed;//
extern tPid pidMotor2Speed;
extern tPid pidFollow;    //���������PID
extern tPid pidMPU6050YawMovement;  //����6050ƫ���� ������̬���Ƶ�PID
extern uint8_t Usart1_ReadBuf[255];	//串口1 缓冲数组
float p,i,d,a,b;//串口1 接收字节计数
uint8_t OledString[50];//OLED��ʾʹ�õ��ַ�������
extern float Mileage;//行驶距离
//温湿度
/* USER CODE BEGIN PD */
uint16_t temperature;
uint16_t humidity;
/* USER CODE END PD */
extern tPid pidHW_Tracking;//红外寻迹pid
uint8_t g_ucaHW_Read[4] = {0};//保存红外对管电平数组
int8_t g_cThisState = 0;//这次状�??
int8_t g_cLastState = 0; //上次状�??
float g_fHW_PID_Out;//红外计算pid输出速度
float g_fHW_PID_Out1;//电机�?pid控制速度
float g_fHW_PID_Out2;//电机二pid控制速度

uint8_t g_ucUsart3ReceiveData;  //保存串口三接受的数据

uint8_t Usart3String[50];//����������ַ���ʹ�õ��ַ�������
float g_fHC_SR04_Read;//超声波读取的数据
float g_fFollow_PID_Out;//���������PID��������ٶ�


float pitch,roll,yaw; //俯仰�? 横滚�? 航向�?

float  g_fMPU6050YawMovePidOut = 0.00f; //pid输出运算
float  g_fMPU6050YawMovePidOut1 = 0.00f; //电机控制输出
float  g_fMPU6050YawMovePidOut2 = 0.00f; //电机控制输出

uint8_t g_ucMode = 0; 
//小车运动模式标志�? 0:显示功能�?1:PID循迹模式�?2:手机遥控普�?�运动模式�??3.超声波避障模式�??4:PID跟随模式�?5:遥控角度闭环
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
  OLED_Init();			//��ʼ��OLED  
  OLED_Clear()  	; 
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);//������ʱ��1 ͨ��1 PWM���
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);//������ʱ��1 ͨ��4 PWM���
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);//������ʱ��2
  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);//������ʱ��4
  HAL_TIM_Base_Start_IT(&htim2);				//������ʱ��2 �ж�
  HAL_TIM_Base_Start_IT(&htim4);                //������ʱ��4 �ж�
  
  HAL_TIM_Base_Start_IT(&htim1);                //������ʱ��1 �ж�
  __HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);	//��������1�����ж�
  PID_init();//PID������ʼ��
  HAL_UART_Receive_IT(&huart3,&g_ucUsart3ReceiveData,1);  //��������������
  
	//温湿度
//	DHT11_Data_TypeDef DHT11_Data;
	
  HAL_Delay(500);//延时
  MPU_Init(); //初始�?6050
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
	sprintf((char *)OledString," g_ucMode:%d",g_ucMode);// 显示当前模式
	OLED_ShowString(0,6,OledString,12);	//OLED显示
	
	sprintf((char *)Usart3String," g_ucMode:%d",g_ucMode);//蓝牙显示
	HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//����ʽ����ͨ������������ַ� strlen:�����ַ�����С
	
	if(g_ucMode == 0)
	{
	//OLED显示功能
		sprintf((char*)OledString, "V1:%.2fV2:%.2f", Motor1Speed,Motor2Speed);//速度
		OLED_ShowString(0,0,OledString,12);//
		
		sprintf((char*)OledString, "Mileage:%.2f", Mileage);//运动里程
		OLED_ShowString(0,1,OledString,12);
		
		sprintf((char*)OledString, "U:%.2fV", adcGetBatteryVoltage());//电压
		OLED_ShowString(0,2,OledString,12);
		
		sprintf((char *)OledString,"HC_SR04:%.2fcm\r\n",HC_SR04_Read());//距离超声�?
		OLED_ShowString(0,3,OledString,12);
		
		sprintf((char *)OledString,"p:%.2f r:%.2f \r\n",pitch,roll);//�?螺仪6050俯仰�? 横滚�?
		OLED_ShowString(0,4,OledString,12);
		
		sprintf((char *)OledString,"y:%.2f  \r\n",yaw);//�?螺仪6050俯仰�? 横滚�?
		OLED_ShowString(0,5,OledString,12);
		DHT11_Read_Data(&temperature,&humidity);
		sprintf((char *)OledString,"t= %d.%d,h=%d.%d%%\r\n",temperature>>8,temperature&0xff,humidity>>8,humidity&0xff);//陿螺仪6050俯仰觿 横滚觿
		OLED_ShowString(0,7,OledString,11);


		
	//终端显示
		sprintf((char*)Usart3String, "V1:%.2fV2:%.2f", Motor1Speed,Motor2Speed);//速度
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//阻塞式发送�?�过串口三输出字�? strlen:计算字符串大�?
		//阻塞方式发�?�可以保证数据发送完毕，中断发�?�不�?定可以保证数据已经发送完毕才启动下一次发�?
		sprintf((char*)Usart3String, "Mileage:%.2f", Mileage);//ju距离
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);
		
		sprintf((char*)Usart3String, "U:%.2fV", adcGetBatteryVoltage());//电压
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);
		
		sprintf((char *)Usart3String,"HC_SR04:%.2fcm\r\n",HC_SR04_Read());//超声�?
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);
		
		sprintf((char *)Usart3String,"p:%.2f r:%.2f \r\n",pitch,roll);//�?螺仪6050俯仰�? 横滚�?  
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);
		
		sprintf((char *)Usart3String,"y:%.2f  \r\n",yaw);//航向�? 
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);
		DHT11_Read_Data(&temperature,&humidity);
	sprintf((char*)Usart3String, "t= %d.%d,h=%d.%d%%\r\n",temperature>>8,temperature&0xff,humidity>>8,humidity&0xff);//速度
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);
		//�?螺仪6050数据
		while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){}//这个可以解决经常读不出数据的问题
		
		//电机停止
		motorPidSetSpeed(0,0);
	}
	if(g_ucMode == 1)
	{
	///****    红外寻迹pid******************/
	g_ucaHW_Read[0] = READ_HW_OUT_1;//直接读取红外状�?�更�?
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
	g_fHW_PID_Out = PID_realize(&pidHW_Tracking,g_cThisState);///PID计算输出目标速度这个速度，会和基�?速度加减 

	g_fHW_PID_Out1 = 3 + g_fHW_PID_Out;//电机1速度=基础速度+循迹PID输出速度
	g_fHW_PID_Out2 = 3 - g_fHW_PID_Out;//电机1速度=基础速度-循迹PID输出速度
	if(g_fHW_PID_Out1 >5) g_fHW_PID_Out1 =5;///进行限幅 限幅速度�?0-5之间
	if(g_fHW_PID_Out1 <0) g_fHW_PID_Out1 =0;
	if(g_fHW_PID_Out2 >5) g_fHW_PID_Out2 =5;//
	if(g_fHW_PID_Out2 <0) g_fHW_PID_Out2 =0;
	if(g_cThisState != g_cLastState)//如何这次状�?�不等于上次状�?��?�就进行改变目标速度和控制电机�?�在定时器中依旧定时控制电机
	{
		motorPidSetSpeed(g_fHW_PID_Out1,g_fHW_PID_Out2);///通过计算的�?�度控制电机
	}
	
	g_cLastState = g_cThisState;///保存上次红外对管状�??	

	}
	if(g_ucMode == 2)
	{
		//***************遥控***********************//
		//遥控模式的控制在串口三的中断里面
}
	}
	if(g_ucMode == 3)
	{
		//************超声波***************//
//避障逻辑
		if(HC_SR04_Read() > 25)//25厘米前没有东
		{
			motorPidSetSpeed(1,1);//前进
			HAL_Delay(100);
		}
		else{	//有?
			motorPidSetSpeed(-1,1);//左 原地	
			HAL_Delay(500);
			if(HC_SR04_Read() > 25)//�?
			{
				motorPidSetSpeed(1,1);//�?
				HAL_Delay(100);
			}
			else{//�?
				motorPidSetSpeed(1,-1);//�? 原地
				HAL_Delay(1000);
				if(HC_SR04_Read() >25)//�?
				{
					 motorPidSetSpeed(1,1);//�?
					HAL_Delay(100);
				}
				else{
					motorPidSetSpeed(-1,-1);//后�??
					HAL_Delay(1000);
					motorPidSetSpeed(-1,1);//�? 原地
					HAL_Delay(50);
				}
			}
		}
	}
	if(g_ucMode == 4)
	{
	//**********pid跟随***********//
		g_fHC_SR04_Read=HC_SR04_Read();//读取前方距离
		if(g_fHC_SR04_Read < 60){  // 60厘米有东�?
			g_fFollow_PID_Out = PID_realize(&pidFollow,g_fHC_SR04_Read);//pid 计算速度和基�?速度加减
			if(g_fFollow_PID_Out > 3) g_fFollow_PID_Out = 3;//�?大�?�度小于�?
			if(g_fFollow_PID_Out < -3) g_fFollow_PID_Out = -3;
			motorPidSetSpeed(g_fFollow_PID_Out,g_fFollow_PID_Out);//运动
		}
		else motorPidSetSpeed(0,0);//60厘米�? �?
		HAL_Delay(10);//加入延时
	}
	if(g_ucMode == 5)
	{
	//*************螺仪 pid转向*****************//

		sprintf((char *)Usart3String,"pitch:%.2f roll:%.2f yaw:%.2f\r\n",pitch,roll,yaw);//显示�?螺仪数据
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),0xFFFF);//	

		while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){}  //读不到数�?
		
		
		g_fMPU6050YawMovePidOut = PID_realize(&pidMPU6050YawMovement,yaw);//计算输出速度

		g_fMPU6050YawMovePidOut1 = 1.5 + g_fMPU6050YawMovePidOut;//
		g_fMPU6050YawMovePidOut2 = 1.5 - g_fMPU6050YawMovePidOut;
		if(g_fMPU6050YawMovePidOut1 >3.5) g_fMPU6050YawMovePidOut1 =3.5;//限�??
		if(g_fMPU6050YawMovePidOut1 <0) g_fMPU6050YawMovePidOut1 =0;
		if(g_fMPU6050YawMovePidOut2 >3.5) g_fMPU6050YawMovePidOut2 =3.5;//
		if(g_fMPU6050YawMovePidOut2 <0) g_fMPU6050YawMovePidOut2 =0;
		motorPidSetSpeed(g_fMPU6050YawMovePidOut1,g_fMPU6050YawMovePidOut2); 
	
	}
	
///****    ����PIDѭ������******************/
//	g_ucaHW_Read[0] = READ_HW_OUT_1;//��ȡ����Թ�״̬�����������д��if�������Ч
//	g_ucaHW_Read[1] = READ_HW_OUT_2;
//	g_ucaHW_Read[2] = READ_HW_OUT_3;
//	g_ucaHW_Read[3] = READ_HW_OUT_4;

//	if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )
//	{
////		printf("Ӧ��ǰ��\r\n");//ע�͵����Ӹ�Ч�������ޱ�Ҫ����ִ��
//		g_cThisState = 0;//ǰ��
//	}
//	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 1&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )//ʹ��else if���Ӻ����Ч
//	{
////		printf("Ӧ����ת\r\n");
//		g_cThisState = -1;//Ӧ����ת
//	}
//	else if(g_ucaHW_Read[0] == 1&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )
//	{
////		printf("������ת\r\n");
//		g_cThisState = -2;//������ת
//	}
//	else if(g_ucaHW_Read[0] == 1&&g_ucaHW_Read[1] == 1&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0)
//	{
////		printf("������ת\r\n");
//		g_cThisState = -3;//������ת
//	}
//	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 1&&g_ucaHW_Read[3] == 0 )
//	{
////		printf("Ӧ����ת\r\n");
//		g_cThisState = 1;//Ӧ����ת	
//	}
//	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 1 )
//	{
////		printf("������ת\r\n");
//		g_cThisState = 2;//������ת
//	}
//	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 1&&g_ucaHW_Read[3] == 1)
//	{
////	    printf("������ת\r\n");
//		g_cThisState = 3;//������ת
//	}
//	g_fHW_PID_Out = PID_realize(&pidHW_Tracking,g_cThisState);//PID�������Ŀ���ٶ� ����ٶȣ���ͻ����ٶȼӼ�

//	g_fHW_PID_Out1 = 3 + g_fHW_PID_Out;//���1�ٶ�=�����ٶ�+ѭ��PID����ٶ�
//	g_fHW_PID_Out2 = 3 - g_fHW_PID_Out;//���1�ٶ�=�����ٶ�-ѭ��PID����ٶ�
//	if(g_fHW_PID_Out1 >5) g_fHW_PID_Out1 =5;//�����޷� �޷��ٶ���0-5֮��
//	if(g_fHW_PID_Out1 <0) g_fHW_PID_Out1 =0;
//	if(g_fHW_PID_Out2 >5) g_fHW_PID_Out2 =5;
//	if(g_fHW_PID_Out2 <0) g_fHW_PID_Out2 =0;
//	if(g_cThisState != g_cLastState)//������״̬�������ϴ�״̬���ͽ��иı�Ŀ���ٶȺͿ��Ƶ�����ڶ�ʱ�������ɶ�ʱ���Ƶ��
//	{
//		motorPidSetSpeed(g_fHW_PID_Out1,g_fHW_PID_Out2);//ͨ��������ٶȿ��Ƶ��
//	}
//	
//	g_cLastState = g_cThisState;//�����ϴκ���Թ�״̬	



////ͨ��������(����)�����Ϣ
////***************���������****************************//
//	sprintf((char *)Usart3String,"V1:%.2fV2:%.2f\r\n",Motor1Speed,Motor2Speed);//��ʾ�������ת�� ��λ��ת/��
//	HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//����ʽ����ͨ������������ַ� strlen:�����ַ�����С
//	
//	sprintf((char *)Usart3String,"Mileage%.2f\r\n",Mileage);//����С����� ��λcm
//	HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//����ʽ����ͨ������������ַ� strlen:�����ַ�����С
//	
//	sprintf((char *)Usart3String,"U:%.2fV\r\n",adcGetBatteryVoltage());//��ʾ��ص�ѹ
//	HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//����ʽ����ͨ������������ַ� strlen:�����ַ�����С	
//	
//	sprintf((char *)Usart3String,"HC_SR04:%.2fcm\r\n",HC_SR04_Read());//��ʾ����������
//	HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),0xFFFF);//ͨ������������ַ� strlen:�����ַ�����С	
//	
//   	sprintf((char *)Usart3String,"pitch:%.2f roll:%.2f yaw:%.2f\r\n",pitch,roll,yaw);//��ʾ6050���� ������ ����� �����
//	HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),0xFFFF);//ͨ������������ַ� strlen:�����ַ�����С	
//   
//   //mpu_dmp_get_data(&pitch,&roll,&yaw);//����ֵ:0,DMP�ɹ����ŷ����
//    while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){}  //������Խ���������������ݵ�����
//	
//	
//	HAL_Delay(5);//ע����ò����Թ���Ƶ��HC_SR04_Read()

////*************MPU6050����� PIDת�����*****************//

//   	sprintf((char *)Usart3String,"pitch:%.2f roll:%.2f yaw:%.2f\r\n",pitch,roll,yaw);//��ʾ6050���� ������ ����� �����
//	HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),0xFFFF);//ͨ������������ַ� strlen:�����ַ�����С	
//   
//   //mpu_dmp_get_data(&pitch,&roll,&yaw);//����ֵ:0,DMP�ɹ����ŷ����
//    while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){}  //������Խ���������������ݵ�����
//	
//	
//	g_fMPU6050YawMovePidOut = PID_realize(&pidMPU6050YawMovement,yaw);//PID�������Ŀ���ٶ� ����ٶȣ���ͻ����ٶȼӼ�

//	g_fMPU6050YawMovePidOut1 = 1.5 + g_fMPU6050YawMovePidOut;//�����ٶȼӼ�PID����ٶ�
//	g_fMPU6050YawMovePidOut2 = 1.5 - g_fMPU6050YawMovePidOut;
//	if(g_fMPU6050YawMovePidOut1 >3.5) g_fMPU6050YawMovePidOut1 =3.5;//�����޷�
//	if(g_fMPU6050YawMovePidOut1 <0) g_fMPU6050YawMovePidOut1 =0;
//	if(g_fMPU6050YawMovePidOut2 >3.5) g_fMPU6050YawMovePidOut2 =3.5;
//	if(g_fMPU6050YawMovePidOut2 <0) g_fMPU6050YawMovePidOut2 =0;
//	motorPidSetSpeed(g_fMPU6050YawMovePidOut1,g_fMPU6050YawMovePidOut2);


////**************���Ϲ���********************//
////�����߼�
//	if(HC_SR04_Read() > 25)//ǰ�����ϰ���
//	{
//		motorPidSetSpeed(1,1);//ǰ�˶�
//		HAL_Delay(100);
//	}
//	else{	//ǰ�����ϰ���
//		motorPidSetSpeed(-1,1);//�ұ��˶� ԭ��	
//		HAL_Delay(500);
//		if(HC_SR04_Read() > 25)//�ұ����ϰ���
//		{
//			motorPidSetSpeed(1,1);//ǰ�˶�
//			HAL_Delay(100);
//		}
//		else{//�ұ����ϰ���
//			motorPidSetSpeed(1,-1);//����˶� ԭ��
//			HAL_Delay(1000);
//			if(HC_SR04_Read() >25)//������ϰ���
//			{
//				 motorPidSetSpeed(1,1);//ǰ�˶�
//				HAL_Delay(100);
//			}
//			else{
//				motorPidSetSpeed(-1,-1);//���˶�
//				HAL_Delay(1000);
//				motorPidSetSpeed(-1,1);//�ұ��˶�
//				HAL_Delay(50);
//			}
//		}
//	}


////*************��PID���湦��************//
//	if(HC_SR04_Read() > 25)
//	{
//		motorPidSetSpeed(1,1);//ǰ�˶�
//		HAL_Delay(100);
//	}
//	if(HC_SR04_Read() < 20)
//	{
//		motorPidSetSpeed(-1,-1);//���˶�
//		HAL_Delay(100);
//	}

////**********PID���湦��***********//
//    g_fHC_SR04_Read=HC_SR04_Read();//��ȡǰ���ϰ������
//	if(g_fHC_SR04_Read < 60){  //���ǰ60cm �ж�������������
//		g_fFollow_PID_Out = PID_realize(&pidFollow,g_fHC_SR04_Read);//PID�������Ŀ���ٶ� ����ٶȣ���ͻ����ٶȼӼ�
//		if(g_fFollow_PID_Out > 6) g_fFollow_PID_Out = 6;//������ٶ��޷�
//		if(g_fFollow_PID_Out < -6) g_fFollow_PID_Out = -6;
//		motorPidSetSpeed(g_fFollow_PID_Out,g_fFollow_PID_Out);//�ٶ�����������
//	}
//	else motorPidSetSpeed(0,0);//���ǰ��60cm û�ж�����ֹͣ
//	HAL_Delay(10);//��ȡ���������������ܹ���

//	ANO_DT_Send_F2(Motor1Speed*100, 3.0*100,Motor2Speed*100,3.0*100);//��������λ��ͨ��F2֡����4��int16���͵����� ������ʾ
//	if(Usart_WaitReasFinish() == 0)//�Ƿ�������
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
//		  cJSON_Delete(cJsonData);//�ͷſռ䡢���ǲ���ɾ��cJsonVlaue��Ȼ�� �����쳣����
//		}
//		memset(Usart1_ReadBuf,0,255);//��ս���buf��ע�����ﲻ��ʹ��strlen	
//	}
//	printf("P:%.3f  I:%.3f  D:%.3f A:%.3f\r\n",p,i,d,a);
	
	
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 10);
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 10);
//	HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_SET);
//	HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_SET);
	
	
//	uint8_t c_Data[] = "�����������:�üһ�VCC\r\n";
//	HAL_UART_Transmit(&huart1,c_Data,sizeof(c_Data),0xFFFF);
//	HAL_Delay(1000);
//	printf("printf:�üһ�VCC����\r\n");
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
