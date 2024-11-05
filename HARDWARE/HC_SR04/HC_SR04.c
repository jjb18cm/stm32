#include "HC_SR04.h"

/*******************
* us级延时
* usdelay:要延时的us时间
*  
*
*******************/
void HC_SR04_Delayus(uint32_t usdelay)
{
  __IO uint32_t Delay = usdelay * (SystemCoreClock /8U/1000U/1000);//SystemCoreClock:系统频率
  do
  {
    __NOP();//ʹ�ÿ�ָ����ʱ����ֲ��ͬ��Ƭ��ע��__NOP(); ִ��ʱ��
  }
  while (Delay --);
}
/*******************
*  hc_SR40超声波读取数据距离
*  
*  return距离:cm () 
两个hc_SR40_return间隔两毫秒以上
*******************/
float HC_SR04_Read(void)
{
	uint32_t i = 0;
	float Distance;
	HAL_GPIO_WritePin(HC_SR04_Trig_GPIO_Port,HC_SR04_Trig_Pin,GPIO_PIN_SET);//输出15us高电平
	HC_SR04_Delayus(15);
	HAL_GPIO_WritePin(HC_SR04_Trig_GPIO_Port,HC_SR04_Trig_Pin,GPIO_PIN_RESET);//高电平结束设置为低电平
	
	while(HAL_GPIO_ReadPin(HC_SR04_Echo_GPIO_Port,HC_SR04_Echo_Pin) == GPIO_PIN_RESET)//等待回响高电平
	{
		i++;
		HC_SR04_Delayus(1);
		if(i>100000) return -1;//超时退出防止卡死
	}
	i = 0;
	while(HAL_GPIO_ReadPin(HC_SR04_Echo_GPIO_Port,HC_SR04_Echo_Pin) == GPIO_PIN_SET)//循环为2us
	{
		i = i+1;
		HC_SR04_Delayus(1);//1us 延时
		if(i >100000) return -2;//超时退出
	}
	Distance = i*2*0.033/2;//上面应为2us
	return Distance	;
}

