/**
  ******************************************************************************
  * 文件名程: main.c 
  * 作    者: 硬石嵌入式开发团队
  * 版    本: V1.0
  * 编写日期: 2017-03-30
  * 功    能: 使用串口通信实现发送数据到电脑以及接收电脑端发送的数据
  ******************************************************************************
  * 说明：
  * 本例程配套硬石stm32开发板YS-F4Pro使用。
  * 
  * 淘宝：
  * 论坛：http://www.ing10bbs.com
  * 版权归硬石嵌入式开发团队所有，请勿商用。
  ******************************************************************************
  */
/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "usart/bsp_usartx.h"
#include "string.h"
#include <stdio.h>  
#include "led/bsp_led.h"
#include <stdlib.h>
#include "key/bsp_key.h"
#include "AD5689/bsp_AD5689.h"
#include "math.h"
#include "bsp_ad7606.h"
#include "adc/bsp_adc.h"
#include "filter.h"
#include "pid.h"
#include "usde.h"
#define ARRAY_SIZE 3  
/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
uint8_t aRxBuffer;
uint8_t rx_buffer[256];
#define SERIAL_DATA_SIZE 50  // 假设串口接收到的字符串数组大小为10  
#define FLOAT_ARRAY_SIZE 20  // 假设要存储的浮点型数组大小为10 
#define MAX_STRING_SIZE 20 

#define TEST_PULSE_BUF_LEN           800
 double filtered_value;
 char serialData[SERIAL_DATA_SIZE][MAX_STRING_SIZE] = {"1.2","2.2"};  // 假设串口接收到的字符串数组 
 float floatArray[ SERIAL_DATA_SIZE],q,q1;  // 用于存储转换后的浮点型数组  
 float k[100],d[5],d1[5];
 char a[SERIAL_DATA_SIZE][MAX_STRING_SIZE];
 int i1=0,j1=0,a11=1,t=0,i2=0,i3=0;
 extern int t3;
   uint16_t data=-1;
 int s,a1,b,c;
   double temp,opa;
 int aa[]={65105.0,415.0,32760.0,35000.0,45000.0,15000.0,25000.0};
 float actual_displacement,actual_r_v,r_v,er,kp,r_displacement,r_a,follow1,follow01=170,t1,k1=100,T;
/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/
/**
 
  * 函数功能: 系统时钟配置
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
 #define ZERO_MODE               0 // 1：零点电压测试模式   0：正常电压采集模式
                                  // 零点电压测试用于获取得到0V（即短路）输入时的偏置电压
                                  // AD转换值。根据电路设计把运放输出端电压偏置0.2V左右
                                  // （即模拟量经过运放后实现了“放大倍数+电压偏置”功能）
                                  // 这样可以解决运放零点漂移问题。

// 放大倍数=R2/R1=2000/6800倍
#define OPA_RES_R1              6800  // 6.8k 运放输入端电阻
#define OPA_RES_R2              2000  // 2k 运放反馈电阻
#define REFERENCE_VOLTAGE       3297  // 参考电压（放大1000倍）
#define BIAS_VOLTAGE_IN1        0xFAB3E  // 输入1偏置电压，即把IN1和GND短接时AD7190转换结果
#define BIAS_VOLTAGE_IN2        0xF9DCA  // 输入2偏置电压，即把IN2和GND短接时AD7190转换结果
#define BIAS_VOLTAGE_IN3        0xFA8A4  // 输入3偏置电压，即把IN3和GND短接时AD7190转换结果
#define BIAS_VOLTAGE_IN4        0xFA9EB  // 输入4偏置电压，即把IN4和GND短接时AD7190转换结果
#define OPA1_RES_R1              10000  // 10k 运放输入端电阻
#define OPA1_RES_R2              40200  // 40.2k 运放反馈电阻
#define SAMPLE_RESISTANCE          150   // 电流采样电阻
#define REFERENCE_VOLTAGE          3297  // 参考电压（单位为：mV，理论值为3.3V，该值为实际测量值）
#define ERROR_COMPENSATION_IN1     30    // 通道1误差补偿电流（单位为：mA）
#define ERROR_COMPENSATION_IN2     31    // 通道2误差补偿电流（单位为：mA）
#define ERROR_COMPENSATION_IN3     31    // 通道3误差补偿电流（单位为：mA）
#define ERROR_COMPENSATION_IN4     30    // 通道4误差补偿电流（单位为：mA）
/* 私有变量 ------------------------------------------------------------------*/
__IO int32_t ad7190_data[4]; // AD7190原始转换结果
__IO int32_t bias_data[4];   // 零点电压的AD转换结果
__IO double voltage_data[4]; // 电压值（单位：mV）
__IO uint8_t flag=0;         // 启动采集标志
__IO int8_t number;          // 当前处理的通道

float Voltage[4];
float t_sin,y_sin,A,w,B,pi=3.1415926;
int start1=0,start2=0,start3,start4,stop=0;
__IO double current_data[4];
__IO float error_compensation[4]={ERROR_COMPENSATION_IN1,ERROR_COMPENSATION_IN2,ERROR_COMPENSATION_IN3,ERROR_COMPENSATION_IN4};
__IO int8_t number;  
float admax1=2.7,admin1=0.6,ader1=78;
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
 
  __HAL_RCC_PWR_CLK_ENABLE();                                     //使能PWR时钟

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);  //设置调压器输出电压级别1

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;      // 外部晶振，8MHz
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                        //打开HSE 
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                    //打开PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;            //PLL时钟源选择HSE
  RCC_OscInitStruct.PLL.PLLM = 8;                                 //8分频MHz
  RCC_OscInitStruct.PLL.PLLN = 336;                               //336倍频
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;                     //2分频，得到168MHz主时钟
  RCC_OscInitStruct.PLL.PLLQ = 7;                                 //USB/SDIO/随机数产生器等的主PLL分频系数
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;       // 系统时钟：168MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;              // AHB时钟： 168MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;               // APB1时钟：42MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;               // APB2时钟：84MHz
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_RCC_EnableCSS();                                            // 使能CSS功能，优先使用外部晶振，内部时钟源为备用
  
 	// HAL_RCC_GetHCLKFreq()/1000    1ms中断一次
	// HAL_RCC_GetHCLKFreq()/100000	 10us中断一次
	// HAL_RCC_GetHCLKFreq()/1000000 1us中断一次
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);                // 配置并启动系统滴答定时器
  /* 系统滴答定时器时钟源 */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* 系统滴答定时器中断优先级配置 */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


/**
  * 函数功能: 主函数.
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */



typedef struct {  
    double q; // 过程噪声协方差  
    double r; // 测量噪声协方差  
    double x; // 状态值估计  
    double p; // 估计误差协方差  
    double k; // 卡尔曼增益  
} KalmanFilter;  
  
// 卡尔曼滤波器初始化  
void kalman_filter_init(KalmanFilter *kf, double q, double r, double x_init, double p_init) {  
    kf->q = q;  
    kf->r = r;  
    kf->x = x_init;  
    kf->p = p_init;  
}  
  
// 卡尔曼滤波器更新  
double kalman_filter_update(KalmanFilter *kf, double measurement) {  
    // 预测  
    double p_predict = kf->p + kf->q;  
      
    // 更新卡尔曼增益  
    kf->k = p_predict / (p_predict + kf->r);  
      
    // 更新估计值  
    kf->x = kf->x + kf->k * (measurement - kf->x);  
      
    // 更新估计误差协方差  
    kf->p = (1 - kf->k) * p_predict;  
      
    return kf->x;  
}  






int main(void)

{
	float warebuf[8] =  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  uint8_t txbuf[50];
  /* 复位所有外设，初始化Flash接口和系统滴答定时器 */
  HAL_Init();
  /* 配置系统时钟 */
  SystemClock_Config();
	LED_GPIO_Init();
  KEY_GPIO_Init();
  /* 初始化串口并配置串口中断优先级 */
  MX_USARTx_Init();


  /* 配置系统时钟 */
  memcpy(txbuf,"这是一个串口中断接收回显实验\n",50);
  HAL_UART_Transmit(&husartx,txbuf,strlen((char *)txbuf),1000);

  memcpy(txbuf,"输入数据并以回车键结束\n",50);
  HAL_UART_Transmit(&husartx,txbuf,strlen((char *)txbuf),1000);
  HAL_UART_Receive_IT(&husartx,&aRxBuffer,1); 
	  MX_DMA_Init();
  /* ADC 初始化 */
  MX_ADCx_Init();
  /* 启动AD转换并使能DMA传输和中断 */
  HAL_ADC_Start_DMA(&hadcx,ADC_ConvertedValue,8);

  /* AD5689 初始化 */
  AD5689_Init();
  
  /* butterworth filter初始化 */
  butterworth_Init();
  
  PID_Init();
  
  /* AD7606 初始化 */
  bsp_InitAD7606();
  
  AD7606_SetInputRange(1);    //0 表示正负5V   1表示正负10V
  
  ad7606_StartRecord();



  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

//			 if(AD7190_Init()==0)
//  {
//    printf("获取不到 AD7190 !\n");
//    while(1)
//    {
//      HAL_Delay(1000);
//      if(AD7190_Init())
//        break;
//    }
//  }
//  printf("检测到  AD7190 !\n");
//  /* AD7190四路单端采集配置 */
//  ad7190_unipolar_multichannel_conf();
//  flag=1;

  /* 初始化AD7190检测通信状态 */
 
  
  printf("硬石DAC（AD5689）模块模拟量电压输出测试\n"); 
  
  AD5689_Init();
  AD5689_WriteUpdate_DACREG(DAC_A,data); 
  AD5689_WriteUpdate_DACREG(DAC_B,0xFFFF-data); 
  printf("data:%d\n",data);
  
  opa=OPA_RES_R2/OPA_RES_R1;
  
int ii=0;	
  A=100,B=100;







  
  /* 无限循环 */
  while (1)
  {
		t_sin=t_sin+0.01;			
if(start4==1){printf("1\n");start4=0;}

		
	if(a11==1)
		{

//							for(int i=0;i<SERIAL_DATA_SIZE;i++)
//			{
//				for(int j=0;j<MAX_STRING_SIZE;j++)
////				printf("输入的字符数组a[%d][%d]=%c\n",i,j,a[i][j]);
//				 serialData[i][j]=a[i][j];
////				HAL_Delay(150);
//			}
		
//		for (int i = 0; i < SERIAL_DATA_SIZE; i++) {  
//    printf("serialData[%d] = %s\n", i, serialData[i]);  
//  } 

		  for (int i = 0; i < SERIAL_DATA_SIZE; i++) {  
    // 将字符串转换为浮点数，并存储到浮点型数组中  
    floatArray[i] = atof(serialData[i]);k[i]=  floatArray[i];
  }  
  s=1;
	
	
	
	
//if(t!=0)
//	{ d[i2]=k[0];i2++;t=0;
//		if(d[2]!=0&&d[1]!=0)
//		{
//			if(fabs(d[0]-d[1])>fabs(d[0]-d[2]))
//				{	q1=(d[0]+d[2])/2;}
//			if(fabs(d[0]-d[1])<=fabs(d[0]-d[2]))
//				{	q1=(d[0]+d[1])/2;}
//			if(i2==3){d[0]=d[2],d[1]=0,d[2]=0,i2=1;}}	
//		}
	
	
	
	
//  // 打印转换后的浮点型数组  
////  for (int i = 0; i < FLOAT_ARRAY_SIZE; i++) {  
////		
////    printf("floatArray[%d] = %f\n", i, floatArray[i]);  
////  }    
   
////        printf("%lf\n", q);
	q=k[0];
    if(k[0]==29)
		{
			LED1_ON;
			LED2_OFF;

		}
		if(k[0]!=29)
		{
				LED1_OFF;
				LED2_OFF;

		}
		if(s==1)
		{

			LED3_ON;
		}
		a11=2;

		start3=0;

	}



		if(a11==2)
		{
			

//    if(flag==2)   
//    {      
//			
//#if ZERO_MODE==1
//      printf("IN%d. 0x%05X\n",number,bias_data[number]);
//#else        
////      voltage_data[number]=ad7190_data[number]>>4;
////      voltage_data[number]=voltage_data[number]*REFERENCE_VOLTAGE/OPA_RES_R2*OPA_RES_R1/0xFFFFF;
////      printf("IN%d. 0x%05X->%0.3fV\n",number,ad7190_data[number],voltage_data[number]/1000-0.89+0.3);
////      current_data[number]=ad7190_data[number]>>4;
////      current_data[number]=current_data[number]*REFERENCE_VOLTAGE/0xFFFFF*1000/SAMPLE_RESISTANCE+error_compensation[number];
////      printf("%d. 0x%05X->%0.3fmA\n",number,ad7190_data[number],current_data[number]/1000);
//#endif
//      
//      flag=1;
//    }
		actual_displacement=200*int_sampleVol[1]/(admax1-admin1)-ader1;
		actual_r_v=int_sampleVol[3]*(-2);

// 初始化卡尔曼滤波器  
    KalmanFilter kf;  
    kalman_filter_init(&kf, 0.001, 0.1, 0, 1);  
  
    // 模拟一维实时数据  
if(t!=0)
//{
	{ d[i2]=k[0];i2++;t=0;
		if(d[2]!=0&&d[1]!=0&&d[3]!=0&&d[4]!=0)
		
		{
			
 
    int n = sizeof(d) / sizeof(d[0]);  
  
    // 进行卡尔曼滤波  
    printf("Filtering data:\n");  
    for (int i = 0; i < n; i++) {  
        filtered_value = kalman_filter_update(&kf, d[i]);  
    }
			if(i2==5){d[0]=0,d[1]=0,d[2]=0,d[3]=0,d[4]=0,i2=0;}
		}
	}
	if(t3!=0)	
	{
		d1[i3]=actual_displacement;i3++;t3=0;
		if(d1[2]!=0&&d1[1]!=0&&d1[3]!=0&&d1[4]!=0)
		{
			if(fabs(d1[0]-d1[1])>fabs(d1[0]-d1[2]))
				{	q1=(d1[0]+d1[2])/2;}
			if(fabs(d1[0]-d1[1])<=fabs(d1[0]-d1[2]))
				{	q1=(d1[0]+d1[1])/2;}
			if(fabs(d1[0]-d1[3])>fabs(d1[0]-d1[4]))
				{	q1=(d1[0]+d1[4])/2;}
			if(fabs(d1[0]-d1[3])<=fabs(d1[0]-d1[4]))
				{	q1=(d1[0]+d1[3])/2;}				
			if(i3==5){d1[0]=q1,d1[1]=0,d1[2]=0,d1[3]=0,d1[4]=0,i3=1;}}	
		
			
	}		




//if(t3!=0)
//	{ 
//		}
			
			
//			actual_displacement=q1;
if(start1==1){if(actual_displacement>=1){T=0;}if((actual_displacement<=1&&actual_displacement>=-1)){start1=0;}}
if(start1==2){if(actual_displacement<=200){T=65535;}if((actual_displacement<=201&&actual_displacement>=199)){start1=0;}}
if(start2==0&&stop==1){T=65535/2;}

if(start2==1)
		

{		
		
			if(follow1>follow01+0.01)
			{follow1=follow1-0.003*k1;}
			if(follow1<=follow01-0.01)
			{follow1=follow1+0.003*k1;}

			if(follow1<=follow01+0.1&&follow1>=follow01-0.1)
			follow1=follow01;		
		
//				data=k[0];

		y_sin=A*sin(2*pi*w*t_sin)+B;
		r_displacement=y_sin;
		er=r_displacement-actual_displacement;
//		r_a=0.00061833359097*(data-65105)+20;
		r_a=er*kp;
		T=((r_a-20)/0.00061833359097+65105);
//		data=y_sin;	
}	

		    if(T>65535)
			{
        T=65535;
				a1=2;
			}				
		
    if(T<0)
			{
        T=0;
				b=3;
			}
		data=T;
		
    if(data<=(65535)&&data>=0)
		{

			AD5689_WriteUpdate_DACREG(DAC_A,data); 
      AD5689_WriteUpdate_DACREG(DAC_B,0xFFFF-data);  
      temp=(double)(data*2-0xFFFF)*2500*opa/0xFFFF;
			c=4;
//      printf("data:%d->%0.3fV\n",data,temp/1000);
//			HAL_Delay(50);
		}   
	
if(start3==0)
{
			HAL_Delay(50);
				warebuf[0] = actual_displacement;
        warebuf[1] = r_displacement;
				warebuf[2] = actual_r_v;
				warebuf[3] = q1;
				warebuf[4] = int_sampleVol[1];
				warebuf[4] =filtered_value+2;
				
        vcan_sendware((uint8_t *)warebuf,sizeof(warebuf));	
}

	
	}

//		    if(aRxBuffer==0x32)
//		{
//			LED2_ON;
//			LED1_OFF;
//			LED3_OFF;
//		}
//		    if(aRxBuffer==0x33)
//		{
//			LED3_ON;
//			LED2_OFF;
//			LED1_OFF;
//		}
//		if(aRxBuffer!=0x33&&aRxBuffer!=0x31&&aRxBuffer!=0x32)
//		{
//			LED3_ON;
//			LED2_ON;
//			LED1_ON;
//		}
//			LED3_OFF;
//			LED2_OFF;
//			LED1_OFF;
  }
}

/**
  * 函数功能: 串口接收完成回调函数
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	HAL_UART_Transmit(&husartx,&aRxBuffer,1,0);
	s=0;a11=1;start3=1;t++;
					if(s==0)
		{

			LED3_OFF;
			LED1_OFF;
		}
	a[i1][j1]=aRxBuffer;serialData[i1][j1]=a[i1][j1];
	if(a[i1][j1]!=0x2f&&a[i1][j1-1]==0x2f&&a[i1][j1-2]==0x2f){int w=a[i1][j1]; a[0][0]=w;i1=0;j1=0;serialData[0][0]=a[0][0];}
	if(a[i1][j1]==0x2c||(a[i1][j1]==0x2f&&a[i1][j1]==0x00))
		{	
			a[i1][j1]=0x30;serialData[i1][j1]=a[i1][j1];i1++;j1=0;
		}
		else
		{j1++;}
  HAL_UART_Receive_IT(&husartx,&aRxBuffer,1);

}

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
