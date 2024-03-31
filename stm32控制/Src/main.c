/**
  ******************************************************************************
  * �ļ�����: main.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2017-03-30
  * ��    ��: ʹ�ô���ͨ��ʵ�ַ������ݵ������Լ����յ��Զ˷��͵�����
  ******************************************************************************
  * ˵����
  * ����������Ӳʯstm32������YS-F4Proʹ�á�
  * 
  * �Ա���
  * ��̳��http://www.ing10bbs.com
  * ��Ȩ��ӲʯǶ��ʽ�����Ŷ����У��������á�
  ******************************************************************************
  */
/* ����ͷ�ļ� ----------------------------------------------------------------*/
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
/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
uint8_t aRxBuffer;
uint8_t rx_buffer[256];
#define SERIAL_DATA_SIZE 50  // ���贮�ڽ��յ����ַ��������СΪ10  
#define FLOAT_ARRAY_SIZE 20  // ����Ҫ�洢�ĸ����������СΪ10 
#define MAX_STRING_SIZE 20 

#define TEST_PULSE_BUF_LEN           800
 double filtered_value;
 char serialData[SERIAL_DATA_SIZE][MAX_STRING_SIZE] = {"1.2","2.2"};  // ���贮�ڽ��յ����ַ������� 
 float floatArray[ SERIAL_DATA_SIZE],q,q1;  // ���ڴ洢ת����ĸ���������  
 float k[100],d[5],d1[5];
 char a[SERIAL_DATA_SIZE][MAX_STRING_SIZE];
 int i1=0,j1=0,a11=1,t=0,i2=0,i3=0;
 extern int t3;
   uint16_t data=-1;
 int s,a1,b,c;
   double temp,opa;
 int aa[]={65105.0,415.0,32760.0,35000.0,45000.0,15000.0,25000.0};
 float actual_displacement,actual_r_v,r_v,er,kp,r_displacement,r_a,follow1,follow01=170,t1,k1=100,T;
/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/**
 
  * ��������: ϵͳʱ������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
 #define ZERO_MODE               0 // 1������ѹ����ģʽ   0��������ѹ�ɼ�ģʽ
                                  // ����ѹ�������ڻ�ȡ�õ�0V������·������ʱ��ƫ�õ�ѹ
                                  // ADת��ֵ�����ݵ�·��ư��˷�����˵�ѹƫ��0.2V����
                                  // ����ģ���������˷ź�ʵ���ˡ��Ŵ���+��ѹƫ�á����ܣ�
                                  // �������Խ���˷����Ư�����⡣

// �Ŵ���=R2/R1=2000/6800��
#define OPA_RES_R1              6800  // 6.8k �˷�����˵���
#define OPA_RES_R2              2000  // 2k �˷ŷ�������
#define REFERENCE_VOLTAGE       3297  // �ο���ѹ���Ŵ�1000����
#define BIAS_VOLTAGE_IN1        0xFAB3E  // ����1ƫ�õ�ѹ������IN1��GND�̽�ʱAD7190ת�����
#define BIAS_VOLTAGE_IN2        0xF9DCA  // ����2ƫ�õ�ѹ������IN2��GND�̽�ʱAD7190ת�����
#define BIAS_VOLTAGE_IN3        0xFA8A4  // ����3ƫ�õ�ѹ������IN3��GND�̽�ʱAD7190ת�����
#define BIAS_VOLTAGE_IN4        0xFA9EB  // ����4ƫ�õ�ѹ������IN4��GND�̽�ʱAD7190ת�����
#define OPA1_RES_R1              10000  // 10k �˷�����˵���
#define OPA1_RES_R2              40200  // 40.2k �˷ŷ�������
#define SAMPLE_RESISTANCE          150   // ������������
#define REFERENCE_VOLTAGE          3297  // �ο���ѹ����λΪ��mV������ֵΪ3.3V����ֵΪʵ�ʲ���ֵ��
#define ERROR_COMPENSATION_IN1     30    // ͨ��1������������λΪ��mA��
#define ERROR_COMPENSATION_IN2     31    // ͨ��2������������λΪ��mA��
#define ERROR_COMPENSATION_IN3     31    // ͨ��3������������λΪ��mA��
#define ERROR_COMPENSATION_IN4     30    // ͨ��4������������λΪ��mA��
/* ˽�б��� ------------------------------------------------------------------*/
__IO int32_t ad7190_data[4]; // AD7190ԭʼת�����
__IO int32_t bias_data[4];   // ����ѹ��ADת�����
__IO double voltage_data[4]; // ��ѹֵ����λ��mV��
__IO uint8_t flag=0;         // �����ɼ���־
__IO int8_t number;          // ��ǰ�����ͨ��

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
 
  __HAL_RCC_PWR_CLK_ENABLE();                                     //ʹ��PWRʱ��

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);  //���õ�ѹ�������ѹ����1

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;      // �ⲿ����8MHz
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                        //��HSE 
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                    //��PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;            //PLLʱ��Դѡ��HSE
  RCC_OscInitStruct.PLL.PLLM = 8;                                 //8��ƵMHz
  RCC_OscInitStruct.PLL.PLLN = 336;                               //336��Ƶ
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;                     //2��Ƶ���õ�168MHz��ʱ��
  RCC_OscInitStruct.PLL.PLLQ = 7;                                 //USB/SDIO/������������ȵ���PLL��Ƶϵ��
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;       // ϵͳʱ�ӣ�168MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;              // AHBʱ�ӣ� 168MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;               // APB1ʱ�ӣ�42MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;               // APB2ʱ�ӣ�84MHz
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_RCC_EnableCSS();                                            // ʹ��CSS���ܣ�����ʹ���ⲿ�����ڲ�ʱ��ԴΪ����
  
 	// HAL_RCC_GetHCLKFreq()/1000    1ms�ж�һ��
	// HAL_RCC_GetHCLKFreq()/100000	 10us�ж�һ��
	// HAL_RCC_GetHCLKFreq()/1000000 1us�ж�һ��
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);                // ���ò�����ϵͳ�δ�ʱ��
  /* ϵͳ�δ�ʱ��ʱ��Դ */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* ϵͳ�δ�ʱ���ж����ȼ����� */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


/**
  * ��������: ������.
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */



typedef struct {  
    double q; // ��������Э����  
    double r; // ��������Э����  
    double x; // ״ֵ̬����  
    double p; // �������Э����  
    double k; // ����������  
} KalmanFilter;  
  
// �������˲�����ʼ��  
void kalman_filter_init(KalmanFilter *kf, double q, double r, double x_init, double p_init) {  
    kf->q = q;  
    kf->r = r;  
    kf->x = x_init;  
    kf->p = p_init;  
}  
  
// �������˲�������  
double kalman_filter_update(KalmanFilter *kf, double measurement) {  
    // Ԥ��  
    double p_predict = kf->p + kf->q;  
      
    // ���¿���������  
    kf->k = p_predict / (p_predict + kf->r);  
      
    // ���¹���ֵ  
    kf->x = kf->x + kf->k * (measurement - kf->x);  
      
    // ���¹������Э����  
    kf->p = (1 - kf->k) * p_predict;  
      
    return kf->x;  
}  






int main(void)

{
	float warebuf[8] =  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  uint8_t txbuf[50];
  /* ��λ�������裬��ʼ��Flash�ӿں�ϵͳ�δ�ʱ�� */
  HAL_Init();
  /* ����ϵͳʱ�� */
  SystemClock_Config();
	LED_GPIO_Init();
  KEY_GPIO_Init();
  /* ��ʼ�����ڲ����ô����ж����ȼ� */
  MX_USARTx_Init();


  /* ����ϵͳʱ�� */
  memcpy(txbuf,"����һ�������жϽ��ջ���ʵ��\n",50);
  HAL_UART_Transmit(&husartx,txbuf,strlen((char *)txbuf),1000);

  memcpy(txbuf,"�������ݲ��Իس�������\n",50);
  HAL_UART_Transmit(&husartx,txbuf,strlen((char *)txbuf),1000);
  HAL_UART_Receive_IT(&husartx,&aRxBuffer,1); 
	  MX_DMA_Init();
  /* ADC ��ʼ�� */
  MX_ADCx_Init();
  /* ����ADת����ʹ��DMA������ж� */
  HAL_ADC_Start_DMA(&hadcx,ADC_ConvertedValue,8);

  /* AD5689 ��ʼ�� */
  AD5689_Init();
  
  /* butterworth filter��ʼ�� */
  butterworth_Init();
  
  PID_Init();
  
  /* AD7606 ��ʼ�� */
  bsp_InitAD7606();
  
  AD7606_SetInputRange(1);    //0 ��ʾ����5V   1��ʾ����10V
  
  ad7606_StartRecord();



  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

//			 if(AD7190_Init()==0)
//  {
//    printf("��ȡ���� AD7190 !\n");
//    while(1)
//    {
//      HAL_Delay(1000);
//      if(AD7190_Init())
//        break;
//    }
//  }
//  printf("��⵽  AD7190 !\n");
//  /* AD7190��·���˲ɼ����� */
//  ad7190_unipolar_multichannel_conf();
//  flag=1;

  /* ��ʼ��AD7190���ͨ��״̬ */
 
  
  printf("ӲʯDAC��AD5689��ģ��ģ������ѹ�������\n"); 
  
  AD5689_Init();
  AD5689_WriteUpdate_DACREG(DAC_A,data); 
  AD5689_WriteUpdate_DACREG(DAC_B,0xFFFF-data); 
  printf("data:%d\n",data);
  
  opa=OPA_RES_R2/OPA_RES_R1;
  
int ii=0;	
  A=100,B=100;







  
  /* ����ѭ�� */
  while (1)
  {
		t_sin=t_sin+0.01;			
if(start4==1){printf("1\n");start4=0;}

		
	if(a11==1)
		{

//							for(int i=0;i<SERIAL_DATA_SIZE;i++)
//			{
//				for(int j=0;j<MAX_STRING_SIZE;j++)
////				printf("������ַ�����a[%d][%d]=%c\n",i,j,a[i][j]);
//				 serialData[i][j]=a[i][j];
////				HAL_Delay(150);
//			}
		
//		for (int i = 0; i < SERIAL_DATA_SIZE; i++) {  
//    printf("serialData[%d] = %s\n", i, serialData[i]);  
//  } 

		  for (int i = 0; i < SERIAL_DATA_SIZE; i++) {  
    // ���ַ���ת��Ϊ�����������洢��������������  
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
	
	
	
	
//  // ��ӡת����ĸ���������  
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

// ��ʼ���������˲���  
    KalmanFilter kf;  
    kalman_filter_init(&kf, 0.001, 0.1, 0, 1);  
  
    // ģ��һάʵʱ����  
if(t!=0)
//{
	{ d[i2]=k[0];i2++;t=0;
		if(d[2]!=0&&d[1]!=0&&d[3]!=0&&d[4]!=0)
		
		{
			
 
    int n = sizeof(d) / sizeof(d[0]);  
  
    // ���п������˲�  
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
  * ��������: ���ڽ�����ɻص�����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
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

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
