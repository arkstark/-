#include "bsp_ad7606.h"
#include "stm32f4xx_hal.h"
#include "stm32f407xx.h"



/* ���ù�������GPIO: PA3 PA4 PA5 */
#define OS0_1()		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_SET)   //�ߵ�ƽ
#define OS0_0()		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET)     //�͵�ƽ
#define OS1_1()		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_SET)   //�ߵ�ƽ
#define OS1_0()		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_RESET)     //�͵�ƽ
#define OS2_1()		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_SET)   //�ߵ�ƽ
#define OS2_0()		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_RESET)     //�͵�ƽ

/* ����ADת����GPIO : PA6  */
#define CONVST_1()      HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_SET)   //�ߵ�ƽ
#define CONVST_0()	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_RESET)     //�͵�ƽ

/* �����������̵�GPIO :PA7  */
#define RANGE_1()	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET)   //�ߵ�ƽ
#define RANGE_0()	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_RESET)     //�͵�ƽ

/* AD7606��λ���� :   PA8  */
#define RESET_1()	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_SET)   //�ߵ�ƽ
#define RESET_0()	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_RESET)     //�͵�ƽ 

/* soft spi  : SCK/PH3 CS/PH4 MISO/PH5  */
#define AD_CS_LOW()     HAL_GPIO_WritePin(GPIOH,GPIO_PIN_4,GPIO_PIN_RESET)     //�͵�ƽ
#define AD_CS_HIGH()    HAL_GPIO_WritePin(GPIOH,GPIO_PIN_4,GPIO_PIN_SET)   //�ߵ�ƽ

#define AD_SCK_LOW()    HAL_GPIO_WritePin(GPIOH,GPIO_PIN_3,GPIO_PIN_RESET)        //�͵�ƽ
#define AD_SCK_HIGH()   HAL_GPIO_WritePin(GPIOH,GPIO_PIN_3,GPIO_PIN_SET)      //�ߵ�ƽ

#define AD_MISO_LOW()   HAL_GPIO_WritePin(GPIOH,GPIO_PIN_5,GPIO_PIN_RESET)        //�͵�ƽ
#define AD_MISO_HIGH()  HAL_GPIO_WritePin(GPIOH,GPIO_PIN_5,GPIO_PIN_SET)      //�ߵ�ƽ

#define ENABLE_INT()	__set_PRIMASK(0)	/* ʹ��ȫ���ж� */
#define DISABLE_INT()	__set_PRIMASK(1)	/* ��ֹȫ���ж� */

#define AD_MISO_IN		HAL_GPIO_ReadPin(GPIOH,GPIO_PIN_5)	


float int_sampleVol[8];           //AD7606�ɼ���ͨ��
float Yz;                         //����λ�Ʋɼ�
float Pra,Prb,Pla,Plb;            //ѹǿ
float I_R,I_L;                    //�������ŷ����Ŵ������ɼ�
int t3=0;

float Active_IR_Sum,Active_IL_Sum;
float Active_IR_Offset,Active_IL_Offset;        //ƫ�õ�ѹ
float Active_IR,Active_IL;                      //�������ŷ����Ŵ��������������λmA
uint32_t I_Count;
int I_Sample_Flag=1;
uint32_t count;

AD7606_FIFO_T        g_tAdcFifo;              /* ����FIFO�ṹ����� */
AD7606_VAR_T         g_tAD7606;	
TIM_HandleTypeDef    TIM_TimeBaseStructure;   
/*
*********************************************************************************************************
*	�� �� ��: bsp_InitExtSRAM
*	����˵��: ���������ⲿSRAM��GPIO��FSMC
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitAD7606(void)	
{
  
  GPIO_InitTypeDef GPIO_InitStructure;
  
  
  __HAL_RCC_GPIOH_CLK_ENABLE();
  
  GPIO_InitStructure.Pin = GPIO_PIN_3|GPIO_PIN_4;    //AD_SPI_SCK_PIN|AD_CS_PIN
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;     //�������
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStructure);         //GPIOH
  
  GPIO_InitStructure.Pin = GPIO_PIN_5;                //AD_SPI_MISO_GPIO_PORT,D7�ӿ�
  GPIO_InitStructure.Mode = GPIO_MODE_INPUT;          //��������
  GPIO_InitStructure.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStructure);          //GPIOH
  
  __HAL_RCC_GPIOE_CLK_ENABLE();
  
  GPIO_InitStructure.Pin = GPIO_PIN_7;                //AD_RESET_PIN
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;      //�������
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);         //GPIOE
  
  GPIO_InitStructure.Pin = GPIO_PIN_8;                //AD_CONVST_PIN
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;      //�������
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);          //GPIOE
  
  GPIO_InitStructure.Pin = GPIO_PIN_9;                //AD_RANGE_PIN
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;      //�������
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);          //GPIOE
  
  GPIO_InitStructure.Pin = GPIO_PIN_10;               //AD_OS0_PIN
  HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);          //GPIOE
  
  GPIO_InitStructure.Pin = GPIO_PIN_11;               //AD_OS1_PIN
  HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);          //GPIOE
  
  GPIO_InitStructure.Pin = GPIO_PIN_12;               //AD_OS2_PIN
  HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);          //GPIOE
  
  AD7606_SetOS(AD_OS_X32);		               /* �޹����� */
  
  AD7606_Reset();
  
  CONVST_1();				               /* ����ת����GPIOƽʱ����Ϊ�� */
  
}

/*
*********************************************************************************************************
*�� �� ��: AD7606_SetOS
*����˵��: ����AD7606�����˲�����Ҳ�����ù��������ʡ�
*ͨ������ AD7606_OS0��OS1��OS2���ߵĵ�ƽ���״̬�������������ʡ�
*����ADת��֮��AD7606�ڲ��Զ�ʵ��ʣ�������Ĳɼ���Ȼ����ƽ��ֵ�����
*
*����������Խ�ߣ�ת��ʱ��Խ����
*       �޹�����ʱ��ADת��ʱ�� 4us;
*       2��������ʱ = 8.7us;
*       4��������ʱ = 16us
*       64��������ʱ = 286us
*
*��    ��: _ucOS : ����������
*�� �� ֵ: ��
*********************************************************************************************************
*/
void AD7606_SetOS(AD7606_OS_E _ucOS)
{
  g_tAD7606.ucOS = _ucOS;
  switch (_ucOS)
  {
  case AD_OS_X2:
    OS2_0();
    OS1_0();
    OS0_1();
    break;
    
  case AD_OS_X4:
    OS2_0();
    OS1_1();
    OS0_0();
    break;
    
  case AD_OS_X8:
    OS2_0();
    OS1_1();
    OS0_1();
    break;
    
  case AD_OS_X16:
    OS2_1();
    OS1_0();
    OS0_0();
    break;
    
  case AD_OS_X32:
    OS2_1();
    OS1_0();
    OS0_1();
    break;
    
  case AD_OS_X64:
    OS2_1();
    OS1_1();
    OS0_0();
    break;
    
  case AD_OS_NO:
  default:
    g_tAD7606.ucOS = AD_OS_NO;
    OS2_0();
    OS1_0();
    OS0_0();
    break;
  }
}

/*
*********************************************************************************************************
*�� �� ��: AD7606_SetInputRange
*����˵��: ����AD7606ģ���ź��������̡�
*��    ��: _ucRange : 0 ��ʾ����5V   1��ʾ����10V
*�� �� ֵ: ��
*********************************************************************************************************
*/
void AD7606_SetInputRange(uint8_t _ucRange)
{
  if (_ucRange == 0)
  {
    g_tAD7606.ucRange = 0;
    RANGE_0();	/* ����Ϊ����5V */
  }
  else
  {
    g_tAD7606.ucRange = 1;
    RANGE_1();	/* ����Ϊ����10V */
  }
}

/*
*********************************************************************************************************
*	�� �� ��: AD7606_Reset
*	����˵��: Ӳ����λAD7606����λ֮��ָ�����������״̬��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void AD7606_Reset(void)
{
  RESET_0();	/* �˳���λ״̬ */
  
  RESET_1();	/* ���븴λ״̬ */
  RESET_1();	/* �������ӳ١� RESET��λ�ߵ�ƽ��������С50ns�� */
  RESET_1();
  RESET_1();
  
  RESET_0();	/* �˳���λ״̬ */
}

/*
*********************************************************************************************************
*	�� �� ��: AD7606_StartConvst
*	����˵��: ����1��ADCת��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void AD7606_StartConvst(void)
{
  /* page 7��  CONVST �ߵ�ƽ�����Ⱥ͵͵�ƽ��������� 25ns */
  /* CONVSTƽʱΪ�� */
  CONVST_0();
  CONVST_0();
  CONVST_0();
  
  CONVST_1();
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim_base)
{
  if(htim_base->Instance==TIM4)
  { 
    __HAL_RCC_TIM4_CLK_ENABLE();
    
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
    HAL_NVIC_SetPriority(TIM4_IRQn,1,0);
  }
}

void bsp_SET_TIM4_FREQ(void)
{
  TIM_ClockConfigTypeDef     sClockSourceConfig;
  TIM_MasterConfigTypeDef    sMasterConfig;
  
  
  HAL_TIM_Base_DeInit(&TIM_TimeBaseStructure);   //����ʼ��
  /* TIM4 configuration 
  TIM4CLK = 84 MHz	
  */
  
  TIM_TimeBaseStructure.Instance = TIM4;                  //����Ƶ��84MHz
  TIM_TimeBaseStructure.Init.Prescaler = 2100;           /* Ԥ��Ƶϵ�� */
  TIM_TimeBaseStructure.Init.Period = 10 - 1; 		/* �������� */
  TIM_TimeBaseStructure.Init.CounterMode = TIM_COUNTERMODE_UP;  //�����������ϼ���
  TIM_TimeBaseStructure.Init.ClockDivision = 0x0; 		/* 0 */
  HAL_TIM_Base_Init(&TIM_TimeBaseStructure);
  
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;   //ʱ��Դѡ��
  HAL_TIM_ConfigClockSource(&TIM_TimeBaseStructure, &sClockSourceConfig);
  
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&TIM_TimeBaseStructure, &sMasterConfig);
  
  
  /* Clear TIM2 update pending flag[���TIM4����жϱ�־] */
  __HAL_TIM_CLEAR_FLAG(&TIM_TimeBaseStructure, TIM_FLAG_UPDATE);
  //Sets the TIM Counter Register value on runtime.
  __HAL_TIM_SET_COUNTER(&TIM_TimeBaseStructure, 0);
  
  /* Enable TIM2 Update interrupt [ʹ��TIM4�жϸ���]*/ 
  HAL_TIM_Base_Start_IT(&TIM_TimeBaseStructure);
  /* TIM2 enable counter [����TIM4����]*/
  __HAL_TIM_ENABLE(&TIM_TimeBaseStructure);
}  


void SPI_SendData(unsigned short data)    
{  
  unsigned char count=0;   
  AD_SCK_LOW();	
  for(count=0;count<16;count++)  
  { 	  
    if(data&0x8000)
      AD_MISO_LOW();  
    else 
      AD_MISO_HIGH();   
    data<<=1;    
    AD_SCK_LOW(); 	 
    AD_SCK_HIGH();			        
  }		 			    
} 		 

unsigned short SPI_ReceiveData(void)	  
{ 	 
  unsigned char count=0; 	  
  unsigned short Num=0; 
  AD_SCK_HIGH();	
  for(count=0;count<16;count++)//����16λ����
  {
    Num<<=1;
    AD_SCK_LOW();	        //�½�����Ч
    if(AD_MISO_IN)Num++;
    AD_SCK_HIGH();
  }
  return(Num);
}
/*
*********************************************************************************************************
*������: ad7606_ReadBytes
*********************************************************************************************************
*/
uint16_t ad7606_ReadBytes(void)
{
  uint16_t usData = 0;
  
  usData = SPI_ReceiveData();
  /* Return the shifted data */
  return usData;
}

/*
*********************************************************************************************************
*********************************************************************************************************
*/
void ad7606_IRQSrc(void)
{
  uint8_t i;
	t3++;
  uint16_t usReadValue;
  
  __HAL_TIM_CLEAR_FLAG(&TIM_TimeBaseStructure, TIM_FLAG_UPDATE);
  
  AD_CS_LOW();
  
  for (i = 0; i < CH_NUM; i++)
  {
    usReadValue = ad7606_ReadBytes();	
    if (g_tAdcFifo.usWrite < ADC_FIFO_SIZE)
    {
      g_tAdcFifo.sBuf[g_tAdcFifo.usWrite] = usReadValue;
      ++g_tAdcFifo.usWrite;
    }
  }		
  
  AD_CS_HIGH();
  
  AD7606_StartConvst();
  
  int_sampleVol[1] = ((int32_t)5)*((float)((short)g_tAdcFifo.sBuf[1])/32768);   //��λV   ����λ��
  int_sampleVol[2] = ((int32_t)5)*((float)((short)g_tAdcFifo.sBuf[2])/32768);   //��λV   �Ҳ��������ŷ����Ŵ�����������ɼ�
  int_sampleVol[3] = ((int32_t)5)*((float)((short)g_tAdcFifo.sBuf[3])/32768);   //��λV   ����������ŷ����Ŵ�����������ɼ�
  int_sampleVol[4] = ((int32_t)5)*((float)((short)g_tAdcFifo.sBuf[4])/32768);   //��λV   ���ϣ���A��
  int_sampleVol[5] = ((int32_t)5)*((float)((short)g_tAdcFifo.sBuf[5])/32768);   //��λV   ���� (��B)
  int_sampleVol[6] = ((int32_t)5)*((float)((short)g_tAdcFifo.sBuf[6])/32768);   //��λV   ���ϣ���A��
  int_sampleVol[7] = ((int32_t)5)*((float)((short)g_tAdcFifo.sBuf[7])/32768);   //��λV   ���� (��B)
  
  Yz  = int_sampleVol[1]*600/4.995-157+3;     //����λ�ƣ���λmm
  
  I_R = int_sampleVol[2];                 //�Ҳ��������ŷ����Ŵ�����������ɼ�
  
  I_L = int_sampleVol[3];                 //����������ŷ����Ŵ�����������ɼ�
  
  Pra = int_sampleVol[4]*35/7*1966.20375;            //�Ҳ�������Aǻѹǿ�ɼ�����λMPa
  
  Prb = int_sampleVol[5]*35/7*1350.4516;            //�Ҳ�������Bǻѹǿ�ɼ�����λMPa
  
  Pla = int_sampleVol[6]*35/7*1966.20375;            //���������Aǻѹǿ�ɼ�����λMPa
  
  Plb = int_sampleVol[7]*35/7*1350.4516;            //���������Bǻѹǿ�ɼ�����λMPa
  
}

void ACTIVE_I(void)
{
  if(2000<=count&&count<5000)
  {
    I_Count +=1 ;
    Active_IR_Sum += I_R;
    Active_IL_Sum += I_L;  
  }
    
  if(count>=5000)
  {
    Active_IR_Offset = Active_IR_Sum/I_Count;
    Active_IL_Offset = Active_IL_Sum/I_Count;
  }
  
}


/*
*********************************************************************************************************
*********************************************************************************************************
*/
uint8_t GetAdcFormFifo(uint16_t *_usReadAdc)
{
  uint16_t usWrite;
  
  DISABLE_INT();	
  usWrite = g_tAdcFifo.usWrite;
  ENABLE_INT();
  
  if (usWrite != g_tAdcFifo.usRead)
  {
    *_usReadAdc = g_tAdcFifo.sBuf[g_tAdcFifo.usRead];
    
    DISABLE_INT();
    if (++g_tAdcFifo.usRead >= ADC_FIFO_SIZE)
    {
      g_tAdcFifo.usRead = 0;
    }		
    ENABLE_INT();
    return 1;		
  }
  return 0;
}

/*
*********************************************************************************************************
*********************************************************************************************************
*/
void ad7606_StartRecord(void)
{
  
  AD7606_StartConvst();				/* ???��2��?����?����?a�̨�1������y?Y��?0��??����a */
  
  g_tAdcFifo.usRead = 0;				/* ��?D??��?a??TIM2???��??0 */
  g_tAdcFifo.usWrite = 0;
  
  bsp_SET_TIM4_FREQ();		/* ����??2��?��?��?��, 2�騺1?��TIM2?������2��?��?D?? */
}



void ad7606_StopRecord(void)
{
  
  __HAL_TIM_DISABLE(&TIM_TimeBaseStructure);
  
}


void TIM4_IRQHandler(void)
{
  ad7606_IRQSrc();
  g_tAdcFifo.usWrite = 0;
}


