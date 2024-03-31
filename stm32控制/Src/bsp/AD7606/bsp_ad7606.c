#include "bsp_ad7606.h"
#include "stm32f4xx_hal.h"
#include "stm32f407xx.h"



/* 设置过采样的GPIO: PA3 PA4 PA5 */
#define OS0_1()		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_SET)   //高电平
#define OS0_0()		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET)     //低电平
#define OS1_1()		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_SET)   //高电平
#define OS1_0()		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_RESET)     //低电平
#define OS2_1()		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_SET)   //高电平
#define OS2_0()		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_RESET)     //低电平

/* 启动AD转换的GPIO : PA6  */
#define CONVST_1()      HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_SET)   //高电平
#define CONVST_0()	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_RESET)     //低电平

/* 设置输入量程的GPIO :PA7  */
#define RANGE_1()	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET)   //高电平
#define RANGE_0()	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_RESET)     //低电平

/* AD7606复位口线 :   PA8  */
#define RESET_1()	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_SET)   //高电平
#define RESET_0()	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_RESET)     //低电平 

/* soft spi  : SCK/PH3 CS/PH4 MISO/PH5  */
#define AD_CS_LOW()     HAL_GPIO_WritePin(GPIOH,GPIO_PIN_4,GPIO_PIN_RESET)     //低电平
#define AD_CS_HIGH()    HAL_GPIO_WritePin(GPIOH,GPIO_PIN_4,GPIO_PIN_SET)   //高电平

#define AD_SCK_LOW()    HAL_GPIO_WritePin(GPIOH,GPIO_PIN_3,GPIO_PIN_RESET)        //低电平
#define AD_SCK_HIGH()   HAL_GPIO_WritePin(GPIOH,GPIO_PIN_3,GPIO_PIN_SET)      //高电平

#define AD_MISO_LOW()   HAL_GPIO_WritePin(GPIOH,GPIO_PIN_5,GPIO_PIN_RESET)        //低电平
#define AD_MISO_HIGH()  HAL_GPIO_WritePin(GPIOH,GPIO_PIN_5,GPIO_PIN_SET)      //高电平

#define ENABLE_INT()	__set_PRIMASK(0)	/* 使能全局中断 */
#define DISABLE_INT()	__set_PRIMASK(1)	/* 禁止全局中断 */

#define AD_MISO_IN		HAL_GPIO_ReadPin(GPIOH,GPIO_PIN_5)	


float int_sampleVol[8];           //AD7606采集八通道
float Yz;                         //垂向位移采集
float Pra,Prb,Pla,Plb;            //压强
float I_R,I_L;                    //主动缸伺服阀放大版电流采集
int t3=0;

float Active_IR_Sum,Active_IL_Sum;
float Active_IR_Offset,Active_IL_Offset;        //偏置电压
float Active_IR,Active_IL;                      //主动缸伺服阀放大板驱动电流，单位mA
uint32_t I_Count;
int I_Sample_Flag=1;
uint32_t count;

AD7606_FIFO_T        g_tAdcFifo;              /* 定义FIFO结构体变量 */
AD7606_VAR_T         g_tAD7606;	
TIM_HandleTypeDef    TIM_TimeBaseStructure;   
/*
*********************************************************************************************************
*	函 数 名: bsp_InitExtSRAM
*	功能说明: 配置连接外部SRAM的GPIO和FSMC
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitAD7606(void)	
{
  
  GPIO_InitTypeDef GPIO_InitStructure;
  
  
  __HAL_RCC_GPIOH_CLK_ENABLE();
  
  GPIO_InitStructure.Pin = GPIO_PIN_3|GPIO_PIN_4;    //AD_SPI_SCK_PIN|AD_CS_PIN
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;     //推挽输出
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStructure);         //GPIOH
  
  GPIO_InitStructure.Pin = GPIO_PIN_5;                //AD_SPI_MISO_GPIO_PORT,D7接口
  GPIO_InitStructure.Mode = GPIO_MODE_INPUT;          //下拉输入
  GPIO_InitStructure.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStructure);          //GPIOH
  
  __HAL_RCC_GPIOE_CLK_ENABLE();
  
  GPIO_InitStructure.Pin = GPIO_PIN_7;                //AD_RESET_PIN
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;      //推挽输出
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);         //GPIOE
  
  GPIO_InitStructure.Pin = GPIO_PIN_8;                //AD_CONVST_PIN
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;      //推挽输出
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);          //GPIOE
  
  GPIO_InitStructure.Pin = GPIO_PIN_9;                //AD_RANGE_PIN
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;      //推挽输出
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);          //GPIOE
  
  GPIO_InitStructure.Pin = GPIO_PIN_10;               //AD_OS0_PIN
  HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);          //GPIOE
  
  GPIO_InitStructure.Pin = GPIO_PIN_11;               //AD_OS1_PIN
  HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);          //GPIOE
  
  GPIO_InitStructure.Pin = GPIO_PIN_12;               //AD_OS2_PIN
  HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);          //GPIOE
  
  AD7606_SetOS(AD_OS_X32);		               /* 无过采样 */
  
  AD7606_Reset();
  
  CONVST_1();				               /* 启动转换的GPIO平时设置为高 */
  
}

/*
*********************************************************************************************************
*函 数 名: AD7606_SetOS
*功能说明: 配置AD7606数字滤波器，也就设置过采样倍率。
*通过设置 AD7606_OS0、OS1、OS2口线的电平组合状态决定过采样倍率。
*启动AD转换之后，AD7606内部自动实现剩余样本的采集，然后求平均值输出。
*
*过采样倍率越高，转换时间越长。
*       无过采样时，AD转换时间 4us;
*       2倍过采样时 = 8.7us;
*       4倍过采样时 = 16us
*       64倍过采样时 = 286us
*
*形    参: _ucOS : 过采样倍率
*返 回 值: 无
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
*函 数 名: AD7606_SetInputRange
*功能说明: 配置AD7606模拟信号输入量程。
*形    参: _ucRange : 0 表示正负5V   1表示正负10V
*返 回 值: 无
*********************************************************************************************************
*/
void AD7606_SetInputRange(uint8_t _ucRange)
{
  if (_ucRange == 0)
  {
    g_tAD7606.ucRange = 0;
    RANGE_0();	/* 设置为正负5V */
  }
  else
  {
    g_tAD7606.ucRange = 1;
    RANGE_1();	/* 设置为正负10V */
  }
}

/*
*********************************************************************************************************
*	函 数 名: AD7606_Reset
*	功能说明: 硬件复位AD7606。复位之后恢复到正常工作状态。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void AD7606_Reset(void)
{
  RESET_0();	/* 退出复位状态 */
  
  RESET_1();	/* 进入复位状态 */
  RESET_1();	/* 仅用于延迟。 RESET复位高电平脉冲宽度最小50ns。 */
  RESET_1();
  RESET_1();
  
  RESET_0();	/* 退出复位状态 */
}

/*
*********************************************************************************************************
*	函 数 名: AD7606_StartConvst
*	功能说明: 启动1次ADC转换
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void AD7606_StartConvst(void)
{
  /* page 7：  CONVST 高电平脉冲宽度和低电平脉冲宽度最短 25ns */
  /* CONVST平时为高 */
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
  
  
  HAL_TIM_Base_DeInit(&TIM_TimeBaseStructure);   //反初始化
  /* TIM4 configuration 
  TIM4CLK = 84 MHz	
  */
  
  TIM_TimeBaseStructure.Instance = TIM4;                  //基础频率84MHz
  TIM_TimeBaseStructure.Init.Prescaler = 2100;           /* 预分频系数 */
  TIM_TimeBaseStructure.Init.Period = 10 - 1; 		/* 计数周期 */
  TIM_TimeBaseStructure.Init.CounterMode = TIM_COUNTERMODE_UP;  //计数方向向上计数
  TIM_TimeBaseStructure.Init.ClockDivision = 0x0; 		/* 0 */
  HAL_TIM_Base_Init(&TIM_TimeBaseStructure);
  
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;   //时钟源选择
  HAL_TIM_ConfigClockSource(&TIM_TimeBaseStructure, &sClockSourceConfig);
  
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&TIM_TimeBaseStructure, &sMasterConfig);
  
  
  /* Clear TIM2 update pending flag[清除TIM4溢出中断标志] */
  __HAL_TIM_CLEAR_FLAG(&TIM_TimeBaseStructure, TIM_FLAG_UPDATE);
  //Sets the TIM Counter Register value on runtime.
  __HAL_TIM_SET_COUNTER(&TIM_TimeBaseStructure, 0);
  
  /* Enable TIM2 Update interrupt [使能TIM4中断更新]*/ 
  HAL_TIM_Base_Start_IT(&TIM_TimeBaseStructure);
  /* TIM2 enable counter [允许TIM4计数]*/
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
  for(count=0;count<16;count++)//读出16位数据
  {
    Num<<=1;
    AD_SCK_LOW();	        //下降沿有效
    if(AD_MISO_IN)Num++;
    AD_SCK_HIGH();
  }
  return(Num);
}
/*
*********************************************************************************************************
*函数名: ad7606_ReadBytes
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
  
  int_sampleVol[1] = ((int32_t)5)*((float)((short)g_tAdcFifo.sBuf[1])/32768);   //单位V   垂向位移
  int_sampleVol[2] = ((int32_t)5)*((float)((short)g_tAdcFifo.sBuf[2])/32768);   //单位V   右侧主动缸伺服阀放大板驱动电流采集
  int_sampleVol[3] = ((int32_t)5)*((float)((short)g_tAdcFifo.sBuf[3])/32768);   //单位V   左侧主动缸伺服阀放大板驱动电流采集
  int_sampleVol[4] = ((int32_t)5)*((float)((short)g_tAdcFifo.sBuf[4])/32768);   //单位V   右上（右A）
  int_sampleVol[5] = ((int32_t)5)*((float)((short)g_tAdcFifo.sBuf[5])/32768);   //单位V   右下 (右B)
  int_sampleVol[6] = ((int32_t)5)*((float)((short)g_tAdcFifo.sBuf[6])/32768);   //单位V   左上（左A）
  int_sampleVol[7] = ((int32_t)5)*((float)((short)g_tAdcFifo.sBuf[7])/32768);   //单位V   左下 (左B)
  
  Yz  = int_sampleVol[1]*600/4.995-157+3;     //垂向位移，单位mm
  
  I_R = int_sampleVol[2];                 //右侧主动杠伺服阀放大板驱动电流采集
  
  I_L = int_sampleVol[3];                 //左侧主动缸伺服阀放大板驱动电流采集
  
  Pra = int_sampleVol[4]*35/7*1966.20375;            //右侧主动缸A腔压强采集，单位MPa
  
  Prb = int_sampleVol[5]*35/7*1350.4516;            //右侧主动缸B腔压强采集，单位MPa
  
  Pla = int_sampleVol[6]*35/7*1966.20375;            //左侧主动缸A腔压强采集，单位MPa
  
  Plb = int_sampleVol[7]*35/7*1350.4516;            //左侧主动缸B腔压强采集，单位MPa
  
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
  
  AD7606_StartConvst();				/* ???ˉ2é?ù￡?±ü?aμú1×éêy?Yè?0μ??êìa */
  
  g_tAdcFifo.usRead = 0;				/* ±?D??ú?a??TIM2???°??0 */
  g_tAdcFifo.usWrite = 0;
  
  bsp_SET_TIM4_FREQ();		/* éè??2é?ù?μ?ê, 2￠ê1?üTIM2?¨ê±2é?ù?D?? */
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


