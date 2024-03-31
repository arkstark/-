#ifndef _BSP_AD7606_H
#define _BSP_AD7606_H

#include "stm32f4xx_hal.h"

typedef enum
{
  AD_OS_NO = 0,
  AD_OS_X2 = 1,
  AD_OS_X4 = 2,
  AD_OS_X8 = 3,
  AD_OS_X16 = 4,
  AD_OS_X32 = 5,
  AD_OS_X64 = 6
}AD7606_OS_E;

typedef struct
{
  uint8_t ucOS;			/* 过采样倍率 */
  uint8_t ucRange;		/* 输入量程 */
}AD7606_VAR_T;

#define ADC_FIFO_SIZE	(2*1024)	
#define CH_NUM		8			


typedef struct
{
  uint16_t usRead;		
  uint16_t usWrite;		

  uint16_t usCount;		
  uint8_t ucFull;			

  uint16_t  sBuf[ADC_FIFO_SIZE];
  
}AD7606_FIFO_T;

void bsp_InitAD7606(void);
void AD7606_SetOS(AD7606_OS_E _ucOS);
void AD7606_SetInputRange(uint8_t _ucRange);
void AD7606_Reset(void);
void AD7606_StartConvst(void);
void AD7606_ReadNowAdc(void);
void ad7606_StartRecord(void);
void ACTIVE_I(void);


/* 扩展变量 */
extern AD7606_FIFO_T g_tAdcFifo;
extern AD7606_VAR_T g_tAD7606;
extern TIM_HandleTypeDef TIM_TimeBaseStructure;
extern float int_sampleVol[8];      
extern float Yz;
extern float Pra,Prb,Pla,Plb;
extern float I_R,I_L;
extern float Active_IR,Active_IL;
extern float Active_IR_Offset,Active_IL_Offset;
extern float Active_IR_Sum,Active_IL_Sum;
extern uint32_t count;

#endif
