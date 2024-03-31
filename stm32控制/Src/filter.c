/**
  ******************************************************************************
  * 文件名程: filter.c 
  * 作    者: HOUHUIDONG
  * 版    本: V1.0
  * 编写日期: 2021-09-10
  * 功    能: 板载串口底层驱动程序
  ******************************************************************************
  * 说明：
  * butterfilter
  * 需要结合采样频率和截至频率使用
  * 
  * 
  * 
  ******************************************************************************
  */

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "filter.h"
#include "math.h"

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
float Dr_Butter,Dl_Butter,Yz_Butter;

butterworth_filter DR_Butter;
butterworth_filter DL_Butter;
butterworth_filter YZ_Butter;

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/
float Butterworth(butterworth_filter* filter, float data);
/* 函数体 --------------------------------------------------------------------*/
void butterworth_Init (void)
{
    /* 采样频率1000  截至频率20 */
  DR_Butter.a[0] = 1.0f;
  DR_Butter.a[1] =-1.866892279711715f;
  DR_Butter.a[2] = 0.875214548253684f;
  DR_Butter.b[0] = 0.002080567135492f;
  DR_Butter.b[1] = 0.004161134270985f;
  DR_Butter.b[2] = 0.002080567135492f;

    /* 采样频率1000  截至频率20 */
  DL_Butter.a[0] = 1.0f;
  DL_Butter.a[1] =-1.866892279711715f;
  DL_Butter.a[2] = 0.875214548253684f;
  DL_Butter.b[0] = 0.002080567135492f;
  DL_Butter.b[1] = 0.004161134270985f;
  DL_Butter.b[2] = 0.002080567135492f;
  
      /* 采样频率1000  截至频率20 */
  YZ_Butter.a[0] = 1.0f;
  YZ_Butter.a[1] =-1.866892279711715f;
  YZ_Butter.a[2] = 0.875214548253684f;
  YZ_Butter.b[0] = 0.002080567135492f;
  YZ_Butter.b[1] = 0.004161134270985f;
  YZ_Butter.b[2] = 0.002080567135492f;
}

float Butterworth(butterworth_filter* filter, float data)
{
  int8_t i;
  for(i=2;i>0;i--)
  {
    filter->outputBuff[i] = filter->outputBuff[i-1];
    filter->inputBuff[i] = filter->inputBuff[i-1];
    
  }
  filter->inputBuff[0] = data;
  filter->outputBuff[0] = 0;
  for(i=1;i<3;i++)
  {
    filter->outputBuff[0] = filter->outputBuff[0] + filter->b[i]*filter->inputBuff[i];
    filter->outputBuff[0] = filter->outputBuff[0] - filter->a[i]*filter->outputBuff[i];
  }
  filter->outputBuff[0] = filter->outputBuff[0] + filter->b[0]*filter->inputBuff[0];
  return filter->outputBuff[0];
}

