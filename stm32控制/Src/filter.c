/**
  ******************************************************************************
  * �ļ�����: filter.c 
  * ��    ��: HOUHUIDONG
  * ��    ��: V1.0
  * ��д����: 2021-09-10
  * ��    ��: ���ش��ڵײ���������
  ******************************************************************************
  * ˵����
  * butterfilter
  * ��Ҫ��ϲ���Ƶ�ʺͽ���Ƶ��ʹ��
  * 
  * 
  * 
  ******************************************************************************
  */

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "filter.h"
#include "math.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
float Dr_Butter,Dl_Butter,Yz_Butter;

butterworth_filter DR_Butter;
butterworth_filter DL_Butter;
butterworth_filter YZ_Butter;

/* ��չ���� ------------------------------------------------------------------*/

/* ˽�к���ԭ�� --------------------------------------------------------------*/
float Butterworth(butterworth_filter* filter, float data);
/* ������ --------------------------------------------------------------------*/
void butterworth_Init (void)
{
    /* ����Ƶ��1000  ����Ƶ��20 */
  DR_Butter.a[0] = 1.0f;
  DR_Butter.a[1] =-1.866892279711715f;
  DR_Butter.a[2] = 0.875214548253684f;
  DR_Butter.b[0] = 0.002080567135492f;
  DR_Butter.b[1] = 0.004161134270985f;
  DR_Butter.b[2] = 0.002080567135492f;

    /* ����Ƶ��1000  ����Ƶ��20 */
  DL_Butter.a[0] = 1.0f;
  DL_Butter.a[1] =-1.866892279711715f;
  DL_Butter.a[2] = 0.875214548253684f;
  DL_Butter.b[0] = 0.002080567135492f;
  DL_Butter.b[1] = 0.004161134270985f;
  DL_Butter.b[2] = 0.002080567135492f;
  
      /* ����Ƶ��1000  ����Ƶ��20 */
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

