/**
  ******************************************************************************
  * @file    filter.h
  * @brief   This file contains the headers of the interrupt handlers.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FILTER_H
#define __FILTER_H
/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

typedef struct _butterworth_filter
{
  float      inputBuff[3];
  float      outputBuff[3];
  float      filteredData;
  float      a[3];
  float      b[3];
}butterworth_filter;
/* Exported functions ------------------------------------------------------- */
void butterworth_Init(void);
float Butterworth(butterworth_filter* filter, float data);

//extern butterworth_filter PHI_Butter;
//extern butterworth_filter W2_Butter;
//extern butterworth_filter AZ_Butter;

extern butterworth_filter DR_Butter;
extern butterworth_filter DL_Butter;
extern butterworth_filter YZ_Butter;
extern float Dr_Butter,Dl_Butter,Yz_Butter;
#endif /* __STM32F4xx_IT_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/