/**
  ******************************************************************************
  * @file    usde.h
  * @brief   This file contains the headers of the interrupt handlers.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USDE_H
#define __USDE_H

    
    

    


/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
void USDE_Controller(void);
float HGO_V( float V1 );
float HGO_W( float W1 );

void signal_sin();

extern float z1_hat,z2_hat,z3_hat;
extern float w1_hat,w2_hat,w3_hat;
extern double Sin_Tar;
extern float PIDOutput_TEST_R;
extern float output_test_R;
extern float Ur_True;
extern float PIDOutput_TEST_L;
extern float output_test_L;
extern float Ul_True;
extern float Output_L,Output_R;
extern float UL,UR;
#endif /* __STM32F4xx_IT_H */