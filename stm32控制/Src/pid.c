#include "stm32f4xx_hal.h"
#include "pid.h"
#include "math.h"
#include "AD5689/bsp_AD5689.h"
#include "adc/bsp_adc.h"
#include "filter.h"
/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
float PIDOutput_R,PIDOutput_L;
float Centre_R,Centre_L;

/* 扩展变量 ------------------------------------------------------------------*/
PID_t PID_R,PID_L,PID_R_TEST,PID_L_TEST,PID_Y,PID_W;
/* 私有函数原形 --------------------------------------------------------------*/
float PID_Cal(PID_t* pid,float current);
/* 函数体 --------------------------------------------------------------------*/
/**
  * 函数功能: PID参数初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 
  */
void PID_Init(void)
{
  PID_R.Kp = 500.0f;
  PID_R.Ki = 2.0f;
  PID_R.Kd = 100.0f;
  PID_R.alpha = 0.0f;
  PID_R.epsilon = 5.0f;
  
  PID_L.Kp = 500.0f;
  PID_L.Ki = 2.0f;
  PID_L.Kd = 100.0f;
  PID_L.alpha = 0.0f;
  PID_L.epsilon = 5.0f;
  
  PID_R_TEST.Kp = 400.0f;
  PID_R_TEST.Ki = 2.0f;
  PID_R_TEST.Kd = 10.0f;
  PID_R_TEST.alpha = 0.0f;
  PID_R_TEST.epsilon = 0.0f;
  
  PID_L_TEST.Kp = 400.0f;
  PID_L_TEST.Ki = 2.0f;
  PID_L_TEST.Kd = 10.0f;
  PID_L_TEST.alpha = 0.0f;
  PID_L_TEST.epsilon = 0.0f;
  
  PID_Y.Kp = 600.0f;     //600    580
  PID_Y.Ki = 5.0f;
  PID_Y.Kd = 20.0f;
  PID_Y.alpha = 0.0f;                             
  PID_Y.epsilon =1.0f;
  
  PID_W.Kp = 3800.0f;
  PID_W.Ki = 5.0f;
  PID_W.Kd = 100.0f;
  PID_W.alpha = 0.0f;
  PID_W.epsilon = 1.0f;
  
//  PID_Y.Kp = 500.0f;
//  PID_Y.Ki = 0.0f;
//  PID_Y.Kd = 100.0f;  
  
//  PID_W.Kp = 7000.0f;
//  PID_W.Ki = 0.0f;
//  PID_W.Kd = 100.0f;
}

float PID_Cal(PID_t* pid,float current)
{
  float pErr,iErr,dErr = 0.0f;
  float output = 0.0f;
  
  pid->Err = pid->target - current;
  
  pErr = pid->Err - pid->Err_Last1;
  iErr = (pid->Err + pid->Err_Last1)*0.5f;     //梯形积分
  dErr = pid->Err - 2 * pid->Err_Last1 + pid->Err_Last2;
  
  pid->Pout = pid->Kp * pErr;
  
  /*** 积分分离 ***/
  if(fabsf(pid->Err)<=pid->epsilon)
    pid->Iout = pid->Ki * iErr;
  else
    pid->Iout = 0;
  
  /*** 不完全微分 ***/
  pid->Dout = pid->Kd * (1 - pid->alpha) * dErr + pid->alpha * pid->Dout_Last;
  
  output = pid->Pout + pid->Iout + pid->Dout;
  pid->Out += output;
  /*** 记录历史数据 ***/
  float err0,err1 = 0.0f;
  
  err1 = pid->Err_Last1;
  pid->Err_Last2 = err1;
  
  err0 = pid->Err;
  pid->Err_Last1 = err0;
  
  pid->Dout_Last = pid->Dout;
  
  return pid->Out;
}

void Set_PID_Target(PID_t *pid ,float target)
{
  pid->target = target;
}

/**
  * 函数功能: 主动缸调中
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 
  */
void CENTRE(void)
{
  
  Set_PID_Target(&PID_L,0);
  
  PIDOutput_L = PID_Cal(&PID_L,Dl_Butter);    
  
  Centre_L = 0xffff/2 - PIDOutput_L;
  
  if(Centre_L > 3*0xffff/4)
    Centre_L = 3*0xffff/4;
  if(Centre_L < 0xffff/4)
    Centre_L = 0xffff/4;
  
  AD5689_WriteUpdate_DACREG(DAC_A,(uint16_t)Centre_L);  //(uint16_t)Centre_L+0xffff/38
  
  
  Set_PID_Target(&PID_R,0);
  
  PIDOutput_R = PID_Cal(&PID_R,Dr_Butter); 
  
  Centre_R = 0xffff/2 - PIDOutput_R;
  
  if(Centre_R > 3*0xffff/4)
    Centre_R = 3*0xffff/4;
  if(Centre_R < 0xffff/4)
    Centre_R = 0xffff/4; 
  
  AD5689_WriteUpdate_DACREG(DAC_B,(uint16_t)Centre_R);     //(uint16_t)Centre_R+0xffff/384
  
}




