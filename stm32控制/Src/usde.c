#include "usde.h"
#include "bsp_ad7606.h"
#include "usart/bsp_usartx.h"
#include "adc/bsp_adc.h"
#include "pid.h"
#include "filter.h"
#include "math.h"
#include "AD5689/bsp_AD5689.h"

#define   Lambda1               10                //1 1 1  800   
#define   Lambda2               1                 
#define   Lambda3               1
#define   Lambda1_W             10                //1 1 1  800   
#define   Lambda2_W             1                 
#define   Lambda3_W             1    
#define   Epsilon               0.01 
#define   T                     0.001
#define   K                     1950   //1950
#define   Kphi                  1950

//#define   PI                    3.1415926
//#define   SIN_MAX              170             //单位mm  9  11   13   15   17   19   21
//#define   SIN_MIN              130            //单位mm
//#define   CONTROL_PERIOD        1
//#define   SIN_PERIOD            2              //正弦函数周期/秒
//#define   DELTA_T               0.025f         //单位/秒

//double Sin_Count;
//double Sin_Tar;

float dz1_hat,dz2_hat,dz3_hat;
float z1_hat,z2_hat,z3_hat;

float dw1_hat,dw2_hat,dw3_hat;
float w1_hat,w2_hat,w3_hat;

//float PIDOutput_TEST_R;
//float output_test_R;
//float Ur_True;
//
//float PIDOutput_TEST_L;
//float output_test_L;
//float Ul_True;

float PIDOutput_Y,PIDOutput_W;
float Output_L,Output_R;
float Ul_True,Ur_True;
float UL,UR;



//void signal_sin()
//{  
//  Sin_Count += (1.0/(SIN_PERIOD*1000/CONTROL_PERIOD));   //909.090909   1.1Hz    769.230769     1.3Hz    666.666667    1.5Hz
//  Sin_Tar = (SIN_MAX - SIN_MIN)*0.5 * sin(Sin_Count*2*PI) + (SIN_MAX + SIN_MIN)*0.5;
//  if(Sin_Count > SIN_PERIOD) 
//    Sin_Count = 0;  
//}

void USDE_Controller(void)
{
  
  
  HGO_V(Yz_Butter);
  HGO_W(Angle_y);
  
  Set_PID_Target(&PID_Y,0);
  PIDOutput_Y = PID_Cal(&PID_Y,Yz_Butter);
  
  
  Set_PID_Target(&PID_W,0);
  PIDOutput_W = PID_Cal(&PID_W,Angle_y);
  
  UL = (PIDOutput_Y + PIDOutput_W)/2;
  UR = (PIDOutput_Y - PIDOutput_W)/2;
  
  if(UL > 0) UL += -320;
  if(UL < 0) UL += -320;
  if(UR > 0) UR += -1250;
  if(UR < 0) UR += -1250;
  
  Output_L = 0xffff/2 - UL;
  
  if(Output_L < 0xffff/4+900)
    Output_L = 0xffff/4+900;
  if(Output_L > 3*0xffff/4-750)
    Output_L = 3*0xffff/4-750;
  
  Ul_True = (Output_L - 0xffff/2) / 3276.8f;
  
//  AD5689_WriteUpdate_DACREG(DAC_A,(uint16_t)Output_L+0xffff/384);
  AD5689_WriteUpdate_DACREG(DAC_A,(uint16_t)Output_L);
  
  
  Output_R = 0xffff/2 - UR;
  
  if(Output_R < 0xffff/4+900)
    Output_R = 0xffff/4+900;
  if(Output_R > 3*0xffff/4-750)
    Output_R = 3*0xffff/4-750;
  
  Ur_True = (Output_R - 0xffff/2) / 3276.8f;
  
  AD5689_WriteUpdate_DACREG(DAC_B,(uint16_t)Output_R);
  
  
 
//  signal_sin();
//  Set_PID_Target(&PID_R_TEST,Sin_Tar);
//    
//  PIDOutput_TEST_R = PID_Cal(&PID_R_TEST,Yz); 
//
//  output_test_R = 0xffff/2-PIDOutput_TEST_R;
//  
//  if(output_test_R < 0xffff/4+900)
//    output_test_R = 0xffff/4+900;
//  if(output_test_R > 3*0xffff/4-750)
//    output_test_R = 3*0xffff/4-750;
//  
//  Ur_True = (output_test_R - 0xffff/2) / 3276.8f;
//  
//
//  AD5689_WriteUpdate_DACREG(DAC_B,(uint16_t)output_test_R);
//  
//  
//  Set_PID_Target(&PID_L_TEST,Sin_Tar);
//  
//  PIDOutput_TEST_L = PID_Cal(&PID_L_TEST,Yz); 
//  
//  output_test_L = 0xffff/2-PIDOutput_TEST_L;
//  
//  if(output_test_L < 0xffff/4+900)
//    output_test_L = 0xffff/4+900;
//  if(output_test_L > 3*0xffff/4-750)
//    output_test_L = 3*0xffff/4-750;
//  
//  Ul_True = (output_test_L - 0xffff/2) / 3276.8f;
//  
//  
//  AD5689_WriteUpdate_DACREG(DAC_A,(uint16_t)output_test_L);
  
}


//0xffff/4+1000对应右侧主动缸伺服阀放大板的驱动电流为-50mA（电流饱和）
//3*0xffff/4-750对应右侧主动缸伺服阀放大板的驱动电流为-50mA（电流饱和）

/**
* 函数功能: 高增益状态观测器
* 输入参数: 无
* 返 回 值: 无
* 说    明: 无
*/
float HGO_V( float V1 )
{  
  dz1_hat = z2_hat+(1/Epsilon)*(V1-z1_hat);
  z1_hat += dz1_hat * T;
  dz2_hat = z3_hat+(1/(Epsilon*Epsilon))*(V1-z1_hat);
  z2_hat += dz2_hat * T;
  dz3_hat = (-K*(Lambda1*z1_hat+Lambda2*z2_hat+Lambda3*z3_hat)-Lambda1*z2_hat-Lambda2*z3_hat)+(1/(Epsilon*Epsilon*Epsilon))*(V1-z1_hat);
  z3_hat += dz3_hat * T;
  
  return dz1_hat;
  
}
/**
* 函数功能: 高增益状态观测器
* 输入参数: 俯仰角位移
* 返 回 值: 无
* 说    明: 无
*/
float HGO_W( float W1 )
{  
  dw1_hat = w2_hat+(1/Epsilon)*(W1-w1_hat);
  w1_hat += dw1_hat * T;
  dw2_hat = w3_hat+(1/(Epsilon*Epsilon))*(W1-w1_hat);
  w2_hat += dw2_hat * T;
  dw3_hat = (-Kphi*(Lambda1_W*w1_hat+Lambda2_W*w2_hat+Lambda3_W*w3_hat)-Lambda1_W*w2_hat-Lambda2_W*w3_hat)+(1/(Epsilon*Epsilon*Epsilon))*(W1-w1_hat);
  w3_hat += dw3_hat * T;
  
  return dw1_hat;
  
}