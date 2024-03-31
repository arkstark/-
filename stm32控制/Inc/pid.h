#ifndef __PID_H
#define __PID_H
/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
typedef struct
{
  float Kp;
  float Ki;
  float Kd;
  float Pout;
  float Iout;
  float Dout;
  float Err;
  float target;
  float Err_Last1;
  float Err_Last2;
  float epsilon;
  float alpha;
  float Dout_Last;
  float Out; 
  float dead_up;
  float dead_down;
}PID_t;

/* Exported functions ------------------------------------------------------- */
float PID_Cal(PID_t* pid,float current);
void Set_PID_Target(PID_t *pid ,float target);
void PID_Init(void);
void CENTRE(void);

/* ¿©’π±‰¡ø ------------------------------------------------------- */
extern float Centre_R,Centre_L;
extern PID_t PID_R,PID_L,PID_R_TEST,PID_L_TEST,PID_Y,PID_W;
extern float PIDOutput_L,PIDOutput_R;

#endif /* __STM32F4xx_IT_H */