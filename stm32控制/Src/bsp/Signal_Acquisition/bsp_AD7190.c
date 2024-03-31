/**
  ******************************************************************************
  * �ļ�����: bsp_spiflash.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2017-03-30
  * ��    ��: ���ش���Flash�ײ�����ʵ��
  ******************************************************************************
  * ˵����
  * ����������Ӳʯstm32������YS-F4Proʹ�á�
  * 
  * �Ա���
  * ��̳��http://www.ing10bbs.com
  * ��Ȩ��ӲʯǶ��ʽ�����Ŷ����У��������á�
  ******************************************************************************
  */

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "Signal_Acquisition/bsp_AD7190.h"
#include "usart/bsp_usartx.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
SPI_HandleTypeDef hspi_AD7190;

/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/**
  * ��������: SPI��ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
*/
void MX_AD7190_SPI_Init(void)
{
  
  GPIO_InitTypeDef GPIO_InitStruct;
  /* ʹ��SPI�����Լ�SPI����ʱ�� */
  AD7190_SPIx_CLK_ENABLE();
  AD7190_GPIO_CLK_ENABLE();
  /**SPI1 GPIO Configuration    
  PA5     ------> SPI1_SCK
  PB4     ------> SPI1_MISO
  PB5     ------> SPI1_MOSI 
  */    
  GPIO_InitStruct.Pin = AD7190_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(AD7190_SCK_GPIO_Port, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = AD7190_MISO_Pin|AD7190_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(AD7190_MISO_GPIO_Port, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = AD7190_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(AD7190_CS_GPIO_Port, &GPIO_InitStruct);  
  
  /* SPI�������� */
  hspi_AD7190.Instance = AD7190_SPIx;
  hspi_AD7190.Init.Mode = SPI_MODE_MASTER;
  hspi_AD7190.Init.Direction = SPI_DIRECTION_2LINES;
  hspi_AD7190.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi_AD7190.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi_AD7190.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi_AD7190.Init.NSS = SPI_NSS_SOFT;
  hspi_AD7190.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi_AD7190.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi_AD7190.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi_AD7190.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi_AD7190.Init.CRCPolynomial = 7;
  HAL_SPI_Init(&hspi_AD7190);
  /* Ƭѡʹ�� */
  AD7190_CS_ENABLE();
}

/***************************************************************************//**
 * @brief Writes data into a register.
 *
 * @param registerAddress - Address of the register.
 * @param registerValue - Data value to write.
 * @param bytesNumber - Number of bytes to be written.
 *
 * @return none.
*******************************************************************************/
void AD7190_SetRegisterValue(unsigned char registerAddress,unsigned int  registerValue,unsigned char bytesNumber)
{
  unsigned char writeCommand[5] = {0, 0, 0, 0, 0};
  unsigned char* dataPointer    = (unsigned char*)&registerValue;
  unsigned char bytesNr         = bytesNumber;
  
  writeCommand[0] = AD7190_COMM_WRITE | AD7190_COMM_ADDR(registerAddress);
  while(bytesNr > 0)
  {
      writeCommand[bytesNr] = *dataPointer;
      dataPointer ++;
      bytesNr --;
  }
  HAL_SPI_Transmit(&hspi_AD7190,writeCommand, bytesNumber+1,0xFFFFFF);
}


/***************************************************************************//**
 * @brief Reads the value of a register.
 *
 * @param registerAddress - Address of the register.
 * @param bytesNumber - Number of bytes that will be read.
 *
 * @return buffer - Value of the register.
*******************************************************************************/
unsigned int AD7190_GetRegisterValue(unsigned char registerAddress,unsigned char bytesNumber)
{
  unsigned char registerWord[4] = {0, 0, 0, 0}; 
  unsigned char address         = 0;
  unsigned int  buffer          = 0x0;
  unsigned char i               = 0;
  
  address = AD7190_COMM_READ | AD7190_COMM_ADDR(registerAddress);
  
  HAL_SPI_Transmit(&hspi_AD7190,&address, 1,0xFFFFFF);    
  HAL_SPI_Receive(&hspi_AD7190,registerWord,bytesNumber,0xFFFFFF);
  for(i = 0; i < bytesNumber; i++) 
  {
    buffer = (buffer << 8) + registerWord[i];
  }
  return buffer;
}

/***************************************************************************//**
 * @brief Resets the device.
 *
 * @return none.
*******************************************************************************/
void AD7190_Reset(void)
{
  unsigned char registerWord[7];
  
  registerWord[0] = 0x01;
  registerWord[1] = 0xFF;
  registerWord[2] = 0xFF;
  registerWord[3] = 0xFF;
  registerWord[4] = 0xFF;
  registerWord[5] = 0xFF;
  registerWord[6] = 0xFF;
  HAL_SPI_Transmit(&hspi_AD7190,registerWord, 7,0xFFFFFF);
}

/***************************************************************************//**
 * @brief Checks if the AD7190 part is present.
 *
 * @return status - Indicates if the part is present or not.
*******************************************************************************/
unsigned char AD7190_Init(void)
{
  unsigned char status = 1;
  uint32_t regVal = 0;
  
  MX_AD7190_SPI_Init();  
	
  AD7190_Reset();

  /* Allow at least 500 us before accessing any of the on-chip registers. */
  HAL_Delay(5);
	
  /* Ƭѡʹ�� */
  AD7190_CS_ENABLE();
	
  regVal = AD7190_GetRegisterValue(AD7190_REG_ID, 1);
    printf("ad7190:0x%X\n",regVal);
  if((regVal & AD7190_ID_MASK) != ID_AD7190)
  {
    status = 0;
  }
  return status ;
}

/***************************************************************************//**
 * @brief Set device to idle or power-down.
 *
 * @param pwrMode - Selects idle mode or power-down mode.
 *                  Example: 0 - power-down
 *                           1 - idle
 *
 * @return none.
*******************************************************************************/
void AD7190_SetPower(unsigned char pwrMode)
{
  unsigned int oldPwrMode = 0x0;
  unsigned int newPwrMode = 0x0; 

  oldPwrMode = AD7190_GetRegisterValue(AD7190_REG_MODE, 3);
  oldPwrMode &= ~(AD7190_MODE_SEL(0x7));
  newPwrMode = oldPwrMode | AD7190_MODE_SEL((pwrMode * (AD7190_MODE_IDLE)) | (!pwrMode * (AD7190_MODE_PWRDN)));
  AD7190_SetRegisterValue(AD7190_REG_MODE, newPwrMode, 3);
}

/***************************************************************************//**
 * @brief Waits for RDY pin to go low.
 *
 * @return none.
*******************************************************************************/
void AD7190_WaitRdyGoLow(void)
{
  unsigned int timeOutCnt = 0xFFFFF;

  while(AD7190_RDY_STATE && timeOutCnt--)
  {
      ;
  }
}

/***************************************************************************//**
 * @brief setting continuous read data enable or disable
 *
 * @param cread - continuous read data
 *                 Example: 0 - Disable
 *                          1 - enable
 *
 * @return none.
*******************************************************************************/
void AD7190_Continuous_ReadData(unsigned char cread)
{
  unsigned char registerWord=0;
  
  if(cread==1)
  {
    registerWord=0x5C;
  }
  else
  {
    AD7190_WaitRdyGoLow();
    registerWord=0x58;
  }
  HAL_SPI_Transmit(&hspi_AD7190,&registerWord,1,0xFFFFFF);
}

/***************************************************************************//**
 * @brief Performs the given calibration to the specified channel.
 *
 * @param mode - Calibration type.
 * @param channel - Channel to be calibrated.
 *
 * @return none.
*******************************************************************************/
void AD7190_Calibrate(unsigned char mode, unsigned char channel)
{
  unsigned int oldRegValue = 0x0;
  unsigned int newRegValue = 0x0;
  
  AD7190_ChannelSelect(channel);
  oldRegValue = AD7190_GetRegisterValue(AD7190_REG_MODE, 3);
  oldRegValue &= ~AD7190_MODE_SEL(0x7);
  newRegValue = oldRegValue | AD7190_MODE_SEL(mode);
//    AD7190_CS_ENABLE(); 
  AD7190_SetRegisterValue(AD7190_REG_MODE, newRegValue, 3);
  AD7190_WaitRdyGoLow();
//    AD7190_CS_DISABLE();
}

/***************************************************************************//**
 * @brief Setting chop enable or disable
 *
 * @param chop - chop setting
 *               Example: 0 - Disable
 *                        1 - enable
 *  
 * @return none.
*******************************************************************************/
void AD7190_ChopSetting(unsigned char chop)
{
  unsigned int oldRegValue = 0x0;
  unsigned int newRegValue = 0x0;   
   
  oldRegValue = AD7190_GetRegisterValue(AD7190_REG_CONF, 3);
  if(chop==1)
  {
    newRegValue = oldRegValue | AD7190_CONF_CHOP;
  }
  else
  {
    newRegValue = oldRegValue & (~AD7190_CONF_CHOP); 
  }
  
  AD7190_SetRegisterValue(AD7190_REG_CONF, newRegValue, 3);
}

/***************************************************************************//**
 * @brief Selects the channel to be enabled.
 *
 * @param channel - Selects a channel.
 *  
 * @return none.
*******************************************************************************/
void AD7190_ChannelSelect(unsigned short channel)
{
  unsigned int oldRegValue = 0x0;
  unsigned int newRegValue = 0x0;   
   
  oldRegValue = AD7190_GetRegisterValue(AD7190_REG_CONF, 3);
  oldRegValue &= ~(AD7190_CONF_CHAN(0xFF));
  newRegValue = oldRegValue | AD7190_CONF_CHAN(1 << channel);   
  AD7190_SetRegisterValue(AD7190_REG_CONF, newRegValue, 3);
}

void AD7190_MultiChannelSelect(unsigned short channel)
{
  unsigned int oldRegValue = 0x0;
  unsigned int newRegValue = 0x0;   
   
  oldRegValue = AD7190_GetRegisterValue(AD7190_REG_CONF, 3);
  oldRegValue &= ~(AD7190_CONF_CHAN(0xFF));
  newRegValue = oldRegValue | AD7190_CONF_CHAN(channel);   
  AD7190_SetRegisterValue(AD7190_REG_CONF, newRegValue, 3);
}

/***************************************************************************//**
 * @brief Selects the polarity of the conversion and the ADC input range.
 *
 * @param polarity - Polarity select bit. 
                     Example: 0 - bipolar operation is selected.
                              1 - unipolar operation is selected.
* @param range - Gain select bits. These bits are written by the user to select 
                 the ADC input range.     
 *
 * @return none.
*******************************************************************************/
void AD7190_RangeSetup(unsigned char polarity, unsigned char range)
{
  unsigned int oldRegValue = 0x0;
  unsigned int newRegValue = 0x0;
  
  oldRegValue = AD7190_GetRegisterValue(AD7190_REG_CONF,3);
  oldRegValue &= ~(AD7190_CONF_UNIPOLAR | AD7190_CONF_GAIN(0x7));
  newRegValue = oldRegValue | (polarity * AD7190_CONF_UNIPOLAR) | AD7190_CONF_GAIN(range) | AD7190_CONF_BUF; 
  AD7190_SetRegisterValue(AD7190_REG_CONF, newRegValue, 3);
}

/***************************************************************************//**
 * @brief Returns the result of a single conversion.
 *
 * @return regData - Result of a single analog-to-digital conversion.
*******************************************************************************/
unsigned int AD7190_SingleConversion(void)
{
  unsigned int command = 0x0;
  unsigned int regData = 0x0;

  command = AD7190_MODE_SEL(AD7190_MODE_SINGLE) | AD7190_MODE_CLKSRC(AD7190_CLK_INT) | AD7190_MODE_RATE(0x060);    
//    AD7190_CS_ENABLE(); 
  AD7190_SetRegisterValue(AD7190_REG_MODE, command, 3);
  AD7190_WaitRdyGoLow();
  regData = AD7190_GetRegisterValue(AD7190_REG_DATA, 3);
//    AD7190_CS_DISABLE();
  
  return regData;
}

/***************************************************************************//**
 * @brief Returns the average of several conversion results.
 *
 * @return samplesAverage - The average of the conversion results.
*******************************************************************************/
unsigned int AD7190_ContinuousReadAvg(unsigned char sampleNumber)
{
  unsigned int samplesAverage = 0x0;
  unsigned char count = 0x0;
  unsigned int command = 0x0;
  
  command = AD7190_MODE_SEL(AD7190_MODE_CONT) | AD7190_MODE_CLKSRC(AD7190_CLK_INT) | AD7190_MODE_RATE(0x060);
//    AD7190_CS_ENABLE(); 
  AD7190_SetRegisterValue(AD7190_REG_MODE, command, 3);
  for(count = 0;count < sampleNumber;count ++)
  {
      AD7190_WaitRdyGoLow();
      samplesAverage += AD7190_GetRegisterValue(AD7190_REG_DATA, 3);
  }
//    AD7190_CS_DISABLE();
  samplesAverage = samplesAverage / sampleNumber;
  
  return samplesAverage ;
}

/***************************************************************************//**
 * @brief Read data from temperature sensor and converts it to Celsius degrees.
 *
 * @return temperature - Celsius degrees.
*******************************************************************************/
unsigned int AD7190_TemperatureRead(void)
{
  unsigned char temperature = 0x0;
  unsigned int dataReg = 0x0;
  
  AD7190_RangeSetup(0, AD7190_CONF_GAIN_1);
  AD7190_ChannelSelect(AD7190_CH_TEMP_SENSOR);
  dataReg = AD7190_SingleConversion();
  dataReg -= 0x800000;
  dataReg /= 2815;   // Kelvin Temperature
  dataReg -= 273;    //Celsius Temperature
  temperature = (unsigned int) dataReg;
  
  return temperature;
}

/**
  * ��������: ��·��������AD�ɼ�����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void ad7190_unipolar_multichannel_conf(void)
{
  unsigned int command = 0x0;
  
  // chop enable
  AD7190_ChopSetting(1);
  
  /* Calibrates channel AIN1(+) - AINCOM(-). */
  AD7190_Calibrate(AD7190_MODE_CAL_INT_ZERO, AD7190_CH_AIN1P_AINCOM);
  /* Calibrates channel AIN2(+) - AINCOM(-). */
  AD7190_Calibrate(AD7190_MODE_CAL_INT_ZERO, AD7190_CH_AIN2P_AINCOM);
  /* Calibrates channel AIN3(+) - AINCOM(-). */
  AD7190_Calibrate(AD7190_MODE_CAL_INT_ZERO, AD7190_CH_AIN3P_AINCOM);
  /* Calibrates channel AIN4(+) - AINCOM(-). */
  AD7190_Calibrate(AD7190_MODE_CAL_INT_ZERO, AD7190_CH_AIN4P_AINCOM);
  
  /* Selects unipolar operation and ADC's input range to +-Vref/1. */
  AD7190_RangeSetup(1, AD7190_CONF_GAIN_1);  
  /* Calibrates channel AIN1(+) - AINCOM(-). */
  AD7190_Calibrate(AD7190_MODE_CAL_INT_FULL, AD7190_CH_AIN1P_AINCOM);
  /* Calibrates channel AIN2(+) - AINCOM(-). */
  AD7190_Calibrate(AD7190_MODE_CAL_INT_FULL, AD7190_CH_AIN2P_AINCOM);
  /* Calibrates channel AIN3(+) - AINCOM(-). */
  AD7190_Calibrate(AD7190_MODE_CAL_INT_FULL, AD7190_CH_AIN3P_AINCOM);
  /* Calibrates channel AIN4(+) - AINCOM(-). */
  AD7190_Calibrate(AD7190_MODE_CAL_INT_FULL, AD7190_CH_AIN4P_AINCOM);
     
  // ʹ�ܣ�AD7190_CH_AIN1P_AINCOM
  //       AD7190_CH_AIN2P_AINCOM
  //       AD7190_CH_AIN3P_AINCOM
  //       AD7190_CH_AIN4P_AINCOM
  AD7190_MultiChannelSelect(0xF0);
  
  /* Performs a conversion. */ 
  command = AD7190_MODE_SEL(AD7190_MODE_CONT) | AD7190_MODE_DAT_STA| \
                            AD7190_MODE_CLKSRC(AD7190_CLK_EXT_MCLK1_2) | AD7190_MODE_RATE(500); //�޸�AD�ɼ��ٶȣ�����Ϊ12����Լ0.02s���ʱ��
  AD7190_SetRegisterValue(AD7190_REG_MODE, command, 3);  
  
  AD7190_Continuous_ReadData(1);

}

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
