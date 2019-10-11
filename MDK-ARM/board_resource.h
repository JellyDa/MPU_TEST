#ifndef __BOARD_RESOURCE_H
#define __BOARD_RESOURCE_H

#include "stm32f4xx.h"
#include "stm32f4xx_hal_usart.h" 
#include "stm32f4xx_hal_rcc.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "arch.h"

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"
#include "mltypes.h"
#include "mpu.h"
#include "log.h"
#include "packet.h"

#define USART_ReceiveData(__USARTx__) (__USARTx__)->Instance->DR & (uint16_t)0x01FF

#define WAIT_FOR_FLAG(flag, value, timeout, errorcode) I2CTimeout = (timeout); \
          while(__HAL_I2C_GET_FLAG(&SENSORS_I2C, (flag)) != (value)) {\
            if((I2CTimeout--) == 0) return I2Cx_TIMEOUT_UserCallback((errorcode)); \
          }
  
#define CLEAR_ADDR_BIT      I2C_ReadRegister(&SENSORS_I2C, I2C_Register_SR1);\
                            I2C_ReadRegister(&SENSORS_I2C, I2C_Register_SR2);

int get_tick_count(unsigned long *count);
void mdelay(unsigned long nTime);

void Set_I2C_Retry(unsigned short ml_sec);
unsigned short Get_I2C_Retry(void);

static uint32_t I2Cx_TIMEOUT_UserCallback(char value);
uint16_t I2C_ReadRegister(I2C_HandleTypeDef* I2Cx, uint8_t I2C_Register);
void I2C_SoftwareResetCmd(I2C_HandleTypeDef I2Cx, FunctionalState NewState);
void I2C_GenerateSTART(I2C_HandleTypeDef* I2Cx, FunctionalState NewState);
void I2C_GenerateSTOP(I2C_HandleTypeDef* I2Cx, FunctionalState NewState);
void I2C_Send7bitAddress(I2C_HandleTypeDef* I2Cx, uint8_t Address, uint8_t I2C_Direction);
void I2C_AcknowledgeConfig(I2C_HandleTypeDef* I2Cx, FunctionalState NewState);

int Sensors_I2C_ReadRegister(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, unsigned char *RegisterValue);
int Sensors_I2C_WriteRegister(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, const unsigned char *RegisterValue);

unsigned long ST_Sensors_I2C_WriteRegister(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, const unsigned char *RegisterValue);
unsigned long ST_Sensors_I2C_ReadRegister(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, unsigned char *RegisterValue);

#endif	/* __BOARD_RESOURCE_H */
