#include "board_resource.h"
#include "stdio.h"

uint8_t UART_RxCplt = 0, UART_RxData = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	UART_RxCplt = 1;
	HAL_UART_Receive_IT(&USB_UART, (uint8_t*)&UART_RxData, 1);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_1)
		gyro_data_ready_cb();
}


/**
  * @brief  Clears the USARTx's interrupt pending bits.
  * @param  USARTx: where x can be 1, 2, 3, 4, 5 or 6 to select the USART or 
  *         UART peripheral.
  * @param  USART_IT: specifies the interrupt pending bit to clear.
  *          This parameter can be one of the following values:
  *            @arg USART_IT_CTS:  CTS change interrupt (not available for UART4 and UART5)
  *            @arg USART_IT_LBD:  LIN Break detection interrupt
  *            @arg USART_IT_TC:   Transmission complete interrupt. 
  *            @arg USART_IT_RXNE: Receive Data register not empty interrupt.
  *
  * @note   PE (Parity error), FE (Framing error), NE (Noise error), ORE (OverRun 
  *          error) and IDLE (Idle line detected) pending bits are cleared by 
  *          software sequence: a read operation to USART_SR register 
  *          (USART_GetITStatus()) followed by a read operation to USART_DR register 
  *          (USART_ReceiveData()).
  * @note   RXNE pending bit can be also cleared by a read to the USART_DR register 
  *          (USART_ReceiveData()).
  * @note   TC pending bit can be also cleared by software sequence: a read 
  *          operation to USART_SR register (USART_GetITStatus()) followed by a write 
  *          operation to USART_DR register (USART_SendData()).
  * @note   TXE pending bit is cleared only by a write to the USART_DR register 
  *          (USART_SendData()).
  *  
  * @retval None
  */

void mdelay(unsigned long nTime)
{
	HAL_Delay(nTime);
}

int get_tick_count(unsigned long *count)
{
  count[0] = HAL_GetTick();
	return 0;
}

/**
  * @brief  Generates I2Cx communication START condition.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  NewState: new state of the I2C START condition generation.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None.
  */
void I2C_GenerateSTART(I2C_HandleTypeDef* I2Cx, FunctionalState NewState)
{
  if (NewState != DISABLE)
  {
    /* Generate a START condition */
    I2Cx->Instance->CR1 |= I2C_CR1_START;
  }
  else
  {
    /* Disable the START condition generation */
    I2Cx->Instance->CR1 &= (uint16_t)~((uint16_t)I2C_CR1_START);
  }
}

/**
  * @brief  Generates I2Cx communication STOP condition.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  NewState: new state of the I2C STOP condition generation.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None.
  */
void I2C_GenerateSTOP(I2C_HandleTypeDef* I2Cx, FunctionalState NewState)
{
  /* Check the parameters */
  if (NewState != DISABLE)
  {
    /* Generate a STOP condition */
    I2Cx->Instance->CR1 |= I2C_CR1_STOP;
  }
  else
  {
    /* Disable the STOP condition generation */
    I2Cx->Instance->CR1 &= (uint16_t)~((uint16_t)I2C_CR1_STOP);
  }
}

/**
  * @brief  Enables or disables the specified I2C software reset.
  * @note   When software reset is enabled, the I2C IOs are released (this can
  *         be useful to recover from bus errors).  
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  NewState: new state of the I2C software reset.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void I2C_SoftwareResetCmd(I2C_HandleTypeDef I2Cx, FunctionalState NewState)
{
  /* Check the parameters */
  if (NewState != DISABLE)
  {
    /* Peripheral under reset */
    I2Cx.Instance->CR1 |= I2C_CR1_SWRST;
  }
  else
  {
    /* Peripheral not under reset */
    I2Cx.Instance->CR1 &= (uint16_t)~((uint16_t)I2C_CR1_SWRST);
  }
}

/**
  * @brief  Reads the specified I2C register and returns its value.
  * @param  I2C_Register: specifies the register to read.
  *          This parameter can be one of the following values:
  *            @arg I2C_Register_CR1:  CR1 register.
  *            @arg I2C_Register_CR2:   CR2 register.
  *            @arg I2C_Register_OAR1:  OAR1 register.
  *            @arg I2C_Register_OAR2:  OAR2 register.
  *            @arg I2C_Register_DR:    DR register.
  *            @arg I2C_Register_SR1:   SR1 register.
  *            @arg I2C_Register_SR2:   SR2 register.
  *            @arg I2C_Register_CCR:   CCR register.
  *            @arg I2C_Register_TRISE: TRISE register.
  * @retval The value of the read register.
  */
uint16_t I2C_ReadRegister(I2C_HandleTypeDef* I2Cx, uint8_t I2C_Register)
{
  __IO uint32_t tmp = 0;

  /* Check the parameters */

  tmp = (uint32_t) I2Cx;
  tmp += I2C_Register;

  /* Return the selected register value */
  return (*(__IO uint16_t *) tmp);
}

/**
  * @brief  Basic management of the timeout situation.
  * @param  None.
  * @retval None.
  */
static uint32_t I2Cx_TIMEOUT_UserCallback(char value)
{
  /* Generate a STOP condition */
  I2C_GenerateSTOP(&SENSORS_I2C, ENABLE);
  I2C_SoftwareResetCmd(SENSORS_I2C, ENABLE);
  I2C_SoftwareResetCmd(SENSORS_I2C, DISABLE);
  
  HAL_I2C_DeInit(&hi2c2);
	
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
	
  return 1;
}

void I2C_Send7bitAddress(I2C_HandleTypeDef* I2Cx, uint8_t Address, uint8_t I2C_Direction)
{
	/* Test on the direction to set/reset the read/write bit */
  if (I2C_Direction != I2C_Direction_Transmitter)
  {
    /* Set the address bit0 for read */
    Address |= I2C_OAR1_ADD0;
  }
  else
  {
    /* Reset the address bit0 for write */
    Address &= (uint8_t)~((uint8_t)I2C_OAR1_ADD0);
  }
  /* Send the address */
  I2Cx->Instance->DR = Address;
}

/**
  * @brief  Enables or disables the specified I2C acknowledge feature.
  * @param  I2Cx: where x can be 1, 2 or 3 to select the I2C peripheral.
  * @param  NewState: new state of the I2C Acknowledgement.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None.
  */
void I2C_AcknowledgeConfig(I2C_HandleTypeDef* I2Cx, FunctionalState NewState)
{
  if (NewState != DISABLE)
  {
    /* Enable the acknowledgement */
    I2Cx->Instance->CR1 |= I2C_CR1_ACK;
  }
  else
  {
    /* Disable the acknowledgement */
    I2Cx->Instance->CR1 &= (uint16_t)~((uint16_t)I2C_CR1_ACK);
  }
}

int Sensors_I2C_WriteRegister(unsigned char slave_addr,
                                        unsigned char reg_addr,
                                        unsigned short len, 
                                        const unsigned char *data_ptr)
{
  char retries=0;
  int ret = 0;
  unsigned short retry_in_mlsec = Get_I2C_Retry();
                              
tryWriteAgain:  
  ret = 0;
  ret = ST_Sensors_I2C_WriteRegister( slave_addr, reg_addr, len, data_ptr); 

  if(ret && retry_in_mlsec)
  {
    if( retries++ > 4 )
		{
			printf("aaaaa");
        return ret;
		}
    
    HAL_Delay(retry_in_mlsec);
    goto tryWriteAgain;
  }
  return ret;  
}

int Sensors_I2C_ReadRegister(unsigned char slave_addr,
                                       unsigned char reg_addr,
                                       unsigned short len, 
                                       unsigned char *data_ptr)
{
  char retries=0;
  int ret = 0;
  unsigned short retry_in_mlsec = Get_I2C_Retry();
  
tryReadAgain:  
  ret = 0;
  ret = ST_Sensors_I2C_ReadRegister( slave_addr, reg_addr, len, data_ptr);

  if(ret && retry_in_mlsec)
  {
    if( retries++ > 4 )
        return ret;
    
    HAL_Delay(retry_in_mlsec);
    goto tryReadAgain;
  } 
  return ret;
}

unsigned long ST_Sensors_I2C_WriteRegister(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, const unsigned char *RegisterValue)
{
  uint32_t  result = 0;
  uint32_t  i = 0; // i = RegisterLen;
  __IO uint32_t  I2CTimeout = I2Cx_LONG_TIMEOUT;
  
//  RegisterValue = RegisterValue + (RegisterLen - 1);

  /* Wait for the busy flag to be cleared */
  WAIT_FOR_FLAG (I2C_FLAG_BUSY, RESET, I2Cx_LONG_TIMEOUT, 1);
  
  /* Start the config sequence */
  I2C_GenerateSTART(&SENSORS_I2C, ENABLE);

  /* Wait for the start bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_SB, SET, I2Cx_FLAG_TIMEOUT, 2);

  /* Transmit the slave address and enable writing operation */
  I2C_Send7bitAddress(&SENSORS_I2C, (Address<<1), I2C_Direction_Transmitter);
  
  /* Wait for address bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_ADDR, SET, I2Cx_FLAG_TIMEOUT, 3);
  
  /* clear the ADDR interrupt bit  - this is done by reading SR1 and SR2*/
  CLEAR_ADDR_BIT
  
  /* Wait for address bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_TXE, SET, I2Cx_FLAG_TIMEOUT, 4);

  /* Transmit the first address for write operation */
  I2C_SendData(&SENSORS_I2C, RegisterAddr);

//  while (i--)
//  {
//    /* Wait for address bit to be set */
//    WAIT_FOR_FLAG (I2C_FLAG_TXE, SET, I2Cx_FLAG_TIMEOUT, 5);
//  
//    /* Prepare the register value to be sent */
//    I2C_SendData(SENSORS_I2C, *RegisterValue--);
//  }
  
  for(i=0; i<(RegisterLen); i++)
  {
    /* Wait for address bit to be set */
    WAIT_FOR_FLAG (I2C_FLAG_TXE, SET, I2Cx_FLAG_TIMEOUT, 5);
  
    /* Prepare the register value to be sent */
    I2C_SendData(&SENSORS_I2C, RegisterValue[i]);
  }  
   
  /* Wait for address bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_BTF, SET, I2Cx_FLAG_TIMEOUT, 6);
  
  /* End the configuration sequence */
  I2C_GenerateSTOP(&SENSORS_I2C, ENABLE);  
  
  /* Return the verifying value: 0 (Passed) or 1 (Failed) */
  return result;  
}

unsigned long ST_Sensors_I2C_ReadRegister(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, unsigned char *RegisterValue)
{
  uint32_t i=0, result = 0;
  __IO uint32_t  I2CTimeout = I2Cx_LONG_TIMEOUT;
   
  /* Wait for the busy flag to be cleared */
  WAIT_FOR_FLAG(I2C_FLAG_BUSY, RESET, I2Cx_LONG_TIMEOUT, 7);
  
  /* Start the config sequence */
  I2C_GenerateSTART(&SENSORS_I2C, ENABLE);

  /* Wait for the start bit to be set */
  WAIT_FOR_FLAG(I2C_FLAG_SB, SET, I2Cx_FLAG_TIMEOUT, 8);
  
  /* Transmit the slave address and enable writing operation */
  I2C_Send7bitAddress(&SENSORS_I2C, (Address<<1), I2C_Direction_Transmitter);

  /* Wait for the start bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_ADDR, SET, I2Cx_FLAG_TIMEOUT, 9);

  /* clear the ADDR interrupt bit  - this is done by reading SR1 and SR2*/
  CLEAR_ADDR_BIT;
  
  /* Wait for address bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_TXE, SET, I2Cx_FLAG_TIMEOUT, 10);
  
  /* Transmit the register address to be read */
  I2C_SendData(&SENSORS_I2C, RegisterAddr);
  
  /* Wait for address bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_TXE, SET, I2Cx_FLAG_TIMEOUT, 11);  

  /*!< Send START condition a second time */  
  I2C_GenerateSTART(&SENSORS_I2C, ENABLE);
  
  /* Wait for the start bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_SB, SET, I2Cx_FLAG_TIMEOUT, 12);
  
  /*!< Send address for read */
  I2C_Send7bitAddress(&SENSORS_I2C, (Address<<1), I2C_Direction_Receiver);  
  
  /* Wait for the start bit to be set */
  WAIT_FOR_FLAG (I2C_FLAG_ADDR, SET, I2Cx_FLAG_TIMEOUT, 13);
  
  if (RegisterLen == 1) 
  {
    /*!< Disable Acknowledgment */
    I2C_AcknowledgeConfig(&SENSORS_I2C, DISABLE);
    
    /* clear the ADDR interrupt bit  - this is done by reading SR1 and SR2*/
    CLEAR_ADDR_BIT;
    
    /*!< Send STOP Condition */
    I2C_GenerateSTOP(&SENSORS_I2C, ENABLE);
    
    /* Wait for the RXNE bit to be set */
    WAIT_FOR_FLAG (I2C_FLAG_RXNE, SET, I2Cx_FLAG_TIMEOUT, 14);
    
    RegisterValue[0] = I2C_ReceiveData(&SENSORS_I2C);
  } 
  else if( RegisterLen == 2) 
  {
     /*!< Disable Acknowledgment */
    I2C_AcknowledgeConfig(&SENSORS_I2C, DISABLE);
    
   /* Set POS bit */ 
   SENSORS_I2C.Instance->CR1 |= I2C_CR1_POS;
   
   /* clear the ADDR interrupt bit  - this is done by reading SR1 and SR2*/
   CLEAR_ADDR_BIT; 
   
   /* Wait for the buffer full bit to be set */
   WAIT_FOR_FLAG (I2C_FLAG_BTF, SET, I2Cx_FLAG_TIMEOUT, 15);
   
   /*!< Send STOP Condition */
   I2C_GenerateSTOP(&SENSORS_I2C, ENABLE);

   /* Read 2 bytes */
   RegisterValue[0] = I2C_ReceiveData(&SENSORS_I2C);
   RegisterValue[1] = I2C_ReceiveData(&SENSORS_I2C);
  } 
  else if( RegisterLen == 3)
  {
    CLEAR_ADDR_BIT;
    
    /* Wait for the buffer full bit to be set */
    WAIT_FOR_FLAG (I2C_FLAG_BTF, SET, I2Cx_FLAG_TIMEOUT, 16);
    /*!< Disable Acknowledgment */
    I2C_AcknowledgeConfig(&SENSORS_I2C, DISABLE);
    /* Read 1 bytes */
    RegisterValue[0] = I2C_ReceiveData(&SENSORS_I2C);
    /*!< Send STOP Condition */
    I2C_GenerateSTOP(&SENSORS_I2C, ENABLE);        
    /* Read 1 bytes */
    RegisterValue[1] = I2C_ReceiveData(&SENSORS_I2C);
    /* Wait for the buffer full bit to be set */
    WAIT_FOR_FLAG (I2C_FLAG_RXNE, SET, I2Cx_FLAG_TIMEOUT, 17);
    /* Read 1 bytes */
    RegisterValue[2] = I2C_ReceiveData(&SENSORS_I2C);  
  }  
  else /* more than 2 bytes */
  { 
    /* clear the ADDR interrupt bit  - this is done by reading SR1 and SR2*/
    CLEAR_ADDR_BIT;
    
    for(i=0; i<(RegisterLen); i++)
    {
      if(i==(RegisterLen-3))
      {
        /* Wait for the buffer full bit to be set */
        WAIT_FOR_FLAG (I2C_FLAG_BTF, SET, I2Cx_FLAG_TIMEOUT, 16);
        
        /*!< Disable Acknowledgment */
        I2C_AcknowledgeConfig(&SENSORS_I2C, DISABLE);
        
        /* Read 1 bytes */
        RegisterValue[i++] = I2C_ReceiveData(&SENSORS_I2C);
        
        /*!< Send STOP Condition */
        I2C_GenerateSTOP(&SENSORS_I2C, ENABLE);        
        
        /* Read 1 bytes */
        RegisterValue[i++] = I2C_ReceiveData(&SENSORS_I2C);
        
        /* Wait for the buffer full bit to be set */
        WAIT_FOR_FLAG (I2C_FLAG_RXNE, SET, I2Cx_FLAG_TIMEOUT, 17);
        
        /* Read 1 bytes */
        RegisterValue[i++] = I2C_ReceiveData(&SENSORS_I2C);  
        goto endReadLoop;
      }
            
      /* Wait for the RXNE bit to be set */
      WAIT_FOR_FLAG (I2C_FLAG_RXNE, SET, I2Cx_FLAG_TIMEOUT, 18);
      RegisterValue[i] = I2C_ReceiveData(&SENSORS_I2C); 
    }   
  } 
  
endReadLoop:  
  /* Clear BTF flag */
  __HAL_I2C_CLEAR_FLAG(&SENSORS_I2C, I2C_FLAG_BTF);
  /* Wait for the busy flag to be cleared */
  WAIT_FOR_FLAG (I2C_FLAG_BUSY, RESET, I2Cx_LONG_TIMEOUT, 19);  
  /*!< Re-Enable Acknowledgment to be ready for another reception */
  I2C_AcknowledgeConfig(&SENSORS_I2C, ENABLE);
  //Disable POS -- TODO
  SENSORS_I2C.Instance->CR1 &= ~I2C_CR1_POS;  
     
  /* Return the byte read from sensor */
  return result;
}

static unsigned short RETRY_IN_MLSEC  = 5; // 5 = 101

void Set_I2C_Retry(unsigned short ml_sec)
{
  RETRY_IN_MLSEC = ml_sec;
}

unsigned short Get_I2C_Retry(void)
{
  return RETRY_IN_MLSEC;
}
