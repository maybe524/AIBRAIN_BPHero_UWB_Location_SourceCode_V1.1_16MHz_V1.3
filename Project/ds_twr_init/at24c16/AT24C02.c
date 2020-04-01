#include "AT24C02.h"
#include "stm32f10x_i2c.h"

void AT24C02_Write(I2C_TypeDef* I2Cx, unsigned char AddressDevice, unsigned char AddressByte, unsigned char Value)
{
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
	I2C_AcknowledgeConfig(I2Cx, ENABLE);

	I2C_GenerateSTART(I2Cx, ENABLE);
	while( !I2C_GetFlagStatus(I2Cx, I2C_FLAG_SB) ); // wait Generate Start

	I2C_Send7bitAddress(I2Cx, AddressDevice, I2C_Direction_Transmitter);
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) ); // wait send Address

	I2C_SendData(I2Cx, AddressByte);
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // wait send Address Byte

	I2C_SendData(I2Cx, Value);
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // wait Send Value for Byte

	I2C_GenerateSTOP(I2Cx, ENABLE);
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF)); // wait Generate Stop
}

unsigned char AT24C02_Read(I2C_TypeDef* I2Cx, unsigned char AddressDevice, unsigned char AddressByte)
{
	unsigned char ReceiveData = 0;

	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
	I2C_GenerateSTART(I2Cx, ENABLE);
	while( !I2C_GetFlagStatus(I2Cx, I2C_EVENT_MASTER_MODE_SELECT) ); // wait Generate Start

	I2C_Send7bitAddress(I2Cx, AddressDevice, I2C_Direction_Transmitter);
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) ); // wait send Address Device

	I2C_SendData(I2Cx, AddressByte);
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED) ); // wait send Address Byte

	I2C_GenerateSTART(I2Cx, ENABLE);
	while( !I2C_GetFlagStatus(I2Cx, I2C_EVENT_MASTER_MODE_SELECT) ); // wait Generate Start

	I2C_Send7bitAddress(I2Cx, AddressDevice, I2C_Direction_Receiver);
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) ); // wait Send Address Device As Receiver

	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) ); // wait Receive a Byte
	ReceiveData = I2C_ReceiveData(I2Cx);

	I2C_NACKPositionConfig(I2Cx, I2C_NACKPosition_Current); // send not acknowledge
	I2C_AcknowledgeConfig(I2Cx, DISABLE);// disable acknowledge

	I2C_GenerateSTOP(I2Cx, ENABLE);
	while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF)); // wait Generate Stop Condition

	I2C_AcknowledgeConfig(I2Cx, DISABLE);// disable acknowledge

	return ReceiveData;
}
/* STM32 I2C ???? */
#define I2C_Speed              400000
 
/* ???????STM32???I2C????????? */
#define I2C1_OWN_ADDRESS7      0X0A   
 
/* AT24C01/02???8??? */
#define I2C_PageSize           8
 
/* AT24C04/08A/16A???16??? */
//#define I2C_PageSize           16			
 
uint16_t EEPROM_ADDRESS = EEPROM_Block0_ADDRESS;

/**
  * @brief   ??????????I2C EEPROM?
  * @param   
  *		@arg pBuffer:?????
  *		@arg WriteAddr:???
  *     @arg NumByteToWrite:?????
  * @retval  ?
  */
void I2C_EE_BufferWrite(u8* pBuffer, u8 WriteAddr, u16 NumByteToWrite)
{
  u8 NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0;
 
  Addr = WriteAddr % I2C_PageSize;              // addr = 8bit % 8
  count = I2C_PageSize - Addr;                  // 8 - addr
  NumOfPage =  NumByteToWrite / I2C_PageSize;
  NumOfSingle = NumByteToWrite % I2C_PageSize;
 
  /* If WriteAddr is I2C_PageSize aligned  */
  if(Addr == 0) 
  {
    /* If NumByteToWrite < I2C_PageSize */
    if(NumOfPage == 0) 
    {
      I2C_EE_PageWrite(pBuffer, WriteAddr, NumOfSingle);
      I2C_EE_WaitEepromStandbyState();
    }
    /* If NumByteToWrite > I2C_PageSize */
    else  
    {
      while(NumOfPage--)
      {
        I2C_EE_PageWrite(pBuffer, WriteAddr, I2C_PageSize); 
    	I2C_EE_WaitEepromStandbyState();
        WriteAddr +=  I2C_PageSize;
        pBuffer += I2C_PageSize;
      }
 
      if(NumOfSingle!=0)
      {
        I2C_EE_PageWrite(pBuffer, WriteAddr, NumOfSingle);
        I2C_EE_WaitEepromStandbyState();
      }
    }
  }
  /* If WriteAddr is not I2C_PageSize aligned  */
  else 
  {
    /* If NumByteToWrite < I2C_PageSize */
    if(NumOfPage== 0) 
    {
      I2C_EE_PageWrite(pBuffer, WriteAddr, NumOfSingle);
      I2C_EE_WaitEepromStandbyState();
    }
    /* If NumByteToWrite > I2C_PageSize */
    else
    {
      NumByteToWrite -= count;
      NumOfPage =  NumByteToWrite / I2C_PageSize;
      NumOfSingle = NumByteToWrite % I2C_PageSize;	
      
      if(count != 0)
      {  
        I2C_EE_PageWrite(pBuffer, WriteAddr, count);
        I2C_EE_WaitEepromStandbyState();
        WriteAddr += count;
        pBuffer += count;
      } 
      
      while(NumOfPage--)
      {
        I2C_EE_PageWrite(pBuffer, WriteAddr, I2C_PageSize);
        I2C_EE_WaitEepromStandbyState();
        WriteAddr +=  I2C_PageSize;
        pBuffer += I2C_PageSize;  
      }
      if(NumOfSingle != 0)
      {
        I2C_EE_PageWrite(pBuffer, WriteAddr, NumOfSingle); 
        I2C_EE_WaitEepromStandbyState();
      }
    }
  }  
}
 
/**
  * @brief   ??????I2C EEPROM?
  * @param   
  *		@arg pBuffer:?????
  *		@arg WriteAddr:??? 
  * @retval  ?
  */
void I2C_EE_ByteWrite(u8* pBuffer, u8 WriteAddr)
{
  /* Send STRAT condition */
  I2C_GenerateSTART(I2C1, ENABLE);
 
  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));  
 
  /* Send EEPROM address for write */
  I2C_Send7bitAddress(I2C1, EEPROM_ADDRESS, I2C_Direction_Transmitter);
  
  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
      
  /* Send the EEPROM's internal address to write to */
  I2C_SendData(I2C1, WriteAddr);
  
  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
 
  /* Send the byte to be written */
  I2C_SendData(I2C1, *pBuffer); 
   
  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  
  /* Send STOP condition */
  I2C_GenerateSTOP(I2C1, ENABLE);
}
 
/**
  * @brief   ?EEPROM??????????????,?????????
  *          ????EEPROM????,AT24C02???8???
  * @param   
  *		@arg pBuffer:?????
  *		@arg WriteAddr:???
  *     @arg NumByteToWrite:?????
  * @retval  ?
  */
void I2C_EE_PageWrite(u8* pBuffer, u8 WriteAddr, u8 NumByteToWrite)
{
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)); // Added by Najoua 27/08/2008
    
  /* ??start?? */
  I2C_GenerateSTART(I2C1, ENABLE);
  
  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)); 
  
  /* Send EEPROM address for write */
  I2C_Send7bitAddress(I2C1, EEPROM_ADDRESS, I2C_Direction_Transmitter);
  
  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));  
 
  /* Send the EEPROM's internal address to write to */    
  I2C_SendData(I2C1, WriteAddr);  
 
  /* Test on EV8 and clear it */
  while(! I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
 
  /* While there is data to be written */
  while(NumByteToWrite--)  
  {
    /* Send the current byte */
    I2C_SendData(I2C1, *pBuffer); 
 
    /* Point to the next byte to be written */
    pBuffer++; 
  
    /* Test on EV8 and clear it */
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  }
 
  /* Send STOP condition */
  I2C_GenerateSTOP(I2C1, ENABLE);
}
 
/**
  * @brief   ?EEPROM???????? 
  * @param   
  *		@arg pBuffer:???EEPROM???????????
  *		@arg WriteAddr:?????EEPROM???
  *     @arg NumByteToWrite:??EEPROM??????
  * @retval  ?
  */
void I2C_EE_BufferRead(u8* pBuffer, u8 ReadAddr, u16 NumByteToRead)
{  
  //*((u8 *)0x4001080c) |=0x80; 
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)); // Added by Najoua 27/08/2008    
    
  /* Send START condition */
  I2C_GenerateSTART(I2C1, ENABLE);
  //*((u8 *)0x4001080c) &=~0x80;
  
  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
 
  /* Send EEPROM address for write */
  I2C_Send7bitAddress(I2C1, EEPROM_ADDRESS, I2C_Direction_Transmitter);
 
  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
  
  /* Clear EV6 by setting again the PE bit */
  I2C_Cmd(I2C1, ENABLE);
 
  /* Send the EEPROM's internal address to write to */
  I2C_SendData(I2C1, ReadAddr);  
 
  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  
  /* Send STRAT condition a second time */  
  I2C_GenerateSTART(I2C1, ENABLE);
  
  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
  
  /* Send EEPROM address for read */
  I2C_Send7bitAddress(I2C1, EEPROM_ADDRESS, I2C_Direction_Receiver);
  
  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
  
  /* While there is data to be read */
  while(NumByteToRead)  
  {
    if(NumByteToRead == 1)
    {
      /* Disable Acknowledgement */
      I2C_AcknowledgeConfig(I2C1, DISABLE);
      
      /* Send STOP Condition */
      I2C_GenerateSTOP(I2C1, ENABLE);
    }
 
    /* Test on EV7 and clear it */
    if(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))  
    {      
      /* Read a byte from the EEPROM */
      *pBuffer = I2C_ReceiveData(I2C1);
 
      /* Point to the next location where the byte read will be saved */
      pBuffer++; 
      
      /* Decrement the read bytes counter */
      NumByteToRead--;        
    }   
  }
 
  /* Enable Acknowledgement to be ready for another reception */
  I2C_AcknowledgeConfig(I2C1, ENABLE);
}
 
/**
  * @brief  Wait for EEPROM Standby state 
  * @param  ?
  * @retval ?
  */
void I2C_EE_WaitEepromStandbyState(void)      
{
  vu16 SR1_Tmp = 0;
 
  do
  {
    /* Send START condition */
    I2C_GenerateSTART(I2C1, ENABLE);
    /* Read I2C1 SR1 register */
    SR1_Tmp = I2C_ReadRegister(I2C1, I2C_Register_SR1);
    /* Send EEPROM address for write */
    I2C_Send7bitAddress(I2C1, EEPROM_ADDRESS, I2C_Direction_Transmitter);
  }while(!(I2C_ReadRegister(I2C1, I2C_Register_SR1) & 0x0002));
  
  /* Clear AF flag */
  I2C_ClearFlag(I2C1, I2C_FLAG_AF);
    /* STOP condition */    
    I2C_GenerateSTOP(I2C1, ENABLE); 
}