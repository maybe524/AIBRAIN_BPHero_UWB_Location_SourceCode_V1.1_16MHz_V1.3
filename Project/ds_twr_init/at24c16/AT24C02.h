
#include "stm32f10x_i2c.h"

#ifndef AT24C02_H_
#define AT24C02_H_

void AT24C02_Write(I2C_TypeDef* I2Cx, unsigned char AddressDevice, unsigned char AddressByte, unsigned char Value);
unsigned char AT24C02_Read(I2C_TypeDef* I2Cx, unsigned char AddressDevice, unsigned char AddressByte);

/* 
 * AT24C02 2kb = 2048bit = 2048/8 B = 256 B
 * 32 pages of 8 bytes each
 *
 * Device Address
 * 1 0 1 0 A2 A1 A0 R/W
 * 1 0 1 0 0  0  0  0 = 0XA0
 * 1 0 1 0 0  0  0  1 = 0XA1 
 */
 
/* EEPROM Addresses defines */
#define EEPROM_Block0_ADDRESS   0xA0 /* E2 = 0 */
//#define EEPROM_Block1_ADDRESS 0xA2 /* E2 = 0 */
//#define EEPROM_Block2_ADDRESS 0xA4 /* E2 = 0 */
//#define EEPROM_Block3_ADDRESS 0xA6 /* E2 = 0 */

void I2C_EE_BufferWrite(u8* pBuffer, u8 WriteAddr, u16 NumByteToWrite);
void I2C_EE_ByteWrite(u8* pBuffer, u8 WriteAddr);
void I2C_EE_PageWrite(u8* pBuffer, u8 WriteAddr, u8 NumByteToWrite);
void I2C_EE_BufferRead(u8* pBuffer, u8 ReadAddr, u16 NumByteToRead);
void I2C_EE_WaitEepromStandbyState(void);

#endif /* AT24C02_H_ */
