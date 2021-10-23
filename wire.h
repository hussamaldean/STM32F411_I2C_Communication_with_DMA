#ifndef __wire__h
#define __wire__h
#include "stm32f4xx.h"                  // Device header
#include <stddef.h>
void i2c1_init(void);
void i2c_rx_dma_init(void);
void i2c_tx_dma_init(void);
void I2C_write(uint8_t SensorAddr,uint8_t * pWriteBuffer, uint16_t NumByteToWrite);
void I2C_Read(uint8_t SensorAddr, uint8_t ReadAddr,uint8_t * pReadBuffer, uint16_t NumByteToRead);


#endif