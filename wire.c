#include "wire.h"
#include "debug.h"
/**
DMA1_Stream5_channel 1 is I2C1_RX
DMA1_Stream6_channel 1 is I2C1_TX
*/

#define PB8_ALT (1<<17)
#define PB9_ALT (1<<19)
#define I2C_AF4 (0x04)

#define ch1 (1<<25)

/**
 * @brief   Initialize I2C1 
 * @note    PB8 is SCL, PB9 is SDA
 * @param   None
 * @retval  None
 */

void i2c1_init(void)
		{
		RCC->AHB1ENR|=RCC_AHB1ENR_GPIOBEN; //enable gpiob clock
		RCC->APB1ENR|=RCC_APB1ENR_I2C1EN; //enable i2c1 clock
		GPIOB->MODER|=PB8_ALT|PB9_ALT; //set PB8 and PB9 to alternative function
		GPIOB->AFR[1]|=(I2C_AF4<<0)|(I2C_AF4<<4);
		GPIOB->OTYPER|=GPIO_OTYPER_OT8|GPIO_OTYPER_OT9;
		I2C1->CR1=I2C_CR1_SWRST;//reset i2c
		I2C1->CR1&=~I2C_CR1_SWRST;// release reset i2c	
		I2C1->CR1 &=~ I2C_CR1_NOSTRETCH;//disable clock strech
		I2C1->CR1 &= ~I2C_CR1_ENGC;
		I2C1->CR2 |= I2C_CR2_LAST;
		I2C1->CR2 |= I2C_CR2_DMAEN;
		I2C1->CR2|=16;//set clock source to 16MHz
		I2C1->CCR=80;  //based on calculation
		I2C1->TRISE=17; //output max rise 
		I2C1->CR1 |=I2C_CR1_PE;
		}
		
	
	
	void i2c_rx_dma_init(void)
		{
		RCC->AHB1ENR|=RCC_AHB1ENR_DMA1EN;
		DMA1_Stream5->CR=0x00;//reset everything
		while((DMA1_Stream5->CR)&DMA_SxCR_EN){;}
		DMA1_Stream5->CR|=ch1|DMA_SxCR_MINC|DMA_SxCR_TCIE|DMA_SxCR_HTIE|DMA_SxCR_TEIE;
		NVIC_EnableIRQ(DMA1_Stream5_IRQn);
		}
		
/**
 * @brief   Initialize DMA1_Stream6 
 * @note    CH3 for I2C1
 * @param   None
 * @retval  None
 */	
		void i2c_tx_dma_init(void)
		{
		RCC->AHB1ENR|=RCC_AHB1ENR_DMA1EN;
		DMA1_Stream6->CR=0x00;//reset everything
		while((DMA1_Stream6->CR)&DMA_SxCR_EN){;}
		DMA1_Stream6->CR|=ch1|DMA_SxCR_MINC|DMA_SxCR_DIR_0|DMA_SxCR_TCIE|DMA_SxCR_HTIE|DMA_SxCR_TEIE;
		
		NVIC_EnableIRQ(DMA1_Stream6_IRQn);	
		}
		
		/**
 * @brief   DMA data transmit
 * @note    I2C1_TX -> DMA1, Stream 6, Channel 3
 * @param   pBuffer, size
 * @retval  None
 */
		
static void DMA_Transmit(const uint8_t * pBuffer, uint8_t size)
{
	
  /* Check null pointers */
  if(NULL != pBuffer)
  {
    DMA1_Stream6->CR&=~DMA_SxCR_EN;
	while((DMA1_Stream6->CR)&DMA_SxCR_EN){;}

    /* Set memory address */
    DMA1_Stream6->M0AR = (uint32_t)pBuffer;
		DMA1_Stream6->PAR=(uint32_t)&I2C1->DR;
    /* Set number of data items */
    DMA1_Stream6->NDTR = size;

    /* Clear all interrupt flags */
    DMA1->HIFCR = (DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CTEIF6
        | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTCIF6);

    /* Enable DMA1_Stream4 */
    DMA1_Stream6->CR |= DMA_SxCR_EN;
  }
  else
  {
    /* Null pointers, do nothing */
  }
}

/**
 * @brief   DMA data receive
 * @note    I2C1_RX -> DMA1, Stream 5, Channel 3
 * @param   pBuffer, size
 * @retval  None
 */
static void DMA_Receive(const uint8_t * pBuffer, uint8_t size)
{
	
  /* Check null pointers */
  if(NULL != pBuffer)
  {
    DMA1_Stream5->CR&=~DMA_SxCR_EN;
	while((DMA1_Stream5->CR)&DMA_SxCR_EN){;}

    /* Set memory address */
    DMA1_Stream5->M0AR = (uint32_t)pBuffer;
		DMA1_Stream5->PAR=(uint32_t)&I2C1->DR;
    /* Set number of data items */
    DMA1_Stream5->NDTR = size;

    /* Clear all interrupt flags */
    DMA1->HIFCR = ( DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CTEIF5
        | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTCIF5);

    /* Enable DMA1_Stream2 */
    DMA1_Stream5->CR |= DMA_SxCR_EN;
  }
  else
  {
    /* Null pointers, do nothing */
  }
}		
		
		
/**
 * @brief   Read register data
 * @note
 * @param   SensorAddr, ReadAddr, pReadBuffer, NumByteToRead
 * @retval  None
 */		

void I2C_Read(uint8_t SensorAddr, uint8_t ReadAddr,
				uint8_t * pReadBuffer, uint16_t NumByteToRead)
		{
			//wait until the bus is free
			while(I2C1->SR2&I2C_SR2_BUSY){;}
			
			/* Generate START */
			I2C1->CR1 |= I2C_CR1_START;
			/* Wait SB flag is set */
			while(!(I2C1->SR1&I2C_SR1_SB)){;}

			/* Read SR1 */
			(void)I2C1->SR1;

			/* Send slave address with write */
			I2C1->DR=(SensorAddr<<1|0);

			/* Wait ADDR flag is set */
			while(((I2C1->SR1)&I2C_SR1_ADDR)==0){;}
			/* Read SR1 */
			(void)I2C1->SR1;

			/* Read SR2 */
			(void)I2C1->SR2;

			/* Wait TXE flag is set */
			while(I2C_SR1_TXE != (I2C_SR1_TXE & I2C1->SR1))
			{
				/* Do nothing */
			}

			if(2 <= NumByteToRead)
			{
				/* Acknowledge enable */
				I2C1->CR1 |= I2C_CR1_ACK;

				/* Send register address to read with increment */
				I2C1->DR =  (ReadAddr);
			}
			else
			{
				/* Acknowledge disable */
				I2C1->CR1 &= ~I2C_CR1_ACK;

				/* Send register address to read (single) */
				I2C1->DR =  ReadAddr;
				
			}



			/* Wait BTF flag is set */
			while(!(I2C_SR1_BTF & I2C1->SR1))
			{
				/* Do nothing */
			}

			/* Generate ReSTART */
			I2C1->CR1 |= I2C_CR1_START;

			/* Wait SB flag is set */
			while(I2C_SR1_SB != (I2C_SR1_SB & I2C1->SR1))
			{
				/* Do nothing */
			}

			/* Read SR1 */
			(void)I2C1->SR1;

			/* Send slave address with read */
			I2C1->DR =  (SensorAddr<<1 | (uint8_t)0x01);

			/* Wait ADDR flag is set */
			while(((I2C1->SR1)&I2C_SR1_ADDR)==0){;}
			

			/* Start DMA */
			DMA_Receive(pReadBuffer, NumByteToRead);

			/* Read SR1 */
			(void)I2C1->SR1;

			/* Read SR2 */
			(void)I2C1->SR2;
		}
	
/**
 * @brief   Write register data
 * @note
 * @param   SensorAddr, pWriteBuffer, NumByteToWrite
 * @retval  None
 */
		
void I2C_write(uint8_t SensorAddr,
    uint8_t * pWriteBuffer, uint16_t NumByteToWrite)
{
	
		while(I2C1->SR2&I2C_SR2_BUSY){;}
	
		/* Generate START */
		I2C1->CR1 |= I2C_CR1_START;

  /* Wait SB flag is set */
		while(!(I2C1->SR1&I2C_SR1_SB)){;}
  /* Read SR1 */
		(void)I2C1->SR1;

  /* Send slave address with write */
			I2C1->DR = (SensorAddr<<1);

  /* Wait ADDR flag is set */
			while(((I2C1->SR1)&I2C_SR1_ADDR)==0){;}
  

  /* Start DMA */
		DMA_Transmit(pWriteBuffer, NumByteToWrite);

  /* Read SR1 */
			(void)I2C1->SR1;

  /* Read SR2 */
			(void)I2C1->SR2;
}		