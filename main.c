#include "debug.h"
#include "wire.h"
volatile int finished=0;
int read_finish();
void reset_finish();
uint8_t data[3];
//note: Index zero shall always contains the starting memory address
uint8_t data_write[4]={0x00,0x10,0x12,0x00};

int main(void)
	{
	i2c1_init();
	i2c_rx_dma_init();
	i2c_tx_dma_init();
	I2C_write(0x68,data_write,sizeof(data_write));
	while(read_finish()==0){;}
	reset_finish();
	while(1)
		{
			I2C_Read(0x68,0x00,data,3);
			while(read_finish()==0){;}
			reset_finish();
		for(volatile int i=0;i<100000;i++);
		}
		
	
	}
	
	
int read_finish()
	{
	return finished;
	}
	
void reset_finish()
	{
	finished=0;
	}

void DMA1_Stream5_IRQHandler(void)
			{
			
			if((DMA1->HISR)&DMA_HISR_TCIF5)
					{
					finished=1;
					log_debug("I2C finished receiving using DMA1_Stream5");
					I2C1->CR1 |= I2C_CR1_STOP;
					DMA1->HIFCR=DMA_HIFCR_CTCIF5;
					}
			if((DMA1->HISR)&DMA_HISR_HTIF5)
					{
					log_debug("DMA1 stream5 half transfer interrupt");
					DMA1->HIFCR=DMA_HIFCR_CHTIF5;
					}
					
			if((DMA1->HISR)&DMA_HISR_TEIF5)
					{
					log_debug("DMA1 stream5 error");
					DMA1->HIFCR=DMA_HIFCR_CTEIF5;
					}
			}
			
void DMA1_Stream6_IRQHandler(void)
			{
			
			if((DMA1->HISR)&DMA_HISR_TCIF6)
					{
			
					log_debug("I2C finished transmiting using DMA1_Stream6");
					finished=1;
					I2C1->CR1 |= I2C_CR1_STOP;
					DMA1->HIFCR=DMA_HIFCR_CTCIF6;
						
					}
			if((DMA1->HISR)&DMA_HISR_HTIF6)
					{
					log_debug("DMA1 stream6 half transfer interrupt");
					DMA1->HIFCR=DMA_HIFCR_CHTIF6;
					}
					
			if((DMA1->HISR)&DMA_HISR_TEIF6)
					{
					log_debug("DMA1 stream6 error");
					DMA1->HIFCR=DMA_HIFCR_CTEIF6;
					}
			
			}
