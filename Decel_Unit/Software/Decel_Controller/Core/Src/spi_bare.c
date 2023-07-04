/*
 * spi_bare.c , see https://controllerstech.com/spi-using-registers-in-stm32/ and RM0090 Reference manual
 *
 *  Created on: March 17, 2023
 *      Author: Josh
 *
 *  Library allows setup and use of SPI1 and SPI2, with limited options
 *  Usage:
	SPIConfig();
	SPI_Enable();
	cs enable
	SPI_Receive() or SPI_Transfer()
	cs_disable
 */

#include "spi_bare.h"
#include "main.h"
#include "misc_funcs.h"

/************** STEPS TO FOLLOW *****************
1. Enable SPI clock
2. Configure the Control Register 1
3. Configure the CR2
************************************************/
void SPIConfig(void)
{
	GPIOConfigSPI(); 				// Setup GPIO
	RCC->APB2ENR |= (1<<12);  		// Enable SPI1 CLock, max 84MHz
	RCC->APB1ENR |= (1<<14);  		// Enable SPI2 Clock, max 42MHz

	SPIMode(SPI1, 0);				// Default to Mode 3
	SPIMode(SPI2, 0);				// Default to Mode 3

	SPI1->CR1 |= (1<<2);  			// Set SPI to Master Mode
	SPI2->CR1 |= (1<<2);  			// Set SPI to Master Mode

	SPISpeed(SPI1, 1000000);		// Default to 10MHz
	SPISpeed(SPI2, 1000000);		// Default to 10MHz

	SPI1->CR1 &= ~(1<<7);  			// LSBFIRST = 0, MSB first
	SPI2->CR1 &= ~(1<<7);  			// LSBFIRST = 0, MSB first

	SPI1->CR1 |= (1<<8) | (1<<9);  	// SSM=1, SSi=1 -> Software Slave Management
	SPI2->CR1 |= (1<<8) | (1<<9);  	// SSM=1, SSi=1 -> Software Slave Management

	SPI1->CR1 &= ~(1<<10);  		// RXONLY = 0, full-duplex
	SPI2->CR1 &= ~(1<<10);  		// RXONLY = 0, full-duplex

	SPI1->CR1 &= ~(1<<11);  		// DFF=0, 8 bit data
	SPI2->CR1 &= ~(1<<11);  		// DFF=0, 8 bit data

	SPI1->CR2 = 0;					// CR2 is used during SPI operation
	SPI2->CR2 = 0;					// CR2 is used during SPI operation
}

// Ensures GPIO pins PB3..5 are set up for SPI1 and PB13..15 for SPI2
void GPIOConfigSPI(void)
{
	RCC->AHB1ENR   |= (1<<0);  					// Enable GPIO Clock

	// Setting up pins for SPI1 on PB3..5
	GPIOB->MODER   |= (2<<6)|(2<<8)|(2<<10);  	// Alternate function mode for PB3, PB4, PB5
	GPIOB->OSPEEDR |= (3<<6)|(3<<8)|(3<<10);  	// Very High Speed for PB3, PB5 (SCLK and MOSI)
	GPIOB->AFR[0]  |= (5<<12)|(5<<16)|(5<<20);  // AF5 (SPI1) for PB3, PB4, PB5
	GPIOB->PUPDR   &= ~((1<<6)|(1<<7));			// No pull-ups or pull-downs on PB3
	GPIOB->PUPDR   &= ~((1<<8)|(1<<9));			// No pull-ups or pull-downs on PB4
	GPIOB->PUPDR   &= ~((1<<10)|(1<<11));		// No pull-ups or pull-downs on PB5

	// Setting up pins for SPI2 on PB13..15
	GPIOB->MODER   |= (2<<26)|(2<<28)|(2<<30);  // Alternate function mode for PB13, PB14, PB15
	GPIOB->OSPEEDR |= (3<<26)|(3<<28)|(3<<30);  // Very High Speed for PB13, PB15 (SCLK and MOSI)
	GPIOB->AFR[1]  |= (5<<20)|(5<<24)|(5<<28);  // AF5 (SPI2) for PB13, PB14, PB15
	GPIOB->PUPDR   &= ~((1<<26)|(1<<27));		// No pull-ups or pull-downs on PB13
	GPIOB->PUPDR   &= ~((1<<28)|(1<<29));		// No pull-ups or pull-downs on PB14
	GPIOB->PUPDR   &= ~((1<<30)|(1<<31));		// No pull-ups or pull-downs on PB15
}

// Determines and sets CPHA and CPOL for mode (0,1,2 or 3)
void SPIMode(SPI_TypeDef *spi, uint8_t mode)
{
	mode &= 0x3;
	if (mode == 0)
	{
		spi->CR1 &= ~((1<<0)|(1<<1));
	}
	else if (mode == 1)
	{
		spi->CR1 |= (1<<0);
		spi->CR1 &= ~(1<<1);
	}
	else if (mode == 2)
	{
		spi->CR1 &= ~(1<<0);
		spi->CR1 |= (1<<1);
	}
	else if (mode == 3)
	{
		spi->CR1 |=  (1<<0)|(1<<1);
	}
}

// Determines and set optimal baud rate divider to attain SCLK speed up to max_freq
void SPISpeed(SPI_TypeDef *spi, uint32_t max_freq)
{
	uint32_t clk = 0;
	if (spi == SPI1) clk = SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1)>> RCC_CFGR_PPRE1_Pos];
	if (spi == SPI2) clk = SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE2)>> RCC_CFGR_PPRE2_Pos];

	// available SPI speeds are clk/2, clk/4, clk/8 etc...
	uint8_t br_div = 0;
	while (((clk >> (br_div + 1)) > max_freq) && (br_div < 8))
	{
		br_div += 1;
	}
	SPIBRDiv(spi, br_div);
}

// For SPI1, SPI clk = 84MHz / (2 ^ (br_div + 1)
// For SPI2, SPI clk = 42MHz / (2 ^ (br_div + 1)

void SPIBRDiv(SPI_TypeDef *spi, uint8_t br_div)
{
	spi->CR1 |= ((br_div & 7) << 3);
	spi->CR1 &= ~((~br_div & 7) << 3);
}

void SPI_Enable (SPI_TypeDef *spi)
{
	spi->CR1 |= (1<<6);   // SPE=1, Peripheral enabled
}

void SPI_Disable (SPI_TypeDef *spi)
{
	spi->CR1 &= ~(1<<6);   // SPE=0, Peripheral disabled
}

//SPI_TypeDef *spi should be either SPI1 or SPI2
void SPI_Receive (uint8_t *data, int size, SPI_TypeDef *spi)
{
	/************** STEPS TO FOLLOW *****************
	1. Wait for the BSY bit to reset in Status Register
	2. Send some Dummy data before reading the DATA
	3. Wait for the RXNE bit to Set in the status Register
	4. Read data from Data Register
	************************************************/

	while (size)
	{
		while (((spi->SR)&(1<<7))) {};  // wait for BSY bit to Reset -> This will indicate that SPI is not busy in communication
		spi->DR = 0;  // send dummy data
		while (!((spi->SR) &(1<<0))){};  // Wait for RXNE to set -> This will indicate that the Rx buffer is not empty
	    *data++ = (spi->DR);
		size--;
	}
}

void SPI_Transmit (uint8_t *data, int size, SPI_TypeDef *spi)
{

	/************** STEPS TO FOLLOW *****************
	1. Wait for the TXE bit to set in the Status Register
	2. Write the data to the Data Register
	3. After the data has been transmitted, wait for the BSY bit to reset in Status Register
	4. Clear the Overrun flag by reading DR and SR
	************************************************/
	int i=0;
	while (i<size)
	{
	   while (!((spi->SR)&(1<<1))) {};  // wait for TXE bit to set -> This will indicate that the buffer is empty
	   spi->DR = data[i];  // load the data into the Data Register
	   i++;
	}


	/*During discontinuous communications, there is a 2 APB clock period delay between the
	write operation to the SPI_DR register and BSY bit setting. As a consequence it is
	mandatory to wait first until TXE is set and then until BSY is cleared after writing the last
	data.
	*/
	while (!((spi->SR)&(1<<1))) {};  // wait for TXE bit to set -> This will indicate that the buffer is empty
	while (((spi->SR)&(1<<7))) {};  // wait for BSY bit to Reset -> This will indicate that SPI is not busy in communication

	//  Clear the Overrun flag by reading DR and SR
	uint8_t temp = spi->DR;
	temp = spi->SR;
	temp = temp; //shuts the compiler up
}

// quickest way to control outputs... (ie fast CS changes)
// setup as output and very-high speed, then change single register bit
//	 GPIOx->OSPEEDR |= (3<<y)
//	 GPIOx->MODER |= (1<<y)
//   GPIOx->BSRR |= (1<<g)<<16 to set to LOW
//   GPIOx->BSRR |= (1<<g) to reset to HIGH


