/*
 * spi_bare.h
 *
 *  Created on: March 17, 2023
 *      Author: Josh
 */

#ifndef SRC_SPI_BARE_H_
#define SRC_SPI_BARE_H_

#include "stdint.h"
#include "main.h"

/*
 * spi_bare.c , see https://controllerstech.com/spi-using-registers-in-stm32/
 *
 *  Created on: March 17, 2023
 *      Author: Josh
 *
 *  Both SPI1 and SPI2 must be controlled, therefore each function must be passed this
 */


#include "spi_bare.h"
#include "main.h"

void SPIConfig(void);
void GPIOConfigSPI (void);
void SPIMode(SPI_TypeDef *spi, uint8_t mode);
void SPISpeed(SPI_TypeDef *spi, uint32_t max_freq);
void SPIBRDiv(SPI_TypeDef *spi, uint8_t br_div);
void SPI_Enable (SPI_TypeDef *spi);
void SPI_Disable (SPI_TypeDef *spi);
void SPI_Receive (uint8_t *data, int size, SPI_TypeDef *spi);
void SPI_Transmit (uint8_t *data, int size, SPI_TypeDef *spi);

#endif /* SRC_SPI_BARE_H_ */
