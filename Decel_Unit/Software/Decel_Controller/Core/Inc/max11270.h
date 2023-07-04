/*
 * max11270.h
 *
 *  Created on: Feb 9, 2023
 *      Author: Josh
 */

#ifndef SRC_MAX11270_H_
#define SRC_MAX11270_H_

#include "stdint.h"
#include "main.h"
#include "max6225.h"

struct max11270_device {
  char name[40];
  uint16_t cs_pin;
  uint16_t cs_pin_num;
  GPIO_TypeDef* cs_port;
  uint16_t rdy_pin;
  GPIO_TypeDef* rdy_port;
  uint16_t rdy_pin_num;
  SPI_TypeDef * spi_handle;
  uint32_t result_bits;
  float result_uV;
  uint16_t last_stat;
  uint8_t speed;
  uint8_t fault;
};

struct max11270_device init_max11270(uint16_t cs_pin, GPIO_TypeDef* cs_port,
		uint16_t rdy_pin, GPIO_TypeDef* rdy_port, uint8_t speed, SPI_TypeDef * spi_handle, uint8_t verbose, char *name);

//calibrate MAX11270
void calibrate_max11270(struct max11270_device *adc, uint8_t verbose);

//read from MAX11270
uint32_t read_max11270(struct max11270_device *adc, struct max6225_device *vref, uint8_t speed, uint8_t verbose);

//setup MAX11270
void setup_max11270(struct max11270_device *adc, uint8_t verbose);

float max11270_bits_to_uV(uint32_t bits, struct max6225_device vref);

uint16_t stat_max11270(struct max11270_device *adc, uint8_t verbose);

uint8_t check_available_max11270(struct max11270_device *adc);

void cont_conversion(struct max11270_device *adc, uint8_t verbose, uint8_t speed);

void setupSPIMAX11270(struct max11270_device *adc);
void returnSPIMAX11270(struct max11270_device *adc);

#endif /* SRC_MAX11270_H_ */
