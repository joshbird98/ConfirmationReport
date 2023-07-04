/*
 * mcp33131.h
 *
 *  Created on: Feb 10, 2023
 *      Author: Josh
 */

#ifndef SRC_MCP33131_H_
#define SRC_MCP33131_H_

#include "main.h"
#include "max6225.h"

#define CAL_TIME 650	//maximum time for calibration to complete, in ms
#define CNV_TIME 750	//maximum time for conversoin to complete, in ns

struct mcp33131_device {
  char name[20];
  uint16_t cs_pin;
  uint16_t cs_pin_num;
  GPIO_TypeDef* cs_port;
  uint16_t sdo_pin;
  GPIO_TypeDef* sdo_port;
  SPI_TypeDef * spi_handle;
  uint16_t result_bits;
  float result_uV;
  uint32_t cal_start_time;
  uint8_t cal_active;
  uint32_t last_clock;
  uint16_t min_clk_cycles;
};

struct mcp33131_device init_mcp33131(uint16_t cs_pin, GPIO_TypeDef * cs_port, SPI_TypeDef * spi_handle, uint8_t verbose, char *name);
void calibrate_mcp33131(struct mcp33131_device *adc, uint8_t verbose);
uint8_t check_available_mcp33131(struct mcp33131_device *adc);
uint16_t read_mcp33131(struct mcp33131_device *adc, struct max6225_device *vref, uint8_t verbose);
float mcp33131_bits_to_uV(uint16_t bits, struct max6225_device vref);

#endif /* SRC_MCP33131_H_ */
