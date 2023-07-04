/*
 * ad5763.h
 *
 *  Created on: Feb 11, 2023
 *      Author: Josh
 */

#ifndef SRC_AD5763_H_
#define SRC_AD5763_H_

#include "stdint.h"
#include "main.h"

struct ad5763_device {
  uint16_t sync_pin;
  uint16_t sync_pin_num;
  GPIO_TypeDef* sync_port;
  SPI_TypeDef * spi_handle;
  char dev_name[20];
  char ch1_name[20];
  char ch2_name[20];
  float ch1_offset;
  float ch1_gain;
  float ch1_out_uV;
  float ch1_stage_out_uV;
  float ch2_offset;
  float ch2_gain;
  float ch2_out_uV;
  float ch2_stage_out_uV;
  float temp_C;
};

/*
 * ad5763.c
 *
 *  Created on: Feb 11, 2023
 *      Author: Josh
 */

#include "ad5763.h"
#include "main.h"
#include "misc_funcs.h"


struct ad5763_device init_ad5763(uint16_t sync_pin, GPIO_TypeDef * sync_port,
		SPI_TypeDef * spi_handle, float ch1_offset, float ch1_gain,
		float ch2_offset, float ch2_gain, uint8_t verbose, char *dev_name, char *ch1_name, char *ch2_name);
void set_ultravolt_mV(struct ad5763_device *dac, uint8_t channel, float hv_output_V, uint8_t verbose);
void set_HP_mV(struct ad5763_device *dac, uint8_t channel, float hv_output_V, uint8_t verbose);
void set_stage_uV(struct ad5763_device *dac, uint8_t channel, float stage_target_uV, uint8_t verbose);
void set_output_uV(struct ad5763_device *dac, uint8_t channel, float target_uV, uint8_t verbose);
void set_output_bits(struct ad5763_device *dac, uint8_t channel, uint16_t bits);
void set_offset_bits(struct ad5763_device *dac, uint8_t channel, uint8_t bits);
void set_gainfine_bits(struct ad5763_device *dac, uint8_t channel, uint8_t bits);
uint16_t readback_ad5763(struct ad5763_device *dac, uint8_t reg, uint8_t ch);
void setupSPIAD5763(struct ad5763_device *dac);
void returnSPIAD5763(struct ad5763_device *dac);
#endif /* SRC_AD5763_H_ */
