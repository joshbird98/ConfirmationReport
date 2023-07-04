/*
 * max6225.h
 *
 *  Created on: Feb 11, 2023
 *      Author: Josh
 */

#ifndef SRC_MAX6225_H_
#define SRC_MAX6225_H_

#include "stdint.h"
#include "main.h"

struct max6225_device {
  float nominal_output; 	// microVolts
  int16_t nominal_temp;		// Kelvin
  int16_t temp_co;			// microVolts_per_Kelvin
  float output;				// microVolts
};

struct max6225_device init_max6225(uint8_t verbose);

float temp_comp_vref(struct max6225_device, float temp_K);

#endif /* SRC_AD5763_H_ */
