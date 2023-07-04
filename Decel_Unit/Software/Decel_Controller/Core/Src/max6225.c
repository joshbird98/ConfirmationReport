/*
 * max6225.c
 *
 *  Created on: Feb 11, 2023
 *      Author: Josh
 */

#include "max6225.h"
#include "main.h"
#include "misc_funcs.h"


struct max6225_device init_max6225(uint8_t verbose)
{
	if (verbose > 0) println("Initialising MAX6225");
    struct max6225_device vref;
    vref.nominal_output = 2500000.0;
    vref.nominal_temp = 300;
	vref.temp_co = 0;
	vref.output = (float)vref.nominal_output;
    if (verbose > 0) println("MAX6225 initialised");
  return vref;
}

float temp_comp_vref(struct max6225_device vref, float temp_K)
{
	vref.output = (float)vref.nominal_output + (vref.temp_co * (temp_K - vref.nominal_temp));
	return vref.output;
}
