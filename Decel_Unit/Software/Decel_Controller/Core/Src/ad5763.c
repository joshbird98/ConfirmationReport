/*
 * ad5763.c
 *
 *  Created on: Feb 11, 2023
 *      Author: Josh
 */

#include "ad5763.h"
#include "main.h"
#include "misc_funcs.h"
#include "spi_bare.h"


struct ad5763_device init_ad5763(uint16_t sync_pin, GPIO_TypeDef * sync_port,
		SPI_TypeDef * spi_handle, float ch1_offset, float ch1_gain,
		float ch2_offset, float ch2_gain, uint8_t verbose, char *dev_name, char *ch1_name, char *ch2_name)
{
    struct ad5763_device dac;
	strcpy(dac.dev_name, "AD5763-");
	strcat(dac.dev_name, dev_name);
	strcpy(dac.ch1_name, ch1_name);
	strcpy(dac.ch2_name, ch2_name);
	if (verbose > 0)
	{
		print("Initialising ");
		print(dac.dev_name);
		print(" with channels: ");
		print(dac.ch1_name);
		print(" and ");
		println(dac.ch2_name);
	}
    dac.sync_pin = sync_pin;
    dac.sync_pin_num = mylog2(sync_pin);
    dac.sync_port = sync_port;
    dac.spi_handle = spi_handle;

    dac.ch1_offset = ch1_offset;
    dac.ch1_gain = ch1_gain;
    dac.ch1_out_uV = 0;
    dac.ch1_stage_out_uV = (dac.ch1_gain * dac.ch1_out_uV) + dac.ch1_offset;

    dac.ch2_offset = ch2_offset;
    dac.ch2_gain = ch2_gain;
    dac.ch2_out_uV = 0;
    dac.ch2_stage_out_uV = (dac.ch2_gain * dac.ch2_out_uV) + dac.ch2_offset;

    dac.temp_C = 25.0;

    if (verbose > 0)
    {
    	print(dac.dev_name);
    	println(" initialised\n");
    }

  return dac;
}

// calculates the stage output voltage needed to attain the high-voltage output
// sends this value to be set
void set_ultravolt_mV(struct ad5763_device *dac, uint8_t channel, float hv_output_V, uint8_t verbose)
{
	if (verbose > 1)
	{
		print("Requesting high voltage output of ");
		printfloat(hv_output_V, 0);
		println("mV");
	}
	set_stage_uV(dac, channel, hv_output_V / 4.0, verbose);
}

void set_HP_mV(struct ad5763_device *dac, uint8_t channel, float hv_output_V, uint8_t verbose)
{
	if (verbose > 1)
	{
		print("Requesting high voltage output of ");
		printfloat(hv_output_V, 0);
		println("mV");
	}
	set_stage_uV(dac, channel, hv_output_V / 2.0, verbose);
}

// calcualtes the dac output voltage needed to attain the stage output voltage
// sends this value to be set
// updates struct output information
void set_stage_uV(struct ad5763_device *dac, uint8_t channel, float stage_target_uV, uint8_t verbose)
{
	if (verbose > 1)
	{
		print("Requesting stage output of ");
		print(dac->dev_name);
		print(" channel ");
		printuint8_t(channel, 0);
		print(" to ");
		printfloat(stage_target_uV, 0);
		println("uV");
	}

	//convert stage target uV to dac output uV
	float dac_output_target_uV;

	if (channel == 1)
	{
		dac_output_target_uV = (stage_target_uV - (1000000 * dac->ch1_offset)) / dac->ch1_gain;
		if (dac_output_target_uV > 4096000) dac_output_target_uV = 4096000;
		if (dac_output_target_uV < -4096000) dac_output_target_uV = -4096000;
		set_output_uV(dac, channel, dac_output_target_uV, verbose);
		dac->ch1_stage_out_uV = (dac->ch1_out_uV * dac->ch1_gain) + (1000000 * dac->ch1_offset);

	}

	else if (channel == 2)
	{
		dac_output_target_uV = (stage_target_uV - (1000000 * dac->ch2_offset)) / dac->ch2_gain;
		if (dac_output_target_uV > 4096000) dac_output_target_uV = 4096000;
		if (dac_output_target_uV < -4096000) dac_output_target_uV = -4096000;
		set_output_uV(dac, channel, dac_output_target_uV, verbose);
		dac->ch2_stage_out_uV = (dac->ch2_out_uV * dac->ch2_gain) + (1000000 * dac->ch2_offset);
	}

	if (verbose > 1)
	{
		if ((channel == 1) || (channel == 2))
		{
			print("Stage output of ");
			if (channel == 1) printfloat(dac->ch1_stage_out_uV, 0);
			if (channel == 2) printfloat(dac->ch2_stage_out_uV, 0);
			println("uV achieved.");
		}
		else print("DAC channel not recognised.");
	}
}


// calculates the dac input bits needed to attain the dac output voltage
// sends this data to be set
// updates struct output information
void set_output_uV(struct ad5763_device *dac, uint8_t channel, float target_uV, uint8_t verbose)
{

	//convert uV to bits
	//uV target value should be float between -4096000 and + 4096000
	uint8_t clamped = 0;
	if (target_uV > 4095875)
	{
		target_uV = 4095875;
		clamped = 1;
	}
	else if (target_uV < -4096000)
	{
		target_uV = -4096000;
		clamped = 1;
	}

	if (verbose > 1)
	{
		print("Requesting voltage output of DAC ");
		print(dac->dev_name);
		print(" Channel ");
		printuint8_t(channel, 0);
		print(" to ");
		printfloat(target_uV, 0);
		println("uV");
		if (clamped == 1) println("This value has been limited by DAC output range.");
	}

	float bits = ((target_uV / 125.0) + 32767.0);
	if (target_uV >=0) bits += 1;

	set_output_bits(dac, channel, (uint16_t)bits);

	if (channel == 1)
	{
		dac->ch1_out_uV = target_uV;
	}
	else if (channel == 2)
	{
		dac->ch2_out_uV = target_uV;
	}

	if (verbose > 1)
	{
		if ((channel == 1) || (channel == 2))
		{
			print("Output of ");
			printuint16_t((uint16_t)bits, 0);
			print(" bits, ie ");
			if (channel == 1) printfloat(dac->ch1_out_uV, 0);
			if (channel == 2) printfloat(dac->ch2_out_uV, 0);
			println(" uV achieved.");
		}
		else println("Channel not recognised.");
	}
}

// literally just sends the right SPI command
void set_output_bits(struct ad5763_device *dac, uint8_t channel, uint16_t bits)
{
	uint8_t data_tx[3] = {0,0,0};
	data_tx[0] = (2 << 3) | (channel - 1);
	data_tx[1] = bits >> 8;
	data_tx[2] = (bits & 0xFF);

	// Inititate SPI transfer
	setupSPIAD5763(dac);
	SPI_Transmit((uint8_t *)&data_tx, 1, dac->spi_handle);			// send 1 dummy byte
	dac->sync_port->BSRR |= (1<<(dac->sync_pin_num))<<16; 			// set CS LOW
	SPI_Transmit((uint8_t *)&data_tx, 3, dac->spi_handle);			// send 3 bytes
	dac->sync_port->BSRR |= (1<<(dac->sync_pin_num)); 				// reset CS HIGH
	returnSPIAD5763(dac);
}


void set_offset_bits(struct ad5763_device *dac, uint8_t channel, uint8_t bits)
{
	print("Sending data ");
	printuint8_t(bits, 1);
	uint8_t data_tx[3] = {0,0,0};
	data_tx[0] = (5 << 3) | (channel - 1);
	data_tx[1] = bits; //don't care
	data_tx[2] = bits;

	// Inititate SPI transfer
	setupSPIAD5763(dac);
	SPI_Transmit((uint8_t *)&data_tx, 1, dac->spi_handle);			// send 1 dummy byte
	dac->sync_port->BSRR |= (1<<(dac->sync_pin_num))<<16; 			// set CS LOW
	SPI_Transmit((uint8_t *)&data_tx, 3, dac->spi_handle);			// send 3 bytes
	dac->sync_port->BSRR |= (1<<(dac->sync_pin_num)); 				// reset CS HIGH
	returnSPIAD5763(dac);
}

void set_gainfine_bits(struct ad5763_device *dac, uint8_t channel, uint8_t bits)
{
	print("Sending data ");
	printuint8_t(bits, 1);
	uint8_t data_tx[3] = {0,0,0};
	data_tx[0] = (4 << 3) | (channel - 1);
	data_tx[1] = bits; //don't care
	data_tx[2] = (0x3F & bits);

	// Inititate SPI transfer
	setupSPIAD5763(dac);
	SPI_Transmit((uint8_t *)&data_tx, 1, dac->spi_handle);			// send 1 dummy byte
	dac->sync_port->BSRR |= (1<<(dac->sync_pin_num))<<16; 			// set CS LOW
	SPI_Transmit((uint8_t *)&data_tx, 3, dac->spi_handle);			// send 3 bytes
	dac->sync_port->BSRR |= (1<<(dac->sync_pin_num)); 				// reset CS HIGH
	returnSPIAD5763(dac);
}

uint16_t readback_ad5763(struct ad5763_device *dac, uint8_t reg, uint8_t ch)
{

	uint8_t data_tx[3] = {0,0,0};
	uint8_t data_rx[3] = {0,0,0};
	data_tx[0] |= (1<<7) | ((reg & 7)<<3) | ((ch - 1) & 7);


	// Inititate SPI transfer
	setupSPIAD5763(dac);
	dac->sync_port->BSRR |= (1<<(dac->sync_pin_num))<<16; 			// set CS LOW
	SPI_Transmit((uint8_t *)&data_tx, 3, dac->spi_handle);			// send 3 bytes
	dac->sync_port->BSRR |= (1<<(dac->sync_pin_num)); 				// reset CS HIGH

	for (uint8_t i = 0; i < 100; i++) asm("NOP");

	dac->sync_port->BSRR |= (1<<(dac->sync_pin_num))<<16; 			// set CS LOW
	SPI_Receive((uint8_t *)&data_rx, 3, dac->spi_handle);			// receive 3 bytes
	dac->sync_port->BSRR |= (1<<(dac->sync_pin_num)); 				// reset CS HIGH
	returnSPIAD5763(dac);

	uint16_t val = (data_rx[0] << 16) | data_rx[1] << 8 | data_rx[2];
	return val;
}


void setupSPIAD5763(struct ad5763_device *dac)
{
	// Setup SPI for this device, ie slow and Mode 1
	SPIMode(dac->spi_handle, 1);
	SPISpeed(dac->spi_handle, 1000000); //1MHz
	for(uint8_t i = 0; i < 50; i++) asm("NOP");
}

void returnSPIAD5763(struct ad5763_device *dac)
{
	// Return SPI to fast and mode 3
	SPIMode(dac->spi_handle, 3);
	SPISpeed(dac->spi_handle, 10000000); //10MHz
}

