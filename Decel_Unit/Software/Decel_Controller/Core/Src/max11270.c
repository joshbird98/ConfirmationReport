/*
 * max11270.c
 *
 *  Created on: Feb 9, 2023
 *      Author: Josh
 */

#include "max11270.h"
#include "main.h"
#include "misc_funcs.h"
#include "spi_bare.h"

//max11270 stuff
const uint8_t MAX11270_REG_WRITE = 0b11000000;
const uint8_t MAX11270_REG_CTRL1_WRITE = 0b11000010;
const uint8_t MAX11270_REG_CTRL1_READ = 0b11000011;
const uint8_t MAX11270_REG_READ = 0b11000001;
const uint8_t MAX11270_READ_DATA = 0b11001101;
const uint8_t MAX11270_CMD = 0b10000000;
const uint8_t MAX11270_CAL = 0b10100000;

struct max11270_device init_max11270(uint16_t cs_pin, GPIO_TypeDef* cs_port,
		uint16_t rdy_pin, GPIO_TypeDef* rdy_port, uint8_t speed, SPI_TypeDef * spi_handle, uint8_t verbose, char *name)
{
    struct max11270_device adc;
	strcpy(adc.name, "MAX11270-");
    strcat(adc.name, name);
	if (verbose > 0)
	{
		print("Initialising ");
		println(adc.name);
	}
    adc.cs_pin = cs_pin;
    adc.cs_pin_num = mylog2(cs_pin);
    adc.cs_port = cs_port;
    adc.rdy_pin = rdy_pin;
    adc.rdy_pin_num = mylog2(rdy_pin);
    adc.rdy_port = rdy_port;
    adc.spi_handle = spi_handle;
    adc.result_bits = 0;
    adc.result_uV = 0;
    adc.fault = 0;
    if (speed > 0x0F) speed = 0x0F;
    adc.speed = speed;

	//ensure CS port is active
	if (adc.cs_port == GPIOA) RCC->AHB1ENR |= (1<<0);  // Enable the GPIOA clock
	if (adc.cs_port == GPIOB) RCC->AHB1ENR |= (1<<1);  // Enable the GPIOB clock
	if (adc.cs_port == GPIOC) RCC->AHB1ENR |= (1<<2);  // Enable the GPIOC clock
	if (adc.cs_port == GPIOD) RCC->AHB1ENR |= (1<<3);  // Enable the GPIOD clock
	if (adc.cs_port == GPIOE) RCC->AHB1ENR |= (1<<4);  // Enable the GPIOE clock
	if (adc.cs_port == GPIOF) RCC->AHB1ENR |= (1<<5);  // Enable the GPIOF clock
	if (adc.cs_port == GPIOG) RCC->AHB1ENR |= (1<<6);  // Enable the GPIOG clock
	if (adc.cs_port == GPIOH) RCC->AHB1ENR |= (1<<7);  // Enable the GPIOH clock

	// ensuring CS pin is very-high speed push-pull output, with no-pulldown
	adc.cs_port->MODER 	 |=  (1<<(adc.cs_pin_num << 1)); 	// Output mode for CS pin
	adc.cs_port->OTYPER  &= ~(1<<(adc.cs_pin_num));		    // Push-pull mode for CS pin
	adc.cs_port->OSPEEDR |=  ((0<<(adc.cs_pin_num << 1)) | (0<<((adc.cs_pin_num << 1)+1)));    // Low Speed for CS pin
	adc.cs_port->PUPDR   &= ~((1<<(adc.cs_pin_num << 1)) | (1<<((adc.cs_pin_num << 1)+1)));  // No pullup or pulldown for CS pin
	adc.cs_port->BSRR    |= (1<<(adc.cs_pin_num)); 			// Resets CS HIGH
	for (uint8_t i = 0; i < 100; i++) asm("NOP");				// Delay to ensure CS high meets timing requirements

	//ensure RDY port is active
	if (adc.rdy_port == GPIOA) RCC->AHB1ENR |= (1<<0);  // Enable the GPIOA clock
	if (adc.rdy_port == GPIOB) RCC->AHB1ENR |= (1<<1);  // Enable the GPIOB clock
	if (adc.rdy_port == GPIOC) RCC->AHB1ENR |= (1<<2);  // Enable the GPIOC clock
	if (adc.rdy_port == GPIOD) RCC->AHB1ENR |= (1<<3);  // Enable the GPIOD clock
	if (adc.rdy_port == GPIOE) RCC->AHB1ENR |= (1<<4);  // Enable the GPIOE clock
	if (adc.rdy_port == GPIOF) RCC->AHB1ENR |= (1<<5);  // Enable the GPIOF clock
	if (adc.rdy_port == GPIOG) RCC->AHB1ENR |= (1<<6);  // Enable the GPIOG clock
	if (adc.rdy_port == GPIOH) RCC->AHB1ENR |= (1<<7);  // Enable the GPIOH clock

	// ensuring RDY pin is input, with no-pulldown
	adc.rdy_port->MODER   &= ~((1<<(adc.rdy_pin_num << 1)) | (1<<((adc.rdy_pin_num << 1)+1))); 	// Input mode for RDY pin
	adc.rdy_port->PUPDR   &= ~((1<<(adc.rdy_pin_num << 1)) | (1<<((adc.rdy_pin_num << 1)+1)));  // No pullup or pulldown for CS pin
	for (uint8_t i = 0; i < 100; i++) asm("NOP");				// Delay to ensure CS high meets timing requirements


	// check that device responds, ie read STAT register
	uint16_t status = stat_max11270(&adc, verbose);

    setup_max11270(&adc, verbose);
    //calibrate_max11270(adc, verbose);
    if (verbose > 0)
    {
    	if (status == 14480)
    	{
    		print(adc.name);
    		println(" initialised");
    	}
    	else
    	{
    		print(adc.name);
    		println(" failure, no stat reg readback.");
    	}
    }
    cont_conversion(&adc, verbose, adc.speed);
    return adc;
}

//calibrate MAX11270
void calibrate_max11270(struct max11270_device *adc, uint8_t verbose)
{
	if (verbose > 0)
	{
		print("Calibrating ");
		println(adc->name);
	}

	setupSPIMAX11270(adc);

	// Inititate SPI transfer
	adc->cs_port->BSRR |= (1<<(adc->cs_pin_num))<<16; 				// set CS LOW
	for (uint8_t i = 0; i < 10; i++) asm("NOP");					// 60ns ensures timing for CSB Low Setup met
	SPI_Transmit((uint8_t *)&MAX11270_CAL, 1, adc->spi_handle);		// transmit 1 byte to request calibration
	adc->cs_port->BSRR |= (1<<(adc->cs_pin_num)); 					// reset CS HIGH

	returnSPIMAX11270(adc);

	HAL_Delay (200);   /* Insert delay 200 ms */
	if (verbose > 0)
	{
		print(adc->name);
		println(" calibrated");
	}
}

//read from MAX11270
uint32_t read_max11270(struct max11270_device *adc, struct max6225_device *vref, uint8_t speed, uint8_t verbose)
{
	if (verbose > 1)
	{
		print("Reading sample from ");
		println(adc->name);
	}
	uint8_t attempts = 1;
	uint8_t success = 0;
	uint8_t ADC_buffer[3] = {0,0,0};
	setupSPIMAX11270(adc);
	while (success == 0)
	{

		// Inititate SPI transfer
		adc->cs_port->BSRR |= (1<<(adc->cs_pin_num))<<16; 					// set CS LOW
		for (uint8_t i = 0; i < 10; i++) asm("NOP");						// 60ns ensures timing for CSB Low Setup met
		SPI_Transmit((uint8_t *)&MAX11270_READ_DATA, 1, adc->spi_handle);	// transmit 1 byte to request data read
		SPI_Receive (ADC_buffer, 3, adc->spi_handle);						// receive 3 bytes of data
		adc->cs_port->BSRR |= (1<<(adc->cs_pin_num)); 						// reset CS HIGH


		// Reassemble bytes into a 24-bit value, then convert to a voltage reading
		uint32_t sample = ((uint32_t)ADC_buffer[0] << 16) | ((uint32_t)ADC_buffer[1] << 8) | ((uint32_t)ADC_buffer[2]);
		adc->result_bits = sample;
		adc->result_uV = max11270_bits_to_uV(adc->result_bits, *vref);

		if (adc->result_bits == 0) attempts += 1;
		else success = 1;

		if (verbose > 1)
		{
			print("Sample read from ");
			print(adc->name);
			print(" = ");
			printuint32_t(sample, 1);
			print("ATTEMPTS: ");
			printuint8_t(attempts, 1);
		}
	}

	uint16_t stat_reg = stat_max11270(adc, 0);
	if (stat_reg != 12290) cont_conversion(adc, verbose, adc->speed);

	returnSPIMAX11270(adc);
	return adc->result_bits;
}

//setup MAX11270
//update in future for customisable settings?
void setup_max11270(struct max11270_device *adc, uint8_t verbose)
{
	if (verbose > 0)
	{
		print("Setting up ");
		println(adc->name);
	}

	setupSPIMAX11270(adc);

	// Prep write for CTRL1 register: continuous mode, unipolar reference, internal clock
	uint8_t tx_data[2] = {0,0};
	tx_data[0] = MAX11270_REG_CTRL1_WRITE;
	tx_data[1] = 0b00001101;											// desired contents of CTRL1 reg

	// Inititate SPI transfer
	adc->cs_port->BSRR |= (1<<(adc->cs_pin_num))<<16; 					// set CS LOW
	for (uint8_t i = 0; i < 10; i++) asm("NOP");						// 60ns ensures timing for CSB Low Setup met
	SPI_Transmit(tx_data, 2, adc->spi_handle);							// transmit 2 bytes to write CTRL1 reg
	adc->cs_port->BSRR |= (1<<(adc->cs_pin_num)); 						// reset CS HIGH

	// Prep readback from CTRL1 register, to check if correct
	uint8_t CTRL1_reg[1] = {0};
	for(uint8_t i = 0; i < 100; i++) asm("NOP");

	// Inititate SPI transfer
	adc->cs_port->BSRR |= (1<<(adc->cs_pin_num))<<16; 						// set CS LOW
	for (uint8_t i = 0; i < 10; i++) asm("NOP");						// 60ns ensures timing for CSB Low Setup met
	SPI_Transmit((uint8_t *)&MAX11270_REG_CTRL1_READ, 1, adc->spi_handle);	// transmit 1 byte to request ctrl1 read
	SPI_Receive (CTRL1_reg, 1, adc->spi_handle);							// receive 1 byte of data
	adc->cs_port->BSRR |= (1<<(adc->cs_pin_num)); 							// reset CS HIGH

	returnSPIMAX11270(adc);

	// Check if readback matched attempted input
	uint8_t readback = (uint8_t)CTRL1_reg[0];
	if (readback != tx_data[1]) adc->fault = 1;


	if (verbose > 0)
	{
		print("CTRL1: ");
		printuint8_t(readback, 1);
		print(adc->name);
		if (adc->fault == 0)
			println(" is setup");
		else
			println(" did not respond to setup request.");
	}
}

// Converts a 24-bit value into a voltage, manually calibrated
float max11270_bits_to_uV(uint32_t bits, struct max6225_device vref)
{
	float uV = ((float)bits * (vref.output / 15764879.0));
	return uV;
}

// Readback stat register, check for faults
uint16_t stat_max11270(struct max11270_device *adc, uint8_t verbose)
{
	setupSPIMAX11270(adc);

	// Prep readback from STAT register, to check for faults
	uint8_t STAT_reg[2] = {0, 0};

	// Inititate SPI transfer
	adc->cs_port->BSRR |= (1<<(adc->cs_pin_num))<<16; 					// set CS LOW
	for (uint8_t i = 0; i < 10; i++) asm("NOP");						// 60ns ensures timing for CSB Low Setup met
	SPI_Transmit((uint8_t *)&MAX11270_REG_READ, 1, adc->spi_handle);	// transmit 1 byte to request data read
	SPI_Receive (STAT_reg, 2, adc->spi_handle);							// receive 2 bytes of data
	adc->cs_port->BSRR |= (1<<(adc->cs_pin_num)); 						// reset CS HIGH

	returnSPIMAX11270(adc);

	// Reassemble bytes into full 16 bit register value, then check for faults
	uint16_t val = ((uint16_t)STAT_reg[0] << 8) | ((uint16_t)STAT_reg[1]);
	if ((val & 0B1111001100001100) == 0B0011000000000000) adc->fault = 0;
	else adc->fault = 1;

	if (verbose == 1)
	{
		printuint16_t(val, 1);
	}
	if (verbose == 2)
	{
		print(adc->name);
		print(" status value is ");
		printuint16_t(val, 1);
		if (adc->fault == 0) println("No faults detected.");
		else println("Fault detected.");
	}
	return val;
}

uint8_t check_available_max11270(struct max11270_device *adc)
{
	return !((adc->rdy_port->IDR) &(1<<adc->rdy_pin_num));
}


void cont_conversion(struct max11270_device *adc, uint8_t verbose, uint8_t speed)
{
	// valid speeds are 0 to 0xF
	if ((speed >= 0) && (speed <= 0x0F))
	{
		adc->speed = (speed & 0x0F);
	}

	setupSPIMAX11270(adc);

	uint8_t value[1] = {(0B10000000 | (adc->speed & 15))};

	// Inititate SPI transfer
	adc->cs_port->BSRR |= (1<<(adc->cs_pin_num))<<16; 					// set CS LOW
	for (uint8_t i = 0; i < 10; i++) asm("NOP");						// 60ns ensures timing for CSB Low Setup met
	SPI_Transmit(value, 1, adc->spi_handle);							// transmit 1 byte to start conversions
	adc->cs_port->BSRR |= (1<<(adc->cs_pin_num)); 						// reset CS HIGH

	returnSPIMAX11270(adc);

	if (verbose > 0)
	{
		print(adc->name);
		print(" starting conversions at data rate: ");
		printuint8_t(adc->speed, 1);
		println("");
	}
}

void setupSPIMAX11270(struct max11270_device *adc)
{
	// Setup SPI for this device, ie slow and Mode 0
	SPIMode(adc->spi_handle, 0);
	SPISpeed(adc->spi_handle, 1000000); //1MHz
}

void returnSPIMAX11270(struct max11270_device *adc)
{
	// Return SPI to fast and mode 3
	SPIMode(adc->spi_handle, 3);
	SPISpeed(adc->spi_handle, 5000000); //10MHz
}
