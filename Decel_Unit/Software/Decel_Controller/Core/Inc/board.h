/*
 * board.h
 *
 *  Created on: Feb 23, 2023
 *      Author: Josh
 */

#ifndef SRC_BOARD_H_
#define SRC_BOARD_H_

#include "stdint.h"
#include "max11270.h"
#include "mcp33131.h"
#include "main.h"
#include "ad5763.h"

#define SLOW_UPDATE_PERIOD 1000

struct hv_feedback_divider {
	float r1_nominal;
	float r2_nominal;
	float r1_nominal_temp;
	float r2_nominal_temp;
	float r1_temp_co;
	float r2_temp_co;
	float r1;
	float r2;
	float ratio;
};

struct board {
	uint8_t hv_enable;					// SAFETY CRITICAL, in series with interlocks, and then HV supplies
	uint8_t hv_enable_readback;			// will read high if (hv_enable == 1) AND (all interlocks connected)
	uint32_t disabled_time;
	uint8_t enable_req;
	uint8_t error_board;						// flag to be set on unexpected board conditions
	uint8_t error_pc;					// flag to be set by USB or UART command

	float target_decel_voltage;		// requested voltage for the decel voltage (voltage from HP020)
	float decel_voltage_slow;				// measured voltage of the decel voltage (from the HP020 output), in mV
	float decel_voltage_fast;				// measured voltage of the decel voltage (from the HP020 output), in mV
	float target_decel_coarse_setpoint;		// ideal voltage output of DAC HV1 coarse *stage*
	float target_decel_fine_setpoint;		// ideal voltage output of DAC HV1 fine *stage*
	float focus_voltage;					// requested voltage for the focus electrode (UV_A output)
	float injection_voltage;				// requested voltage for the injection electrode (UV_B output
	struct hv_feedback_divider divider; 	// complicated structure for handling the HV feedback
	float board_temp;
	float res_stack_temp;
	float vref_temp;
	float dac_temp;
	float fg_voltage;
	float e_integral;
	uint32_t time_prev;
	uint32_t last_slow_update;
};

struct command {
	uint8_t valid;
	char type[100];
	char object[100];
	float value;
};

struct command_handler {
	uint8_t new_usb_data; 			// flag to indicate new usb data has been received
	char buffer_usb_rx[300]; 		// will automatically contain new USB received data
	uint8_t new_uart_data; 			// flag to indicate new uart data has been received
	char buffer_uart_rx[300]; 		// will automatically contain new UART received data
	struct command cmd;				// holds the most recent command received over either USB or UART
};

extern struct command_handler cmd_hdlr;

struct board init_board(uint8_t verbose);

struct hv_feedback_divider init_divider(uint8_t verbose);

float temp_comp_divider(struct hv_feedback_divider, float temp_K);

float getDACTemp(struct mcp33131_device *adc, struct max6225_device *vref, uint8_t verbose);

float getTemp(ADC_HandleTypeDef *adc, uint8_t verbose);

void updateTemperatures(struct board *board,
						ADC_HandleTypeDef * adc1,
						ADC_HandleTypeDef * adc2,
						struct mcp33131_device* adc_mcp33131,
						struct max6225_device * vref,
						uint8_t verbose);


uint8_t updateVoltageMeasurements(	struct board *board,
								struct max11270_device *adc_slow_hv1,
								struct mcp33131_device *adc_fast_hv1,
								struct mcp33131_device *adc_fg,
								struct max6225_device * vref,
								uint8_t verbose);

struct command_handler init_command_handler(uint8_t verbose);

uint8_t check_cmd(uint8_t verbose);
void act_cmd(struct command cmd, struct board *board, struct max11270_device *adc, struct ad5763_device *dac, uint8_t verbose);
uint8_t update_interlock(struct board *board, uint8_t verbose);
void update_decel(struct board *board, struct ad5763_device *dac, uint8_t verbose);


#endif /* SRC_BOARD_H_ */
