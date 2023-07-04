/*
 * hv_feedback.c
 *
 *  Created on: Feb 23, 2023
 *      Author: Josh
 */

#include "board.h"
#include "main.h"
#include "ad5763.h"

struct board init_board(uint8_t verbose)
{
	struct board decel_board;
	decel_board.error_board = 0;
	decel_board.error_pc = 0;
	decel_board.hv_enable = 0;
	decel_board.disabled_time = 0;
	decel_board.enable_req = 0;
	decel_board.hv_enable_readback = HAL_GPIO_ReadPin(HV_ENABLE_RETURN_GPIO_Port, HV_ENABLE_RETURN_Pin);
	decel_board.divider = init_divider(verbose);
	decel_board.target_decel_voltage = 0;		// requested voltage for the decel voltage (voltage from HP020)
	decel_board.decel_voltage_slow = 0;				// measured voltage of the decel voltage (from the HP020 output)
	decel_board.decel_voltage_fast = 0;				// measured voltage of the decel voltage (from the HP020 output)
	decel_board.target_decel_coarse_setpoint = 0;		// ideal voltage output of DAC HV1 coarse *stage*
	decel_board.target_decel_fine_setpoint = 0;			// ideal voltage output of DAC HV1 fine *stage*
	decel_board.focus_voltage = 0;				// requested voltage for the focus electrode (UV_A output)
	decel_board.injection_voltage = 0;			// requested voltage for the injection electrode (UV_B output
	decel_board.last_slow_update = HAL_GetTick();
	decel_board.e_integral = 0;
	decel_board.time_prev = getTimeCPU();

	return decel_board;
}


struct hv_feedback_divider init_divider(uint8_t verbose)
{
	if (verbose > 0) println("Initialising High-Voltage Feedback Divider");
    struct hv_feedback_divider divider;
    divider.r1_nominal = 375000000.0;
    divider.r2_nominal = 39000.0;
    divider.r1_nominal_temp = 300;
    divider.r2_nominal_temp = 300;
    divider.r1_temp_co = 0;
    divider.r2_temp_co = 0;
    divider.r1 = divider.r1_nominal;
    divider.r2 = divider.r2_nominal;
    divider.ratio = divider.r1 / divider.r2;
    if (verbose > 0) println("High-Voltage Feedback Divider initialised");
    return divider;
}

float temp_comp_divider(struct hv_feedback_divider divider, float temp_K)
{
	divider.r1 = divider.r1_nominal + (divider.r1_temp_co * (temp_K - divider.r1_nominal_temp));
	divider.r2 = divider.r2_nominal + (divider.r2_temp_co * (temp_K - divider.r2_nominal_temp));
	divider.ratio = divider.r1 / divider.r2;
	return divider.ratio;
}


//valid range of -60C to 150C
float getDACTemp(struct mcp33131_device *adc, struct max6225_device *vref, uint8_t verbose)
{
	read_mcp33131(adc, vref, verbose);
	float dac_temp_c = (adc->result_uV - 1365000.0) / 3000;
	if (verbose > 0)
	{
		print("ADC measures ");
		printfloat(adc->result_uV / 1000.0, 0);
		print("mV\n\rRelating to a DAC die temperature of ");
		printfloat(dac_temp_c, 0);
		println("C");
	}
	return dac_temp_c;
}

float getTemp(ADC_HandleTypeDef *adc, uint8_t verbose)
{
	HAL_ADC_Start(adc);
	HAL_ADC_PollForConversion(adc, HAL_MAX_DELAY);
	float temp_c = (((float)HAL_ADC_GetValue(adc) * 3361 / 4096) - 400.0) / 19.5;

	if (verbose > 0)
	{
		print("Measuring ");
		if (adc->Instance == ADC1) print("Vref temperature of ");
		else if (adc->Instance == ADC2) print("board temperature of ");
		else print("? temperature of ");
		printfloat(temp_c, 0);
		println("C");
	}
	return temp_c;
}

void updateTemperatures(struct board *board,
						ADC_HandleTypeDef * adc1,
						ADC_HandleTypeDef * adc2,
						struct mcp33131_device* adc_mcp33131,
						struct max6225_device * vref,
						uint8_t verbose)
{
	board->board_temp = getTemp(adc2, 0);
	board->vref_temp = getTemp(adc1, 0);
	board->dac_temp = getDACTemp(adc_mcp33131, vref, 0);

	if (verbose > 1)
	{
		println("New Temperatures ");
		print("Board : ");
		printfloat(board->board_temp, 0);
		print("C\n\rVoltage Reference: ");
		printfloat(board->vref_temp, 0);
		print("C\n\rDAC Internal: ");
		printfloat(board->dac_temp, 0);
		println("C");
	}
}


uint8_t updateVoltageMeasurements(	struct board *board,
								struct max11270_device *adc_slow_hv1,
								struct mcp33131_device *adc_fast_hv1,
								struct mcp33131_device *adc_fg,
								struct max6225_device * vref,
								uint8_t verbose)
{
	uint8_t newResult = 0;

	if (check_available_max11270(adc_slow_hv1) == 1)
	{
		uint8_t nextSpeed = 7;
		float abs_error = board->target_decel_coarse_setpoint;
		if (abs_error < 0) abs_error *= -1;
		if (abs_error < 10) nextSpeed = 3;
		if (abs_error < 5) nextSpeed = 0;
		read_max11270(adc_slow_hv1, vref, nextSpeed, verbose);
		board->decel_voltage_slow = (adc_slow_hv1->result_uV / 1000000.0) * (board->divider.ratio + 1);
		newResult = 1;
	}

	//Checks for new ADC sample from the high speed HV feedback, updates decel_board struct
	if (check_available_mcp33131(adc_fast_hv1) == 1)
	{
		read_mcp33131(adc_fast_hv1, vref, verbose);
		board->decel_voltage_fast = (adc_fast_hv1->result_uV / 1000000.0) * (board->divider.ratio + 1);
	}

	// Checks for new ADC sample from the FG measurement
	if (check_available_mcp33131(adc_fg) == 1)
	{
		read_mcp33131(adc_fg, vref, verbose);
		board->fg_voltage = adc_fg->result_uV / 1000000.0;
	}

	return newResult;
}


struct command_handler init_command_handler(uint8_t verbose)
{

	struct command cmd;
	cmd.valid = 0;
	memset(cmd.type, 0, sizeof(cmd.type));
	memset(cmd.object, 0, sizeof(cmd.object));
	cmd.value = 0;

	struct command_handler cmd_hdlr;
	cmd_hdlr.new_usb_data = 0;
	memset(cmd_hdlr.buffer_usb_rx, 0, sizeof(cmd_hdlr.buffer_usb_rx));
	cmd_hdlr.new_uart_data = 0;
	memset(cmd_hdlr.buffer_uart_rx, 0, sizeof(cmd_hdlr.buffer_uart_rx));
	cmd_hdlr.cmd = cmd;

	return cmd_hdlr;
}

uint8_t check_cmd(uint8_t verbose)
{
	// checks for newly received USB data, attempts to parse and act on it
	cmd_hdlr.cmd.valid = 0;

	if (cmd_hdlr.new_usb_data != 0)
	{
		println(cmd_hdlr.buffer_usb_rx);
		handle_data(&cmd_hdlr.cmd, cmd_hdlr.buffer_usb_rx, verbose);
		cmd_hdlr.new_usb_data = 0;
	}

	// checks for newly received UART data, attempts to parse and act on it
	if (cmd_hdlr.new_uart_data != 0)
	{
		println(cmd_hdlr.buffer_uart_rx);
		handle_data(&cmd_hdlr.cmd, cmd_hdlr.buffer_uart_rx, verbose);
		cmd_hdlr.new_uart_data = 0;
	}
	return cmd_hdlr.cmd.valid;
}

/* Example commands
<type,object,value>
<setSpeed,Vfb_slow_ADC,0>
<setSpeed,Vfb_slow_ADC,15>
<setVoltage,focusVoltage,>
<setVoltage,injectionVoltage,>
<setVoltage,targetDecelVoltage,>
<requestValue,decelVoltageSlow>
<requestValue,decelVoltageFast>
<requestValue,decelVoltages>
<requestValue,fg_voltage>
<requestValue,HVStatus>
<requestValue,error_board>
<requestValue,board_temp>
<requestValue,res_stack_temp>
<requestValue,vref_temp>
<requestValue,dac_temp>
<requestEnable,>
<requestDisable,>
<errorAlert,>
<errorClear,>
<requestHVStatus,>
*/
void act_cmd(struct command cmd, struct board *board, struct max11270_device *adc, struct ad5763_device *dac, uint8_t verbose)
{
	if (verbose > 1)
	{
		print("Command received...\n\rType: ");
		println(cmd.type);
		print("Object: ");
		println(cmd.object);
		print("Value: ");
		printfloat(cmd.value, 1);
	}

	if (strcmp(cmd.type, "setSpeed") == 0)
	{
		if (strcmp(cmd.object, adc->name) == 0)
		{
			adc->speed = (uint8_t)cmd.value;
			cont_conversion(adc, verbose, adc->speed);
		}
		else {} //no other devices with variable speed
	}
	else if (strcmp(cmd.type, "setVoltage") == 0)
	{
		println(cmd.object);
		if (strcmp(cmd.object, "focusVoltage") == 0)
		{
			board->focus_voltage = cmd.value;
			set_ultravolt_mV(dac, 1, board->focus_voltage, 3);
		}
		else if (strcmp(cmd.object, "injectionVoltage") == 0)
		{
			board->injection_voltage = cmd.value;
			set_ultravolt_mV(dac, 2, board->injection_voltage, 3);
		}
		else if (strcmp(cmd.object, "targetDecelVoltage") == 0)
		{
			board->e_integral = 0;
			board->target_decel_voltage = cmd.value;
		}
	}
	else if (strcmp(cmd.type, "requestValue") == 0)
	{
		print("VALUE:");
		if (strcmp(cmd.object, adc->name) == 0)
			printfloat(adc->result_uV, 1);
		else if (strcmp(cmd.object, "decelVoltageSlow") == 0)
			printfloat(board->decel_voltage_slow, 1);
		else if (strcmp(cmd.object, "decelVoltageFast") == 0)
			printfloat(board->decel_voltage_fast, 1);
		else if (strcmp(cmd.object, "decelVoltages") == 0)
		{
			printfloat(board->decel_voltage_slow, 0);
			print(":");
			printfloat(board->decel_voltage_fast, 1);
		}
		else if (strcmp(cmd.object, "fg_voltage") == 0)
			printfloat(board->fg_voltage, 1);
		else if (strcmp(cmd.object, "HVStatus") == 0)
			printuint8_t(board->hv_enable_readback, 1);
		else if (strcmp(cmd.object, "error_board") == 0)
			printuint8_t(board->error_board, 1);
		else if (strcmp(cmd.object, "board_temp") == 0)
			printfloat(board->board_temp, 1);
		else if (strcmp(cmd.object, "res_stack_temp") == 0)
			printfloat(board->res_stack_temp, 1);
		else if (strcmp(cmd.object, "vref_temp") == 0)
			printfloat(board->vref_temp, 1);
		else if (strcmp(cmd.object, "dac_temp") == 0)
			printfloat(board->dac_temp, 1);
	}
	else if (strcmp(cmd.type, "requestEnable") == 0)
	{
		println("Enable request received");
		board->enable_req = 1;
	}
	else if (strcmp(cmd.type, "requestDisable") == 0)
	{
		HAL_GPIO_WritePin(HV_ENABLE_SOFTWARE_GPIO_Port, HV_ENABLE_SOFTWARE_Pin, (board->hv_enable & !board->error_board & !board->error_pc));
		board->hv_enable = 0;
		board->enable_req = 0;
		board->disabled_time = HAL_GetTick();
		println("Disabled");

	}
	else if (strcmp(cmd.type, "errorAlert") == 0)
	{
		println("Error alert received");
		board->error_pc = 1;
	}
	else if (strcmp(cmd.type, "errorClear") == 0)
	{
		println("Error clear received");
		board->error_pc = 0;
	}
	else if (strcmp(cmd.type, "requestHVStatus") == 0)
	{
		printuint8_t(board->hv_enable_readback, 1);
	}

	cmd_hdlr.cmd.valid = 0; // reset flag
}

uint8_t update_interlock(struct board *board, uint8_t verbose)
{
	board->hv_enable_readback = HAL_GPIO_ReadPin(HV_ENABLE_RETURN_GPIO_Port, HV_ENABLE_RETURN_Pin);
	HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, board->hv_enable_readback);

	if ((board->hv_enable_readback == 0) && (board->hv_enable == 1))
	{
		HAL_GPIO_WritePin(HV_ENABLE_SOFTWARE_GPIO_Port, HV_ENABLE_SOFTWARE_Pin, (board->hv_enable & !board->error_board & !board->error_pc));
		board->hv_enable = 0;
		board->enable_req = 0;
		board->disabled_time = HAL_GetTick();
		if (verbose > 0) println("INTERLOCK: HV was enabled, but readback is low, suggesting open interlock or error.\n\r"
				"Disabling and re-writing hv_enable, enable_req and starting timer.");
	}
	if (board->hv_enable == 0)
	{
		// check to see if it can be re-enabled
		if ((HAL_GetTick() - board->disabled_time > 5000) && (board->enable_req == 1) && (board->error_board == 0) && (board->error_pc == 0))
		{
			board->hv_enable = 1;
			board->enable_req = 0;
			if (verbose > 0) println("INTERLOCK: Enough time has passed, no errors present, and enabled request...\n\rEnabling HV");
		}
		else
		{
			HAL_GPIO_WritePin(HV_ENABLE_SOFTWARE_GPIO_Port, HV_ENABLE_SOFTWARE_Pin, (board->hv_enable & !board->error_board & !board->error_pc));
			board->enable_req = 0;
		}
	}
	if (board->error_pc == 1) board->enable_req = 0;
	if (board->error_board == 1) board->enable_req = 0;
	uint8_t interlock_stat = board->hv_enable & !board->error_pc & !board->error_board;
	HAL_GPIO_WritePin(HV_ENABLE_SOFTWARE_GPIO_Port, HV_ENABLE_SOFTWARE_Pin, interlock_stat);

	return interlock_stat;
}

// Assume all measurements are up to date
void calculate_decel_setpoints(struct board *board, uint8_t verbose)
{

	float error = board->target_decel_voltage + board->target_decel_fine_setpoint - board->decel_voltage_slow;

	if (verbose > 1)
	{
		print("Target Decel voltage: ");
		printfloat(board->target_decel_voltage, 1);
		print("Decel voltage slow: ");
		printfloat(board->decel_voltage_slow, 1);
		print("Fine setpoint (FG): ");
		printfloat(board->target_decel_fine_setpoint, 1);
		print("Error: ");
		printfloat(error, 1);
	}

	if (error > 100.0) error = 100.0;
	if (error < -100.0) error = -100.0;
	uint32_t time_now = getCPUTick();
	board->e_integral += (time_now - board->time_prev) * error / 1000000000.0;
	// Clamp integral component
	if (board->e_integral > 200) board->e_integral = 200;
	if (board->e_integral < -200) board->e_integral = -200;
	board->target_decel_coarse_setpoint = (board->target_decel_voltage) + board->e_integral;


	if (error > 10.0) error = 10.0;
	if (error < -10.0) error = -10.0;
	board->target_decel_fine_setpoint = error;

	if (verbose > 1)
	{
		print("New time: ");
		printfloat(time_now, 1);
		print("Old time: ");
		printfloat(board->time_prev, 1);
		print("Integral value: ");
		printfloat(board->e_integral, 1);
		print("New coarse setpoint : ");
		printfloat(board->target_decel_coarse_setpoint, 1);
		print("New fine setpoint : ");
		printfloat(board->target_decel_fine_setpoint, 1);
		println("");
	}

	board->time_prev = time_now;

}

void update_decel(struct board *board, struct ad5763_device *dac, uint8_t verbose)
{
	if (verbose > 1)
	{
		print("Desired Voltage = ");
		printfloat(board->target_decel_voltage, 1);
		print("Setting coarse voltage: ");
		printfloat(board->target_decel_coarse_setpoint, 1);
		print("Setting fine voltage: ");
		printfloat(board->target_decel_fine_setpoint, 1);
		println("");
	}
	calculate_decel_setpoints(board, 2);
	set_stage_uV(dac, 1, board->target_decel_coarse_setpoint * 1000000.0 / 2000.0, verbose);
	set_stage_uV(dac, 2, board->target_decel_fine_setpoint * 1000000.0, verbose);
}
