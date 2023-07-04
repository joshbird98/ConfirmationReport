/*
 * misc_funcs.h
 *
 *  Created on: Feb 10, 2023
 *      Author: Josh
 */

#ifndef SRC_MISC_FUNCS_H_
#define SRC_MISC_FUNCS_H_

#include "stdint.h"
#include "main.h"
#include "stdint.h"
#include <string.h>
#include <stdio.h>
#include "usbd_cdc_if.h"


// Gain and offsets following the various DAC outputs, values determined by components
#define HV1_COARSE_OFFSET 5.0
#define HV1_COARSE_GAIN 1.220703125 //ie stage is 0-10V output
#define HV1_FINE_OFFSET 0.0
#define HV1_FINE_GAIN -2.0	//ie stage is -8 to +8V output
#define UVA_OFFSET 2.5
#define UVA_GAIN 0.6103515625 // ie stage is 0-5V output
#define UVB_OFFSET 2.5
#define UVB_GAIN 0.6103515625 // ie stage is 0-5V output

extern UART_HandleTypeDef huart3;


void print(const char _out[]);
void println(const char _out[]);
void print_uart(const char _out[], UART_HandleTypeDef huart);
void println_uart(const char _out[], UART_HandleTypeDef huart);
void print_buffer_ascii(const char _buffer[], uint8_t header);
void printuint32_t(uint32_t value, uint8_t newline);
void printint32_t(int32_t value, uint8_t newline);
void printint32_t_uart(int32_t value, uint8_t newline, UART_HandleTypeDef huart);
void printuint16_t(uint16_t value, uint8_t newline);
void printint16_t(int16_t value, uint8_t newline);
void printuint8_t(uint8_t value, uint8_t newline);
void printint8_t(int8_t value, uint8_t newline);
void printfloat(float value, uint8_t newline);
void splash_message(uint8_t verbose);
void set_leds(uint8_t led_1_state, uint8_t led_2_state, uint8_t led_3_state);
void led_twinkle(uint8_t repeats);
void print_binary(uint8_t val);
uint16_t mylog2(uint16_t val);
void print_reg_values();

void handle_data(struct command *cmd, char data[], uint8_t verbose);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

void EnableTiming(void);
uint32_t getTimeCPU(void);
uint32_t getCPUTick(void);

#endif /* SRC_MISC_FUNCS_H_ */
