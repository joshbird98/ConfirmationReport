/*
 * misc_funcs.c
 *
 *  Created on: Feb 10, 2023
 *      Author: Josh
 */

#include "misc_funcs.h"
#include "main.h"
#include "stdint.h"
#include <string.h>
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <errno.h>
#include "board.h"


volatile unsigned int *DWT_CYCCNT   = (volatile unsigned int *)0xE0001004;
volatile unsigned int *DWT_CONTROL  = (volatile unsigned int *)0xE0001000;
volatile unsigned int *DWT_LAR      = (volatile unsigned int *)0xE0001FB0;
volatile unsigned int *SCB_DHCSR    = (volatile unsigned int *)0xE000EDF0;
volatile unsigned int *SCB_DEMCR    = (volatile unsigned int *)0xE000EDFC;
volatile unsigned int *ITM_TER      = (volatile unsigned int *)0xE0000E00;
volatile unsigned int *ITM_TCR      = (volatile unsigned int *)0xE0000E80;


void print(const char _out[])
{
  char buf[300];
  sprintf((char*)buf,_out);
  while(CDC_Transmit_FS((uint8_t*)buf, strlen(buf)) == USBD_BUSY) {}
}

void println(const char _out[])
{
  char buf[300];
  sprintf((char*)buf,_out);
  char ending[2] = {'\r', '\n'};
  strncat((char *)buf, (const char *)ending, 2);
  while(CDC_Transmit_FS((uint8_t*)buf, strlen(buf)) == USBD_BUSY) {}
}

void print_uart(const char _out[], UART_HandleTypeDef huart)
{
	uint8_t buf[300];
	sprintf((char*)buf, _out);
	HAL_UART_Transmit(&huart, buf, sizeof(buf), 100);
}

void println_uart(const char _out[], UART_HandleTypeDef huart)
{
	uint8_t buf[300];
	sprintf((char*)buf, _out);
	char ending[2] = {'\r', '\n'};
	strncat((char *)buf, (const char *)ending, 2);
	HAL_UART_Transmit(&huart, buf, sizeof(buf), 100);
}

void print_buffer_ascii(const char _buffer[], uint8_t header)
{
	if (header == 1)
	{
		print("USB data recieved: ");
	}
	char buf[300];
	sprintf((char*)buf, _buffer);
	while(CDC_Transmit_FS((uint8_t*)buf, strlen(buf)) == USBD_BUSY) {}
}

void printuint32_t(uint32_t value, uint8_t newline)
{
  char str[300];
  sprintf(str, "%lu", value);
  if (newline == 1)
  {
	char ending[2] = {'\r', '\n'};
	strncat((char *)str, (const char *)ending, 2);
  }
  while(CDC_Transmit_FS((uint8_t*)str, strlen(str)) == USBD_BUSY) {}
}

void printint32_t(int32_t value, uint8_t newline)
{
  char str[300];
  sprintf(str, "%ld", value);
  if (newline == 1)
  {
	char ending[2] = {'\r', '\n'};
	strncat((char *)str, (const char *)ending, 2);
  }
  while(CDC_Transmit_FS((uint8_t*)str, strlen(str)) == USBD_BUSY) {};
}


void printint32_t_uart(int32_t value, uint8_t newline, UART_HandleTypeDef huart)
{
  char str[300];
  sprintf(str, "%ld", value);
  if (newline == 1)
  {
	char ending[2] = {'\r', '\n'};
	strncat((char *)str, (const char *)ending, 2);
  }
  HAL_UART_Transmit(&huart, (uint8_t *)str, sizeof(str), 100);
}

void printuint16_t(uint16_t value, uint8_t newline)
{
  char str[300];
  sprintf(str, "%u", value);
  if (newline == 1)
  {
	char ending[2] = {'\r', '\n'};
	strncat((char *)str, (const char *)ending, 2);
  }
  while(CDC_Transmit_FS((uint8_t*)str, strlen(str)) == USBD_BUSY) {};
}

void printint16_t(int16_t value, uint8_t newline)
{
  char str[300];
  sprintf(str, "%d", value);
  if (newline == 1)
  {
	char ending[2] = {'\r', '\n'};
	strncat((char *)str, (const char *)ending, 2);
  }
  while(CDC_Transmit_FS((uint8_t*)str, strlen(str)) == USBD_BUSY) {};
}

void printuint8_t(uint8_t value, uint8_t newline)
{
  char str[300];
  sprintf(str, "%u", value);
  if (newline == 1)
  {
	char ending[2] = {'\r', '\n'};
	strncat((char *)str, (const char *)ending, 2);
  }
  while(CDC_Transmit_FS((uint8_t*)str, strlen(str)) == USBD_BUSY) {};
}

void printint8_t(int8_t value, uint8_t newline)
{
  char str[300];
  sprintf(str, "%d", value);
  if (newline == 1)
  {
	char ending[2] = {'\r', '\n'};
	strncat((char *)str, (const char *)ending, 2);
  }
  while(CDC_Transmit_FS((uint8_t*)str, strlen(str)) == USBD_BUSY) {};
}

void printfloat(float value, uint8_t newline)
{
  char str[300];
  sprintf(str, "%f", value);
  if (newline == 1)
  {
	char ending[2] = {'\r', '\n'};
	strncat((char *)str, (const char *)ending, 2);
  }
  while(CDC_Transmit_FS((uint8_t*)str, strlen(str)) == USBD_BUSY) {};
}

void splash_message(uint8_t verbose)
{
	if (verbose > 0)
	{
		println("~~~ Decel Controller ~~~");
		uint32_t clk_freq_MHz = HAL_RCC_GetHCLKFreq() / 1000000;
		print("Clock Frequency: ");
		printuint32_t(clk_freq_MHz, 0);
		println("MHz");
		print("Verbose level = ");
		printuint8_t(verbose, 1);
		println("");
	}
}

void set_leds(uint8_t led_1_state, uint8_t led_2_state, uint8_t led_3_state)
{
	HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, led_1_state);
	HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, led_2_state);
	HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, led_3_state);
}

void led_twinkle(uint8_t repeats)
{
	for (uint8_t i = 0; i < repeats; i++)
	{
		set_leds(1,0,0);
		HAL_Delay(100);
		set_leds(0,1,0);
		HAL_Delay(100);
		set_leds(0,0,1);
		HAL_Delay(100);
		set_leds(0,0,0);
		HAL_Delay(100);
	}
}

void handle_data(struct command *cmd, char data[], uint8_t verbose)
{
	cmd->valid = 0;
	println("Validating user input");
	//check if data is a command
	if (data[0] == '<')
	{
		char *token;
		char delim[] = ",";
		char end[] = ">";

		token = strtok(data+1, delim);
		strcpy(cmd->type, token);
		if ( token != NULL) token = strtok(NULL, delim);
		strcpy(cmd->object, token);
		if ( token != NULL) token = strtok(NULL, end);

		char *endptr = NULL;
		errno = 0;
		cmd->value = (float)strtol (token, &endptr, 10);
		if ((errno == 0) && (*endptr == '\0'))
		{
			cmd->valid = 1;
		}
	}
	if (verbose > 0)
	{
		if (cmd->valid == 0) println("Unrecognised data input.\n\rNeeds format <cmd,obj,val>");
		else println("Data recognised.");
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	cmd_hdlr.new_uart_data = 1; //needs to be global??? fuck
	HAL_UART_Receive_IT(&huart3, (uint8_t*)&cmd_hdlr.buffer_uart_rx, 64);
}

void print_binary(uint8_t val)
{
	uint8_t i = 128;
	char str[10] = {0};
	uint8_t dig = 0;
	while (i > 0)
	{
		if ((val & i) > 0) str[dig] = '1';
		else str[dig] = '0';
		i = i / 2;
		dig += 1;
	}
	println(str);
}

uint16_t mylog2(uint16_t val)
{
    uint16_t i = 0;
    while (val > 1)
    {
        val = val / 2;
        i += 1;
    }
    return i;
}

void print_reg_values()
{
	println("REG VALUES");
	print("SPI1->CR1: ");
	printuint32_t(SPI1->CR1, 1);
	print("SPI2->CR1: ");
	printuint32_t(SPI2->CR1, 1);
	print("SPI1->CR2: ");
	printuint32_t(SPI1->CR2, 1);
	print("SPI2->CR2: ");
	printuint32_t(SPI2->CR2, 1);
	print("RCC->APB1ENR: ");
	printuint32_t(RCC->APB1ENR, 1);
	print("RCC->APB2ENR: ");
	printuint32_t(RCC->APB2ENR, 1);
	print("RCC->AHB1ENR: ");
	printuint32_t(RCC->AHB1ENR, 1);
	print("GPIOA->MODER: ");
	printuint32_t(GPIOA->MODER, 1);
	print("GPIOB->MODER: ");
	printuint32_t(GPIOB->MODER, 1);
	print("GPIOC->MODER: ");
	printuint32_t(GPIOC->MODER, 1);
	print("GPIOA->OTYPER: ");
	printuint32_t(GPIOA->OTYPER, 1);
	print("GPIOB->OTYPER: ");
	printuint32_t(GPIOB->OTYPER, 1);
	print("GPIOC->OTYPER: ");
	printuint32_t(GPIOC->OTYPER, 1);
	print("GPIOA->OSPEEDR: ");
	printuint32_t(GPIOA->OSPEEDR, 1);
	print("GPIOB->OSPEEDR: ");
	printuint32_t(GPIOB->OSPEEDR, 1);
	print("GPIOC->OSPEEDR: ");
	printuint32_t(GPIOC->OSPEEDR, 1);
	print("GPIOA->PUPDR: ");
	printuint32_t(GPIOA->PUPDR, 1);
	print("GPIOB->PUPDR: ");
	printuint32_t(GPIOB->PUPDR, 1);
	print("GPIOC->PUPDR: ");
	printuint32_t(GPIOC->PUPDR, 1);
	print("GPIOA->AFR[0]: ");
	printuint32_t(GPIOA->AFR[0], 1);
	print("GPIOB->AFR[0]: ");
	printuint32_t(GPIOB->AFR[0], 1);
	print("GPIOC->AFR[0]: ");
	printuint32_t(GPIOC->AFR[0], 1);
	print("GPIOA->AFR[1]: ");
	printuint32_t(GPIOA->AFR[1], 1);
	print("GPIOB->AFR[1]: ");
	printuint32_t(GPIOB->AFR[1], 1);
	print("GPIOC->AFR[1]: ");
	printuint32_t(GPIOC->AFR[1], 1);
}

void EnableTiming(void)
{
  *SCB_DEMCR |= 0x01000000;
  *DWT_LAR = 0xC5ACCE55; // enable access
  *DWT_CYCCNT = 0; // reset the counter
  *DWT_CONTROL |= 1 ; // enable the counter
}

// maybe does not work
uint32_t getTimeCPU(void)
{
	uint32_t clk_freq = HAL_RCC_GetHCLKFreq();
	uint32_t clk_cycles = *DWT_CYCCNT;
	uint32_t time_ns = (uint32_t)(1000000000* (float)clk_cycles / (float)clk_freq);
	return time_ns;
}

// maybe does not work
uint32_t getCPUTick(void)
{
	return *DWT_CYCCNT;
}
