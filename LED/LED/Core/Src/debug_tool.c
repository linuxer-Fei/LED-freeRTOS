/*
 * debug_tool.c
 *
 *  Created on: May 22, 2022
 *      Author: SSS
 */
#include "usart.h"
#include "stm32f1xx_hal_uart.h"
#include "debug_tool.h"

static void float2Byte(float *data, unsigned char *buf, unsigned char pos)
{
    unsigned char *ptr;

    if (!data || !buf)
    	return;
    ptr = (unsigned char*)data;

    buf[pos] = ptr[0];
    buf[pos + 1] = ptr[1];
    buf[pos + 2] = ptr[2];
    buf[pos + 3] = ptr[3];
}

static void int2Byte(int *data, unsigned char *buf, unsigned char pos)
{
    unsigned char *ptr;

    if (!data || !buf)
    	return;
    ptr = (unsigned char*)data;

    buf[pos] = ptr[0];
    buf[pos + 1] = ptr[1];
    buf[pos + 2] = ptr[2];
    buf[pos + 3] = ptr[3];
}

#define BUF_LEN 42
#define TIMEOUT 1000
uint8_t debug_buf[BUF_LEN];

void debug_tool_init(void)
{
	debug_buf[0] = '$';//frame head
	debug_buf[41] = 41;
}

static void copy_to_buf(struct db_data * data)
{
	if (!data)
		return;
	debug_tool_init();

	switch(data->chl) {
	case CHANNEL_0:
		if (data->isFloat) {
			float2Byte(&data->fData, debug_buf, 1);
		} else {
			int2Byte(&data->iData, debug_buf, 1);
		}
		break;
	case CHANNEL_1:
		if (data->isFloat) {
			float2Byte(&data->fData, debug_buf, 5);
		} else {
			int2Byte(&data->iData, debug_buf, 5);
		}
		break;
	case CHANNEL_2:
		if (data->isFloat) {
			float2Byte(&data->fData, debug_buf, 9);
		} else {
			int2Byte(&data->iData, debug_buf, 9);
		}
		break;
	case CHANNEL_3:
		if (data->isFloat) {
			float2Byte(&data->fData, debug_buf, 13);
		} else {
			int2Byte(&data->iData, debug_buf, 13);
		}
		break;
	case CHANNEL_4:
		if (data->isFloat) {
			float2Byte(&data->fData, debug_buf, 17);
		} else {
			int2Byte(&data->iData, debug_buf, 17);
		}
		break;
	case CHANNEL_5:
		if (data->isFloat) {
			float2Byte(&data->fData, debug_buf, 21);
		} else {
			int2Byte(&data->iData, debug_buf, 21);
		}
		break;
	case CHANNEL_6:
		if (data->isFloat) {
			float2Byte(&data->fData, debug_buf, 25);
		} else {
			int2Byte(&data->iData, debug_buf, 25);
		}
		break;
	case CHANNEL_7:
		if (data->isFloat) {
			float2Byte(&data->fData, debug_buf, 29);
		} else {
			int2Byte(&data->iData, debug_buf, 29);
		}
		break;
	case CHANNEL_8:
		if (data->isFloat) {
			float2Byte(&data->fData, debug_buf, 33);
		} else {
			int2Byte(&data->iData, debug_buf, 33);
		}
		break;
	case CHANNEL_9:
		if (data->isFloat) {
			float2Byte(&data->fData, debug_buf, 37);
		} else {
			int2Byte(&data->iData, debug_buf, 37);
		}
		break;
	default:
		break;
  }
}

void db_creat(struct db_data * data)
{
	copy_to_buf(data);
}

void db_data_update(struct db_data * data, void* value)
{
	if (!data)
		return;

	if (data->isFloat) {
		data->fData = *((float*)value);
		copy_to_buf(data);
	} else {
		data->iData = *((int*)value);
		copy_to_buf(data);
	}
}

void db_show_data(void)
{
	HAL_UART_Transmit(&huart1, debug_buf, BUF_LEN, TIMEOUT);
}
