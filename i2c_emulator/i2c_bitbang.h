/*
 * i2c_bitbang.h
 *
 *  Created on: Oct 31, 2024
 *      Author: dell
 */

#ifndef I2C_BITBANG_H_
#define I2C_BITBANG_H_

#include "main.h"
#include <stdbool.h>

#include "stdbool.h"
#include "stm32f4xx.h"

/*Define SDA and SCL below after configure GPIO in ioc*/
#define I2C_GPIO_PORT GPIOB
#define I2C_SCL_PIN LL_GPIO_PIN_6
#define I2C_SDA_PIN LL_GPIO_PIN_7

typedef enum
{
    I2C_IDLE,
    I2C_ADDRESS_RECEIVING,
    I2C_SET_SDA_INPUT_ONLY,
    I2C_DATA_RECEIVING,
    I2C_DATA_SENDING,
    I2C_SENDING_ACK,
    I2C_ACK_RECEIVING,
    I2C_CONFIG_SDA_INPUT,
    I2C_STOP,
} I2C_State;

typedef struct
{
	I2C_State state;
	uint8_t count_bit; /*Need reseting to 0 at first*/
	uint8_t slave_address;
	uint8_t slave_rxdata[256];/*Need setting to 0xFF at first*/
	uint8_t slave_txdata[256];/*Need setting to 0xFF at first*/
	uint8_t index_tx;/*Need reseting to 0 at first*/
	uint8_t index_rx;/*Need reseting to 0 at first*/
	uint8_t current_address;
	bool start_condition;/*Need setting to false at first*/
	bool stop_condition;/*Need setting to false at first*/
	bool restart_i2c;/*Need setting to false at first*/
	bool isAddress_correct;/*Need setting to false at first*/
	unsigned char bit;
	unsigned char bit_RW;
	uint8_t list_addr_slave[5];
}i2c_t;



void I2C_Bitbang_Init(void);
void I2C_Bitbang_config(void);
void DWT_Clock_Enable(void);
void I2C_Handle_Event(); // for interrupt
void I2C_Check_Start_Condition();// for interrupt

#endif                 /* I2C_BITBANG_H_ */
