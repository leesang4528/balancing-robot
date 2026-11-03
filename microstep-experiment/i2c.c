#define F_CPU 8000000UL

#include "i2c.h"
#include <util/twi.h>   

void i2c_init(uint32_t scl_hz)
{
	TWSR = 0x00;
	uint32_t twbr_val = ((F_CPU / scl_hz) - 16) / 2;
	TWBR = (uint8_t)twbr_val;
}

uint8_t i2c_start(uint8_t address)
{
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	while (!(TWCR & (1<<TWINT)));

	TWDR = address;
	TWCR = (1<<TWINT)|(1<<TWEN);
	while (!(TWCR & (1<<TWINT)));

	uint8_t status = TWSR & 0xF8;
	if (status != TW_MT_SLA_ACK && status != TW_MR_SLA_ACK)
	return I2C_ERR;

	return I2C_OK;
}

uint8_t i2c_restart(uint8_t address)
{
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	while (!(TWCR & (1<<TWINT)));

	TWDR = address;
	TWCR = (1<<TWINT)|(1<<TWEN);
	while (!(TWCR & (1<<TWINT)));

	uint8_t status = TWSR & 0xF8;
	if (status != TW_MT_SLA_ACK && status != TW_MR_SLA_ACK)
	return I2C_ERR;

	return I2C_OK;
}

void i2c_stop(void)
{
	TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
}

uint8_t i2c_write(uint8_t data)
{
	TWDR = data;
	TWCR = (1<<TWINT)|(1<<TWEN);
	while (!(TWCR & (1<<TWINT)));

	uint8_t status = TWSR & 0xF8;
	if (status != TW_MT_DATA_ACK)
	return I2C_ERR;
	return I2C_OK;
}

uint8_t i2c_read_ack(void)
{
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
	while (!(TWCR & (1<<TWINT)));
	return TWDR;
}

uint8_t i2c_read_nack(void)
{
	TWCR = (1<<TWINT)|(1<<TWEN);
	while (!(TWCR & (1<<TWINT)));
	return TWDR;
}