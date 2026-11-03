#define F_CPU 8000000UL
#define BAUD   38400
#define MYUBRR (F_CPU/(16UL*BAUD)-1)

#include <avr/io.h>
#include <util/delay.h>
#include <util/twi.h>
#include <stdio.h>
#include "USART_ch0.h"

#define I2C_OK   0
#define I2C_ERR  1

// ------------------------------------------------------
// I2C
// ------------------------------------------------------
void i2c_init(uint32_t scl_hz)
{
	TWSR = 0x00;
	uint32_t twbr_val = ((F_CPU / scl_hz) - 16) / 2;
	TWBR = (uint8_t)twbr_val;
	TWCR = (1 << TWEN);
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

uint8_t i2c_read_nack(void)
{
	TWCR = (1<<TWINT)|(1<<TWEN);
	while (!(TWCR & (1<<TWINT)));
	return TWDR;
}

// ------------------------------------------------------
// IMU 주소
// ------------------------------------------------------
#define IMU_ADDR        0x68
#define IMU_ADDR_W      (IMU_ADDR << 1)        // 0xD0
#define IMU_ADDR_R      ((IMU_ADDR << 1) | 1)  // 0xD1

#define IMU_RA_WHO_AM_I 0x75

// ------------------------------------------------------
// UART 출력
// ------------------------------------------------------
void print_msg(const char* s)
{
	USART0_Transmit_String((char*)s);
}

void print_hex(const char* title, uint8_t val)
{
	char buf[50];
	sprintf(buf, "%s0x%02X\r\n", title, val);
	USART0_Transmit_String(buf);
}

// ------------------------------------------------------
// WHO_AM_I 읽기
// ------------------------------------------------------
uint8_t imu_read_whoami(void)
{
	uint8_t who = 0;

	if (i2c_start(IMU_ADDR_W) != I2C_OK) return 0xFF;
	if (i2c_write(IMU_RA_WHO_AM_I) != I2C_OK) return 0xFF;
	if (i2c_restart(IMU_ADDR_R) != I2C_OK) return 0xFF;

	who = i2c_read_nack();
	i2c_stop();

	return who;
}

// ------------------------------------------------------
// main
// ------------------------------------------------------
int main(void)
{
	uint8_t who;

	USART0_Init(MYUBRR);
	_delay_ms(100);

	print_msg("=== IMU INFO ===\r\n");

	// 주소 출력
	print_hex("IMU 7bit Addr : ", IMU_ADDR);
	print_hex("IMU Write Addr: ", IMU_ADDR_W);
	print_hex("IMU Read Addr : ", IMU_ADDR_R);

	// I2C 초기화
	i2c_init(100000UL);
	_delay_ms(100);

	// WHO_AM_I 읽기
	who = imu_read_whoami();
	print_hex("WHO_AM_I       : ", who);

	print_msg("================\r\n");

	// 여기서 끝 (반복 안 함)
	while (1);
}