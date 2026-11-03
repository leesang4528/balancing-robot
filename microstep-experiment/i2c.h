#ifndef I2C_H_
#define I2C_H_

#include <avr/io.h>
#include <stdint.h>

#define I2C_OK   0
#define I2C_ERR  1

void i2c_init(uint32_t scl_hz);
uint8_t i2c_start(uint8_t address);
uint8_t i2c_restart(uint8_t address);
void i2c_stop(void);
uint8_t i2c_write(uint8_t data);
uint8_t i2c_read_ack(void);
uint8_t i2c_read_nack(void);

#endif