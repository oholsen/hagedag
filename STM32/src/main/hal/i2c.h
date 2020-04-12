#pragma once

void i2c_start(I2C_TypeDef *i2c, uint32_t timing);
void i2c_stop(I2C_TypeDef *i2c);

uint8_t i2c_read_register(I2C_TypeDef *i2c, uint8_t busid, uint8_t reg);
void i2c_read(I2C_TypeDef *i2c, uint8_t busid, uint8_t reg, uint32_t nbytes, uint8_t *data);
void i2c_write_register(I2C_TypeDef *i2c, uint8_t busid, uint8_t reg, uint8_t value);
