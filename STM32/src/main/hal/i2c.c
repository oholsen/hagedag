#include "compiler.h"
#include "board/board.h"
#include "hal/delay.h"
#include "hal/i2c.h"


#define I2C_ADDRESS(bus_id)     ((bus_id) << 1)
#define START_READ(busid)       (I2C_ADDRESS(busid) | I2C_CR2_START | I2C_CR2_RD_WRN)
#define START_WRITE(busid)      (I2C_ADDRESS(busid) | I2C_CR2_START)
#define ONE_BYTE                (((uint32_t) 1) << 16)
#define TWO_BYTES               (((uint32_t) 2) << 16)


/**
* Power up and configure I2C
*/
void i2c_start(I2C_TypeDef *i2c, uint32_t timing)
{
  i2c->CR2 = APB1_CLOCK_MHZ;
  // 50% duty cycle at 400kHz
  i2c->CCR = (APB1_CLOCK_MHZ * 1000000 / 400000 / 2);  
  i2c->CR1 = I2C_CR1_PE;
}


void i2c_stop(I2C_TypeDef *i2c)
{
    // wait for transactions to complete before shutting down
    while ((i2c->SR2 & I2C_SR2_BUSY) != 0);

    // Disable I2C
    i2c->CR1 = 0;
}


uint8_t i2c_read_register(I2C_TypeDef *i2c, uint8_t busid, uint8_t reg)
{
    uint8_t value;
    i2c_read(i2c, busid, reg, 1, &value);
    return value;
}

void i2c_read(I2C_TypeDef *i2c, uint8_t busid, uint8_t reg, uint32_t nbytes, uint8_t *data)
{

    i2c->CR2 = START_WRITE(busid) | ONE_BYTE;

    // write register address
    while ((i2c->SR1 & I2C_SR1_TXE) != 0);
    i2c->TXDR = reg;

    // Wait until TC flag is set
    while ((i2c->ISR & I2C_ISR_TC) == 0);

    // Repeated start condition (no stop above):
    // read nbytes byte values
    i2c->CR2 = START_READ(busid) | I2C_CR2_AUTOEND |  (nbytes << 16);

    while (nbytes-- > 0)
    {
        while ((i2c->SR1 & I2C_SR1_RXNE) == 0);
        *data++ = (uint8_t) (i2c->DR);
        // TODO: figure out how to wait for next byte
    }

    while ((i2c->SR1 & I2C_SR1_STOPF) == 0);
    i2c->ICR = I2C_ICR_STOPCF;
    //i2c->CR2 = 0;
}


void i2c_write_register(I2C_TypeDef *i2c, uint8_t busid, uint8_t reg, uint8_t value)
{
    i2c->CR2 = START_WRITE(busid) | TWO_BYTES | I2C_CR2_AUTOEND;

    // write register address
    while ((i2c->ISR & I2C_ISR_TXIS) == 0);
    i2c->TXDR = reg;

    // write register value
    while ((i2c->ISR & I2C_ISR_TXIS) == 0);
    i2c->TXDR = value;

    while ((i2c->ISR & I2C_ISR_STOPF) == 0);
    i2c->ICR = I2C_ICR_STOPCF;
    //i2c->CR2 = 0;
}
