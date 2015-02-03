/*
 * custom_i2c.h
 *
 *  Created on: Dec 17, 2014
 *      Author: VyLong
 */

#ifndef CUSTOM_I2C_H_
#define CUSTOM_I2C_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"

#define I2C_PERIPH_CLOCK	SYSCTL_PERIPH_I2C0
#define I2C_PERIPH_BASE		I2C0_BASE
#define I2C_PORT_CLOCK		SYSCTL_PERIPH_GPIOB
#define I2C_PORT_BASE		GPIO_PORTB_BASE
#define I2C_SCL				GPIO_PB2_I2C0SCL
#define I2C_SDA				GPIO_PB3_I2C0SDA
#define I2C_SCL_PIN			GPIO_PIN_2
#define I2C_SDA_PIN			GPIO_PIN_3

void initI2C(void);

int i2c_write(unsigned char slave_addr,
                     unsigned char reg_addr,
                     unsigned char length,
                     unsigned char const *data);
int i2c_read(unsigned char slave_addr,
                    unsigned char reg_addr,
                    unsigned char length,
                    unsigned char *data);

void get_ms(unsigned long *time);

#ifdef __cplusplus
}
#endif

#endif /* CUSTOM_I2C_H_ */
