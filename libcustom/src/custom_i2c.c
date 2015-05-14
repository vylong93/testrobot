/*
 * custom_i2c.c
 *
 *  Created on: Dec 17, 2014
 *      Author: VyLong
 */

#include "libcustom\inc\custom_i2c.h"

void initI2C(void)
{
    //
    // The I2C peripheral must be enabled before use.
    //
    SysCtlPeripheralEnable(I2C_PERIPH_CLOCK);

    //
	// Reset I2C module
    //
	SysCtlPeripheralReset(I2C_PERIPH_CLOCK);

    //
    // GPIO port  needs to be enabled
    // so SCL and SDA pins can be used.
    //
    SysCtlPeripheralEnable(I2C_PORT_CLOCK);

    //
    // Configure the pin muxing for I2C functions on GPIO port.
    //
    GPIOPinConfigure(I2C_SCL);
    GPIOPinConfigure(I2C_SDA);

    //
    // Select the I2C function for these pins.  This function will also
    // configure the GPIO pins pins for I2C operation, setting them to
    // open-drain operation with weak pull-ups.
    //
    GPIOPinTypeI2C(I2C_PORT_BASE, I2C_SDA_PIN);
    GPIOPinTypeI2CSCL(I2C_PORT_BASE, I2C_SCL_PIN);

    //
    // Enable and initialize the I2C master module.  Use the system clock for
    // the I2C module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    //
    I2CMasterInitExpClk(I2C_PERIPH_BASE, SysCtlClockGet(), true);
//    I2CMasterInitExpClk(I2C_PERIPH_BASE, SysCtlClockGet(), false);

    //
    // Enable I2C
    //
    I2CMasterEnable(I2C_PERIPH_BASE);
}

int i2c_write(unsigned char slave_addr,
                     unsigned char reg_addr,
                     unsigned char length,
                     unsigned char const *data)
{
	uint8_t i;

	//
	// Check for invalid data length
	//
    if(0 == length)
    	return -1;

    //
    // Tell the master module what address it will place on the bus when
    // communicating with the slave.  Set the address to SLAVE_ADDRESS
    // (as set in the slave module).  The receive parameter is set to false
    // which indicates the I2C Master is initiating a writes to the slave.
	// If true, It indicates the I2C Master is initiating reads from the slave.
    //
	I2CMasterSlaveAddrSet(I2C_PERIPH_BASE, slave_addr, false);

    //
	// The register address to be write
    //
	I2CMasterDataPut(I2C_PERIPH_BASE, reg_addr);
	delay_us_i2c(1);

	//
	// Send control byte and register address byte to slave device
	//
	I2CMasterControl(I2C_PERIPH_BASE, I2C_MASTER_CMD_BURST_SEND_START);

	//
	// Wait for MCU to finish transaction
	//
	waitForI2CMasterCompletedTransaction();

	for(i = 0; i < length; i++)
    {
	    //
		// Put the data
	    //
		I2CMasterDataPut(I2C_PERIPH_BASE, data[i]);
		delay_us_i2c(1);

		//
		// Send control byte and data byte to slave device
		//
		I2CMasterControl(I2C_PERIPH_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);

		//
		// Wait for MCU to finish transaction
		//
		waitForI2CMasterCompletedTransaction();
    }

	//
	// Notify slave to stop this transmission
	//
	I2CMasterControl(I2C_PERIPH_BASE, I2C_MASTER_CMD_BURST_SEND_STOP);

	//
	// Wait for MCU to finish transaction
	//
	waitForI2CMasterCompletedTransaction();

	return 0;
}

int i2c_read(unsigned char slave_addr,
                    unsigned char reg_addr,
                    unsigned char length,
                    unsigned char *data)
{
	//
	// Check for invalid data length
	//
    if(0 == length)
    	return -1;

    //
    // Tell the master module what address it will place on the bus when
    // communicating with the slave.  Set the address to SLAVE_ADDRESS
    // (as set in the slave module).  The receive parameter is set to false
    // which indicates the I2C Master is initiating a writes to the slave.
    //
	I2CMasterSlaveAddrSet(I2C_PERIPH_BASE, slave_addr, false);

    //
	// The register address to be read
    //
	I2CMasterDataPut(I2C_PERIPH_BASE, reg_addr);

	//
	// Send control byte and register address byte to slave device
	//
	I2CMasterControl(I2C_PERIPH_BASE, I2C_MASTER_CMD_BURST_SEND_START);

	//
	// Wait for MCU to complete send transaction
	//
	waitForI2CMasterCompletedTransaction();

    //
    // Tell the master module what address it will place on the bus when
    // communicating with the slave.  Set the address to SLAVE_ADDRESS
    // (as set in the slave module). The receive parameter is set to true
	// which indicates the I2C Master is initiating reads from the slave.
    //
	I2CMasterSlaveAddrSet(I2C_PERIPH_BASE, slave_addr, true);

	uint8_t i;
	for(i = 0; i < length; i++)
	{
		//
		// Send control byte from the MCU
		//
		if (length == 1)
			I2CMasterControl(I2C_PERIPH_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
		else if (i == 0)
			I2CMasterControl(I2C_PERIPH_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
		else if (i == length - 1)
			I2CMasterControl(I2C_PERIPH_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
		else
			I2CMasterControl(I2C_PERIPH_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);

		//
		// Wait while checking for MCU to complete the transaction
		//
		waitForI2CMasterCompletedTransaction();

		//
		// Check for I2C error
		//
		if(I2CMasterErr(I2C_PERIPH_BASE) != I2C_MASTER_ERR_NONE)
		{
			HWREG(GPIO_PORTF_BASE + (GPIO_O_DATA + ((GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3) << 2))) |= 0xFF;
			return -1;
		}

		//
		// Get the data from the MCU register
		//
		data[i] = I2CMasterDataGet(I2C_PERIPH_BASE);
	}

	return 0;
}

void waitForI2CMasterCompletedTransaction(void)
{
//	unsigned long long counter = 0;
	//
	// Wait while checking for MCU to complete the transaction
	//
	while(I2CMasterBusy(I2C_PERIPH_BASE))
	{
//		counter++;
//		if(counter > 500000)
//		{
//			while(1)
//			{
//				HWREG(GPIO_PORTF_BASE + (GPIO_O_DATA + ((GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3) << 2))) ^= 0xFF;
//				delay_ms_i2c(750);
//			}
//		}
	}
}

void delay_ms_i2c(unsigned int n)
{
	SysCtlDelay(n * SysCtlClockGet()/3000);
}

void delay_us_i2c(unsigned int n)
{
	SysCtlDelay(n * SysCtlClockGet()/3000000);
}

void get_ms(unsigned long *time)
{

}

