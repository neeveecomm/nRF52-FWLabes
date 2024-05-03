#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include "si7006.h"
#include <stdio.h>
#define I2C_NODE DT_NODELABEL(mysensor)

struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE);

/*******************************************************************************
 * @brief Function for write data to from master to slave
 *
 * @param[in] regAddr hold the argument of Register address.
 *
 * @retval    None.
 *******************************************************************************/
void i2c_data_write(uint8_t regAddr)
{
	uint8_t buf[1];

	buf[0] = regAddr;
	i2c_write_dt(&dev_i2c, buf, sizeof(buf));
}

/*******************************************************************************
 * @brief     Function for Read data from slave to master
 *
 * @param[in] None.
 *
 * @retval    function return received data.
 *******************************************************************************/
 uint8_t i2c_data_read()
{
	uint8_t data;
	i2c_read_dt(&dev_i2c, &data, sizeof(data));
	return data;
}

/*******************************************************************************
 * @brief     Function for Read humidity
 *
 * @param[in] None
 *
 * @retval    None.
 *******************************************************************************/
 void MEASURE_RELATIVE_HUMIDITY_NO_HOLD_MASTER_MODE()
{
	uint32_t data1;
	uint32_t data2;
	uint32_t humidity = 0;

	// Send Relative humidity measurement command, NO HOLD MASTER(0xF5)

	i2c_data_write(SI7006_MEASURE_RELATIVE_HUMIDITY_NO_HOLD_MASTER_MODE);

	// for time delay
	k_sleep(K_MSEC(200));

	// Read 2 bytes of humidity data

	// humidity msb, humidity lsb

	data1 = i2c_data_read();
	data2 = i2c_data_read();

	humidity = ((((data1 * 256) + data2) * 125) / 65536) - 6;
	// printk("FOR NO HOLD MASTER MODE\n");
	printk("Relative Humidity : %02d percentage\n", humidity);
}

/*******************************************************************************
 * @brief     Function for Read Temperature
 *
 * @param[in] None
 *
 * @retval    None.
 *******************************************************************************/
void MEASURE_TEMPERATURE_NO_HOLD_MASTER_MODE()
{
	uint32_t data3;
	uint32_t data4;
	uint32_t cTemp = 0;

	// Send temperature measurement command, NO HOLD MASTER(0xF3)

	i2c_data_write(SI7006_MEASURE_TEMPERATURE_NO_HOLD_MASTER_MODE);

	// For time delay
	k_sleep(K_MSEC(200));

	// Read 2 bytes of temperature data

	// humidity msb, humidity lsb

	data3 = i2c_data_read();
	data4 = i2c_data_read();

	cTemp = ((((data3 * 256) + data4) * 175.72) / 65536) - 46.85;
	// int fTemp = (cTemp * 1.8) + 32;

	// Output data to screen
	// printk("FOR NO HOLD MASTER MODE\n");
	printk("Temperature in Celsius : %02d C \n", cTemp);
	// printk("Temperature in Fahrenheit : %02d F \n", fTemp);
}

/*******************************************************************************
 * @brief     Function for Read Temperature and humidity
 *
 * @param[in] None
 *
 * @retval    None.
 *******************************************************************************/
void si7006()
{
	if (!device_is_ready(dev_i2c.bus))
	{
		printk("I2C bus %s is not ready!\n\r", dev_i2c.bus->name);
		return;
	}
	MEASURE_RELATIVE_HUMIDITY_NO_HOLD_MASTER_MODE();

	MEASURE_TEMPERATURE_NO_HOLD_MASTER_MODE();
}
