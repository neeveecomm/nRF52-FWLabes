#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include "bh1745.h"

uint8_t red;
uint8_t green;
uint8_t blue;
uint8_t cdata;

#define I2C_NODE DT_NODELABEL(colour_sen)
struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE);

uint8_t cs_i2c_read(uint8_t slaveAddr, uint8_t regAddr)
{
	uint8_t data;
	i2c_write_read_dt(&dev_i2c, &regAddr, sizeof(regAddr), &data, sizeof(data));
	return data;
}

void cs_i2c_write(uint8_t slaveAddr, uint8_t regAddr, uint8_t value1)
{
	uint8_t buf[2];
	buf[0] = regAddr;
	buf[1] = value1;
	i2c_write_dt(&dev_i2c, buf, sizeof(buf));
	return;
}

void intital(void)
{
	cs_i2c_write(BH1745_SA_LOW, 0x41, 0x00); // default time set 160 sec
	cs_i2c_write(BH1745_SA_LOW, MODE_CONTROL2, 0x10); // enable rgbc
	cs_i2c_write(BH1745_SA_LOW, MODE_CONTROL3, 0x02); // default value for mc3
	k_sleep(K_MSEC(500));
}

void RGB_DATA_FOR_READ(void)
{

	int data1 = cs_i2c_read(0x38, 0x50);
	int data2 = cs_i2c_read(0x38, 0x51);
	int data3 = cs_i2c_read(0x38, 0x52);
	int data4 = cs_i2c_read(0x38, 0x53);
	int data5 = cs_i2c_read(0x38, 0x54);
	int data6 = cs_i2c_read(0x38, 0x55);
	int data7 = cs_i2c_read(0x38, 0x56);
	int data8 = cs_i2c_read(0x38, 0x57);

	red = ((data2 * 256) + data1);
	green = ((data4 * 256) + data3);
	blue = ((data6 * 256) + data5);
	cdata = ((data8 * 256) + data7);

	printk("Red color luminance : %d lux \n", red);
	printk("Green color luminance : %d lux \n", green);
	printk("Blue color luminance : %d lux \n", blue);
	printk("Clear Data  Luminance : %d lux \n ", cdata);
}

void colour()
{
	if (!device_is_ready(dev_i2c.bus))
	{
		printk("I2C bus %s is not ready!\n\r", dev_i2c.bus->name);
		return;
	}
	cs_i2c_write(BH1745_SA_LOW, 0x40, 0x4b);
	intital();
	while (1)
	{
		RGB_DATA_FOR_READ();
		printk("\n\nnew data:\n");
		k_msleep(1000);
	}
}
