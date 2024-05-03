#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include "si7006.h"


int main(void)
{

while(1)
{
        si7006();
        k_msleep(1000);
}
        return 0;
}




