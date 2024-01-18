/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
/* STEP 3 - Include the header file of the I2C API */
#include <zephyr/drivers/i2c.h>

/* STEP 4.1 - Include the header file of printk() */
#include <zephyr/sys/printk.h>
#include <math.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   100

/* STEP 8 - Define the I2C slave device address and the addresses of relevant registers */


/* STEP 6 - Get the node identifier of the sensor */
#define I2C0_NODE DT_NODELABEL(accgyro)
#define MPU6050_ACCELZ_HIGH            	0x3F
#define MPU6050_ACCELZ_LOW             	0x40
#define MPU6050_CONFIG_ACCEL           	0x1C
#define MPU6050_PWR_MGT1				0x6B

int main(void)
{

	int ret;

/* STEP 7 - Retrieve the API-specific device structure and make sure that the device is ready to use  */
	static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C0_NODE);
	if (!device_is_ready(dev_i2c.bus)) {
		printk("I2C bus %s is not ready!\n\r",dev_i2c.bus->name);
		return -1;
	}


/* STEP 9 - CONFIG ACCELEROMETER FOR 4G RANGE */
	uint8_t config[2] = {MPU6050_CONFIG_ACCEL,0x08};
	uint8_t pwrmgt[2] = {MPU6050_PWR_MGT1,0x00};
	ret = i2c_write_dt(&dev_i2c, config, sizeof(config));
	if(ret != 0){
		printk("Failed to write to I2C device address %x at Reg. %x \n", dev_i2c.addr,config[0]);
		return -1;
	}
	ret = i2c_write_dt(&dev_i2c, pwrmgt, sizeof(pwrmgt));
	if(ret != 0){
		printk("Failed to write to I2C device address %x at Reg. %x \n", dev_i2c.addr,config[0]);
		return -1;
	}


	while (1) {
/* STEP 10 - Read the temperature from the sensor */
	int8_t accel_reading[2]= {0};
	uint8_t sensor_regs[2] ={MPU6050_ACCELZ_LOW,MPU6050_ACCELZ_HIGH};
	ret = i2c_write_read_dt(&dev_i2c,&sensor_regs[0],1,&accel_reading[0],1);
	if(ret != 0){
		printk("Failed to write/read I2C device address %x at Reg. %x \r\n", dev_i2c.addr,sensor_regs[0]);
	}
	ret = i2c_write_read_dt(&dev_i2c,&sensor_regs[1],1,&accel_reading[1],1);
	if(ret != 0){
		printk("Failed to write/read I2C device address %x at Reg. %x \r\n", dev_i2c.addr,sensor_regs[1]);
	}

/* STEP 11 - Convert the two bytes to a 12-bits */
	double rawAccelZ = (int) accel_reading[1]*256 + (int)accel_reading[0];
	// Convert to engineering units 
	double accelZ = 4*rawAccelZ/(1<<15); //divide signed 16 bit value by 2^16 to get percent of maximum (4gs)
	//Print reading to console  
	printk("Raw Acceleration : %.2f \n", rawAccelZ);
	printk("Acceleration in gs : %.2f \n", accelZ);

		k_msleep(SLEEP_TIME_MS);
	}
}
