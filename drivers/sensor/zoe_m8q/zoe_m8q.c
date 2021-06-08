/******************************************************************************
 *
 * @file            zoe_m8q.c
 *
 * @brief           Driver for u-blox ZOE-M8Q
 *
 * @details         
 * 
 * @author(s)       Marshall Wingerson
 *
 * @created         05/26/2021   (MM/DD/YYYY)
 *
 * @copyright       © 2020 SWYM, LLC. All Rights Reserved.
 *
 *****************************************************************************/

#include <device.h>

#include <drivers/spi.h>
#include <init.h>
// #include <sys/byteorder.h>
#include <drivers/sensor.h>
#include <logging/log.h>

#include "zoe_m8q.h"

#define DT_DRV_COMPAT u_blox_zoe_m8q

// LOG_MODULE_REGISTER(ZOE-M8Q, CONFIG_SENSOR_LOG_LEVEL);

static struct zoe_m8q_data zoe_m8q_data;

int zoe_m8q_init(const struct device *dev)
{
	printk("ZOE_M8Q: init!!!!\n");
	return 0;
}

static int zoe_m8q_chip_init(const struct device *dev)
{
	printk("ZOE_M8Q: chip init!!!!\n");
	return 0;
}




static int zoe_m8q_attr_set(const struct device *dev,
			    enum sensor_channel chan,
			    enum sensor_attribute attr,
			    const struct sensor_value *val)
{
	// switch (attr) {
	// case SENSOR_ATTR_UPPER_THRESH:
	// case SENSOR_ATTR_LOWER_THRESH:
	// 	return adxl362_attr_set_thresh(dev, chan, attr, val);
	// default:
	// 	/* Do nothing */
	// 	break;
	// }

	// switch (chan) {
	// case SENSOR_CHAN_ACCEL_X:
	// case SENSOR_CHAN_ACCEL_Y:
	// case SENSOR_CHAN_ACCEL_Z:
	// case SENSOR_CHAN_ACCEL_XYZ:
	// 	return axl362_acc_config(dev, chan, attr, val);
	// default:
	// 	LOG_DBG("attr_set() not supported on this channel.");
	// 	return -ENOTSUP;
	// }

	return 0;
}


static int zoe_m8q_sample_fetch(const struct device *dev,
				 enum sensor_channel chan)
{

    printk("IN SAMPLE FETCH!!!!\n");
    
	// int result = 0;
	// uint16_t fifo_count = 0;
	// struct icm42605_data *drv_data = dev->data;

	// /* Read INT_STATUS (0x45) and FIFO_COUNTH(0x46), FIFO_COUNTL(0x47) */
	// result = inv_spi_read(REG_INT_STATUS, drv_data->fifo_data, 3);

	// if (drv_data->fifo_data[0] & BIT_INT_STATUS_DRDY) {
	// 	fifo_count = (drv_data->fifo_data[1] << 8)
	// 		+ (drv_data->fifo_data[2]);
	// 	result = inv_spi_read(REG_FIFO_DATA, drv_data->fifo_data,
	// 			      fifo_count);

	// 	/* FIFO Data structure
	// 	 * Packet 1 : FIFO Header(1), AccelX(2), AccelY(2),
	// 	 *            AccelZ(2), Temperature(1)
	// 	 * Packet 2 : FIFO Header(1), GyroX(2), GyroY(2),
	// 	 *            GyroZ(2), Temperature(1)
	// 	 * Packet 3 : FIFO Header(1), AccelX(2), AccelY(2), AccelZ(2),
	// 	 *            GyroX(2), GyroY(2), GyroZ(2), Temperature(1)
	// 	 */
	// 	if (drv_data->fifo_data[0] & BIT_FIFO_HEAD_ACCEL) {
	// 		/* Check empty values */
	// 		if (!(drv_data->fifo_data[1] == FIFO_ACCEL0_RESET_VALUE
	// 		      && drv_data->fifo_data[2] ==
	// 		      FIFO_ACCEL1_RESET_VALUE)) {
	// 			drv_data->accel_x =
	// 				(drv_data->fifo_data[1] << 8)
	// 				+ (drv_data->fifo_data[2]);
	// 			drv_data->accel_y =
	// 				(drv_data->fifo_data[3] << 8)
	// 				+ (drv_data->fifo_data[4]);
	// 			drv_data->accel_z =
	// 				(drv_data->fifo_data[5] << 8)
	// 				+ (drv_data->fifo_data[6]);
	// 		}
	// 		if (!(drv_data->fifo_data[0] & BIT_FIFO_HEAD_GYRO)) {
	// 			drv_data->temp =
	// 				(int16_t)(drv_data->fifo_data[7]);
	// 		} else {
	// 			if (!(drv_data->fifo_data[7] ==
	// 			      FIFO_GYRO0_RESET_VALUE &&
	// 			      drv_data->fifo_data[8] ==
	// 			      FIFO_GYRO1_RESET_VALUE)) {
	// 				drv_data->gyro_x =
	// 					(drv_data->fifo_data[7] << 8)
	// 					+ (drv_data->fifo_data[8]);
	// 				drv_data->gyro_y =
	// 					(drv_data->fifo_data[9] << 8)
	// 					+ (drv_data->fifo_data[10]);
	// 				drv_data->gyro_z =
	// 					(drv_data->fifo_data[11] << 8)
	// 					+ (drv_data->fifo_data[12]);
	// 			}
	// 			drv_data->temp =
	// 				(int16_t)(drv_data->fifo_data[13]);
	// 		}
	// 	} else {
	// 		if (drv_data->fifo_data[0] & BIT_FIFO_HEAD_GYRO) {
	// 			if (!(drv_data->fifo_data[1] ==
	// 			      FIFO_GYRO0_RESET_VALUE &&
	// 			      drv_data->fifo_data[2] ==
	// 			      FIFO_GYRO1_RESET_VALUE)) {
	// 				drv_data->gyro_x =
	// 					(drv_data->fifo_data[1] << 8)
	// 					+ (drv_data->fifo_data[2]);
	// 				drv_data->gyro_y =
	// 					(drv_data->fifo_data[3] << 8)
	// 					+ (drv_data->fifo_data[4]);
	// 				drv_data->gyro_z =
	// 					(drv_data->fifo_data[5] << 8)
	// 					+ (drv_data->fifo_data[6]);
	// 			}
	// 			drv_data->temp =
	// 				(int16_t)(drv_data->fifo_data[7]);
	// 		}
	// 	}
	// }

	return 0;
}



static int zoe_m8q_channel_get(const struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	// struct adxl362_data *data = dev->data;

	// switch (chan) {
	// case SENSOR_CHAN_ACCEL_X: /* Acceleration on the X axis, in m/s^2. */
	// 	adxl362_accel_convert(val, data->acc_x, data->selected_range);
	// 	break;
	// case SENSOR_CHAN_ACCEL_Y: /* Acceleration on the Y axis, in m/s^2. */
	// 	adxl362_accel_convert(val, data->acc_y, data->selected_range);
	// 	break;
	// case SENSOR_CHAN_ACCEL_Z: /* Acceleration on the Z axis, in m/s^2. */
	// 	adxl362_accel_convert(val, data->acc_z,  data->selected_range);
	// 	break;
	// case SENSOR_CHAN_DIE_TEMP: /* Temperature in degrees Celsius. */
	// 	adxl362_temp_convert(val, data->temp);
	// 	break;
	// default:
	// 	return -ENOTSUP;
	// }

	return 0;
}

static const struct sensor_driver_api zoe_m8q_api_funcs = {
	.attr_set     = zoe_m8q_attr_set,
	.sample_fetch = zoe_m8q_sample_fetch,
	.channel_get  = zoe_m8q_channel_get,
// #ifdef CONFIG_ZOE_M8Q_TRIGGER
// 	.trigger_set = zoe_m8q_trigger_set,
// #endif
};

static const struct zoe_m8q_config zoe_m8q_config = {
	.spi_name = DT_INST_BUS_LABEL(0),
	.spi_slave = DT_INST_REG_ADDR(0),
// 	.spi_max_frequency = DT_INST_PROP(0, spi_max_frequency),
// #if DT_INST_SPI_DEV_HAS_CS_GPIOS(0)
// 	.gpio_cs_port = DT_INST_SPI_DEV_CS_GPIOS_LABEL(0),
// 	.cs_gpio = DT_INST_SPI_DEV_CS_GPIOS_PIN(0),
// 	.cs_flags = DT_INST_SPI_DEV_CS_GPIOS_FLAGS(0),
// #endif
// #if defined(CONFIG_ADXL362_TRIGGER)
// 	.gpio_port = DT_INST_GPIO_LABEL(0, int1_gpios),
// 	.int_gpio = DT_INST_GPIO_PIN(0, int1_gpios),
// 	.int_flags = DT_INST_GPIO_FLAGS(0, int1_gpios),
// #endif
};

DEVICE_DT_INST_DEFINE(0, zoe_m8q_init, device_pm_control_nop,
		    &zoe_m8q_data, &zoe_m8q_config, POST_KERNEL,
		    CONFIG_SENSOR_INIT_PRIORITY, &zoe_m8q_api_funcs);
