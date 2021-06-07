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

#include <drivers/spi.h>
#include <init.h>
// #include <sys/byteorder.h>
#include <drivers/sensor.h>
#include <logging/log.h>

#define DT_DRV_COMPAT u_blox_zoe_m8q

// LOG_MODULE_REGISTER(ZOE-M8Q, CONFIG_SENSOR_LOG_LEVEL);

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