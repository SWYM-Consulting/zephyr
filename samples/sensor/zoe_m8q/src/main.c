/******************************************************************************
 *
 * @file            main.c
 *
 * @brief           Sample sensor demo for u-blox ZOE-M8Q
 *
 * @details         Using Zephyr icm42605 as a template
 * 
 * @author(s)       Marshall Wingerson
 *
 * @created         05/31/2021   (MM/DD/YYYY)
 *
 * @copyright       © 2020 SWYM, LLC. All Rights Reserved.
 *
 *****************************************************************************/

#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <stdio.h>

void main(void)
{
	// const char *const label = DT_LABEL(DT_INST(0, invensense_icm42605));
	// const struct device *icm42605 = device_get_binding(label);

	// if (!icm42605) {
	// 	printf("Failed to find sensor %s\n", label);
	// 	return;
	// }
	
	const char *const label = DT_LABEL(DT_INST(0, u_blox_zoe_m8q));

	printk("label: %s\n", label);

	const struct device *gps = device_get_binding(label);

	if (!gps) {
		printf("Failed to find sensor %s\n", label);
		return;
	}

	while(1) {

		sensor_sample_fetch(gps);
		// zoe_m8q_do_something();

		k_sleep(K_MSEC(300));
	}

	// tap_trigger = (struct sensor_trigger) {
	// 	.type = SENSOR_TRIG_TAP,
	// 	.chan = SENSOR_CHAN_ALL,
	// };


	// if (sensor_trigger_set(icm42605, &tap_trigger,
	// 		       handle_icm42605_tap) < 0) {
	// 	printf("Cannot configure tap trigger!!!\n");
	// 	return;
	// }

	// double_tap_trigger = (struct sensor_trigger) {
	// 	.type = SENSOR_TRIG_DOUBLE_TAP,
	// 	.chan = SENSOR_CHAN_ALL,
	// };

	// if (sensor_trigger_set(icm42605, &double_tap_trigger,
	// 		       handle_icm42605_double_tap) < 0) {
	// 	printf("Cannot configure double tap trigger!!!\n");
	// 	return;
	// }

	// data_trigger = (struct sensor_trigger) {
	// 	.type = SENSOR_TRIG_DATA_READY,
	// 	.chan = SENSOR_CHAN_ALL,
	// };

	// if (sensor_trigger_set(icm42605, &data_trigger,
	// 		       handle_icm42605_drdy) < 0) {
	// 	printf("Cannot configure data trigger!!!\n");
	// 	return;
	// }

	printf("Configured for triggered sampling.\n");
}
