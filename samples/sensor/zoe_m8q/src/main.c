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

struct sensor_value data_valid, mode, time, latitude, longitude,
                    altitude, groundspeed, true_course;


void print_gps_data() 
{
    printk("GPS data - Zephyr doesn't support printing floats :/\n");
    printk("Data Valid:\t%c\n", data_valid.val1);
    printk("GPS mode:\t%c\n", mode.val1);
    printk("GPS Time:\t%d\tHH:MM:SS\n", time.val1);
    printk("Latitude:\t%d.%d DD:MM.MMMM\n", latitude.val1, latitude.val2);
    printk("Longitude:\t%d.%d DDD:MM.MMMM\n", longitude.val1, longitude.val2);
    printk("Altitude:\t%d xxxx => xxx.x m\n", altitude.val1);
    printk("Groundspeed:\t%d xxxxxx => xxx.xxx knots\n", groundspeed.val1);
    printk("True Course:\t%d\n", true_course.val1);
    printk("\n");
}

void main(void)
{

    const char *const label = DT_LABEL(DT_INST(0, u_blox_zoe_m8q));
    const struct device *gps = device_get_binding(label);

    if (!gps) {
        printf("Failed to find sensor %s\n", label);
        return;
    }

    while(1) {
        //Poll sensor to get a new set of NMEA strings
        printk("Fetching data from GPS\n");
        sensor_sample_fetch(gps);

        printk("Getting sensor channel\n");
        sensor_channel_get(gps, SENSOR_CHAN_GPS_DATA_VALID, &data_valid);
        sensor_channel_get(gps, SENSOR_CHAN_GPS_MODE, &mode);
        sensor_channel_get(gps, SENSOR_CHAN_GPS_TIME, &time);
        sensor_channel_get(gps, SENSOR_CHAN_GPS_LATITUDE, &latitude);
        sensor_channel_get(gps, SENSOR_CHAN_GPS_LONGITUDE, &longitude);
        sensor_channel_get(gps, SENSOR_CHAN_GPS_ALTITUDE, &altitude);
        sensor_channel_get(gps, SENSOR_CHAN_GPS_GROUNDSPEED, &groundspeed);
        sensor_channel_get(gps, SENSOR_CHAN_GPS_TRUE_COURSE, &true_course);

        print_gps_data();

		k_sleep(K_MSEC(1000));
	}

}
