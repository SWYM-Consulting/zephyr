/******************************************************************************
 *
 * @file            zoe_m8q.h
 *
 * @brief           Driver for u-blox ZOE-M8Q
 *
 * @details         Using Zephyr adxl362 as a template
 * 
 *                  GPS definitons are taken from here: http://aprs.gids.nl/nmea/
 * 
 * @author(s)       Marshall Wingerson
 *
 * @created         05/26/2021   (MM/DD/YYYY)
 *
 *****************************************************************************/

#ifndef ZOE_M8Q_H_
#define ZOE_M8Q_H_

#include <drivers/gpio.h>
#include <drivers/spi.h>

#include "nmea.h"


struct zoe_m8q_config {
	char    *spi_name;
	uint32_t spi_max_frequency;
	uint16_t spi_slave;

	const char      *gpio_cs_port;
	gpio_pin_t      cs_gpio;
	gpio_dt_flags_t cs_flags;
};

struct zoe_m8q_data {
    const struct device *spi;
    struct spi_config spi_cfg;
    struct spi_cs_control zoe_m8q_cs_ctrl;

    struct nmea_gxgga gxgga;
    struct nmea_gxrmc gxrmc;
};

int zoe_m8q_init(const struct device *dev);

static int zoe_m8q_channel_get(const struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val);
				   
static int zoe_m8q_sample_fetch(const struct device *dev, 
			enum sensor_channel chan);

#endif  // ZOE_M8Q_H_