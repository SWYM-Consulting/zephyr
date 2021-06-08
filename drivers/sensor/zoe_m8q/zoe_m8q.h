/******************************************************************************
 *
 * @file            zoe_m8q.h
 *
 * @brief           Driver for u-blox ZOE-M8Q
 *
 * @details         Using Zephyr adxl362 as a template
 * 
 * @author(s)       Marshall Wingerson
 *
 * @created         05/26/2021   (MM/DD/YYYY)
 *
 * @copyright       © 2020 SWYM, LLC. All Rights Reserved.
 *
 *****************************************************************************/

#ifndef ZOE_M8Q_h_
#define ZOE_M8Q_h_



struct zoe_m8q_config {
	char *spi_name;
	uint32_t spi_max_frequency;
	uint16_t spi_slave;
// #if DT_INST_SPI_DEV_HAS_CS_GPIOS(0)
// 	const char *gpio_cs_port;
// 	gpio_pin_t cs_gpio;
// 	gpio_dt_flags_t cs_flags;
// #endif
// #if defined(CONFIG_ADXL362_TRIGGER)
// 	const char *gpio_port;
// 	gpio_pin_t int_gpio;
// 	gpio_dt_flags_t int_flags;
// 	uint8_t int1_config;
// 	uint8_t int2_config;
// #endif
};

struct zoe_m8q_data {
	const struct device *spi;
	struct spi_config spi_cfg;
// #if DT_INST_SPI_DEV_HAS_CS_GPIOS(0)
// 	struct spi_cs_control adxl362_cs_ctrl;
// #endif
	// int16_t acc_x;
	// int16_t acc_y;
	// int16_t acc_z;
	// int16_t temp;
	// uint8_t selected_range;

// #if defined(CONFIG_ADXL362_TRIGGER)
// 	const struct device *dev;
// 	const struct device *gpio;
// 	struct gpio_callback gpio_cb;
// 	struct k_mutex trigger_mutex;

// 	sensor_trigger_handler_t th_handler;
// 	struct sensor_trigger th_trigger;
// 	sensor_trigger_handler_t drdy_handler;
// 	struct sensor_trigger drdy_trigger;

// #if defined(CONFIG_ADXL362_TRIGGER_OWN_THREAD)
// 	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_ADXL362_THREAD_STACK_SIZE);
// 	struct k_sem gpio_sem;
// 	struct k_thread thread;
// #elif defined(CONFIG_ADXL362_TRIGGER_GLOBAL_THREAD)
// 	struct k_work work;
// #endif
// #endif /* CONFIG_ADXL362_TRIGGER */
};

// static int zoe_m8q_sample_fetch(const struct device *dev,
// 				 enum sensor_channel chan);


#endif  // ZOE_M8Q_h_