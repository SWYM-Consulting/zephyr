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


#include <string.h>

#define DT_DRV_COMPAT u_blox_zoe_m8q

LOG_MODULE_REGISTER(ZOE_M8Q, CONFIG_SENSOR_LOG_LEVEL);

static struct zoe_m8q_data zoe_m8q_data;

#define BUFF_LEN 200

//Required globals to preserved buffer status 
uint8_t rx_buff[BUFF_LEN];
uint32_t buffer_used_it = 0;

/**
 * @brief
 * 
 * @return
 * 
 */
int zoe_m8q_init(const struct device *dev)
{
	printk("ZOE_M8Q: init!!!!\n");

    const struct zoe_m8q_config *config = dev->config;
    struct zoe_m8q_data *data = dev->data;

    data->spi = device_get_binding(config->spi_name);
    if (NULL == data->spi) {
        printk("[zoe-m8q] Error Could not find SPI device\n: %s", config->spi_name);
        LOG_ERR("Error Could not find SPI device: %s", config->spi_name);
        return -EINVAL;
    }
    
    data->spi_cfg.operation = (SPI_OP_MODE_MASTER | SPI_MODE_CPOL |
            SPI_MODE_CPHA | SPI_WORD_SET(8) | SPI_LINES_SINGLE |
            SPI_TRANSFER_MSB);
    data->spi_cfg.frequency = config->spi_max_frequency;
    data->spi_cfg.slave = config->spi_slave;

#if DT_INST_SPI_DEV_HAS_CS_GPIOS(0)
    data->zoe_m8q_cs_ctrl.gpio_dev = device_get_binding(config->gpio_cs_port);
    if(NULL == data->zoe_m8q_cs_ctrl.gpio_dev) {
        printk("[zoe-m8q] Error Could not find CS GPIO: %s\n", config->gpio_cs_port);
        LOG_ERR("Error Could not find CS GPIO: %s", config->gpio_cs_port);
        return -ENODEV;
    }

    data->zoe_m8q_cs_ctrl.gpio_pin = config->cs_gpio;
    data->zoe_m8q_cs_ctrl.gpio_dt_flags = config->cs_flags;
    data->zoe_m8q_cs_ctrl.delay = 0U;

    data->spi_cfg.cs = &data->zoe_m8q_cs_ctrl;
#endif

    //CHIP RESET if needed

    //GET chip PARTID

    //Finish INIT
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


static int zoe_m8q_process_nmea_string(const uint8_t* buffer) {

    printk("Processing string: %s\n", buffer);

    char temp_buffer[BUFF_LEN];
    // const char s[2] = ",";
    char *state;
    strcpy(temp_buffer, buffer);

    char *str_ptr = strtok_r(temp_buffer, ",*", &state);

    while(str_ptr != NULL) {
        printk("str_ptr: %s\n", str_ptr);
        str_ptr = strtok_r((char *)NULL, ",*", &state);
    }

    return 0;
}

static int zoe_m8q_process_rx_buffer() {

    //Find beginning of NMEA string
    uint8_t* nmea_begin_ptr = strchr(rx_buff, '$');
    
    if(nmea_begin_ptr == NULL) {
        printk("ERROR: Buffer does not contain a NMEA string\n");
        return -1;
    }

    while (true) {

        uint8_t temp_buff[BUFF_LEN]; 

        //Find the beginning of the next NMEA string
        uint8_t* nmea_end_ptr = strchr(nmea_begin_ptr+1, '$');

        uint32_t sentence_length = nmea_end_ptr-nmea_begin_ptr;

        if (nmea_end_ptr == NULL) {
        // if (nmea_end_ptr == &(rx_buff[BUFF_LEN])) {
            printk("NULL strchr\n");

            int left_in_buffer = (&(rx_buff[BUFF_LEN]) - nmea_begin_ptr);

            strncpy(rx_buff, nmea_begin_ptr, left_in_buffer);

            //Clean buffer
            for (int i=left_in_buffer; i<BUFF_LEN; i++) {
                rx_buff[i] = 0;
            }

            buffer_used_it = left_in_buffer;
            
            return 0;
        }

        //Copy NMEA string to temp_buffer
        strncpy(temp_buff, nmea_begin_ptr, sentence_length);
        temp_buff[sentence_length] = 0;

        zoe_m8q_process_nmea_string(temp_buff);

        nmea_begin_ptr = nmea_end_ptr;
    }

    return 0;

}

static int zoe_m8q_sample_fetch(const struct device *dev,
                                enum sensor_channel chan)
{
    printk("\nIn FETCH!!!!\n");

    struct zoe_m8q_data *data = dev->data;
    int return_value = 0;

    //Setup buffers
    const struct spi_buf rx_buf_struct = {
        .buf = &rx_buff[buffer_used_it],
        .len = (BUFF_LEN - buffer_used_it)
    };

    const struct spi_buf_set rx = {
        .buffers = &rx_buf_struct,
        .count = 1
    };

    //Read buffer
    return_value = spi_transceive(data->spi, &data->spi_cfg, NULL, &rx);

    if (return_value != 0) {
        printk("SPI FETCH ERROR!!!: 0x%02x\n", return_value);
        return return_value;
    }

    return_value = zoe_m8q_process_rx_buffer();

    return return_value;
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


#define ZOE_M8Q_HAS_CS(inst) DT_INST_SPI_DEV_HAS_CS_GPIOS(inst)


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
	.spi_max_frequency = DT_INST_PROP(0, spi_max_frequency),
#if DT_INST_SPI_DEV_HAS_CS_GPIOS(0)
	.gpio_cs_port = DT_INST_SPI_DEV_CS_GPIOS_LABEL(0),
	.cs_gpio = DT_INST_SPI_DEV_CS_GPIOS_PIN(0),
	.cs_flags = DT_INST_SPI_DEV_CS_GPIOS_FLAGS(0),
#endif
// #if defined(CONFIG_ADXL362_TRIGGER)
// 	.gpio_port = DT_INST_GPIO_LABEL(0, int1_gpios),
// 	.int_gpio = DT_INST_GPIO_PIN(0, int1_gpios),
// 	.int_flags = DT_INST_GPIO_FLAGS(0, int1_gpios),
// #endif
};

DEVICE_DT_INST_DEFINE(0, zoe_m8q_init, device_pm_control_nop,
		    &zoe_m8q_data, &zoe_m8q_config, POST_KERNEL,
		    CONFIG_SENSOR_INIT_PRIORITY, &zoe_m8q_api_funcs);
