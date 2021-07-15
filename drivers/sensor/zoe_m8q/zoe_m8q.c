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
 *****************************************************************************/

#include <device.h>

#include <drivers/spi.h>
#include <init.h>
// #include <sys/byteorder.h>
#include <drivers/sensor.h>
#include <logging/log.h>

#include "nmea.h"
#include "zoe_m8q.h"

#include <string.h>


#define SENT_ID_LENGTH 7

#define DT_DRV_COMPAT u_blox_zoe_m8q

LOG_MODULE_REGISTER(ZOE_M8Q, CONFIG_SENSOR_LOG_LEVEL);

static struct zoe_m8q_data zoe_m8q_data;


#define ZOE_M8Q_HAS_CS(inst) DT_INST_SPI_DEV_HAS_CS_GPIOS(inst)

//Required globals to preserved buffer status 
uint8_t rx_buff[BUFF_LEN];
uint32_t buffer_used_it = 0;




static const struct sensor_driver_api zoe_m8q_api_funcs = {
    .attr_set = NULL,    
    .attr_get = NULL,  
    .trigger_set = NULL,
    .sample_fetch = zoe_m8q_sample_fetch,
    .channel_get  = zoe_m8q_channel_get
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
};

DEVICE_DT_INST_DEFINE(0, zoe_m8q_init, device_pm_control_nop,
		    &zoe_m8q_data, &zoe_m8q_config, POST_KERNEL,
		    CONFIG_SENSOR_INIT_PRIORITY, &zoe_m8q_api_funcs);


/**
 * @brief
 * 
 * @return
 * 
 */
int zoe_m8q_init(const struct device *dev)
{
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

	return SUCCESS;
}

static int zoe_m8q_process_nmea_string(uint8_t* buffer) 
{
    // printk("In process_nmea_String():\n[%s]\n\n", buffer);
    char sentence_id_buff[SENT_ID_LENGTH] = {0};

    // printk("process_nmea_string:\n[%s]\n\n", buffer);

    //Confirm checksum
    if (false == nmea_is_valid(buffer)) {
        printk("[ZOE_M8Q] NMEA string failed checksum!\n");
        return FAIL;
    }

    //Get sentence ID
    strncpy(sentence_id_buff, buffer, SENT_ID_LENGTH);
    sentence_id_buff[SENT_ID_LENGTH - 1] = 0;


    //Process NMEA string
    if ((0 == strcmp("$GPGGA", sentence_id_buff)) || 
        (0 == strcmp("$GNGGA", sentence_id_buff))) {
        printk("[ZOE_M8Q] Found GXGGA\n");
        process_gxgga(buffer, &zoe_m8q_data.gxgga);

    } else if ((0 == strcmp("$GPRMC", sentence_id_buff)) || 
               (0 == strcmp("$GNRMC", sentence_id_buff))) {
        printk("[ZOE_M8Q] Found GXRMC\n");
        process_gxrmc(buffer, &zoe_m8q_data.gxrmc);

    } else {
        printk("[ZOE_M8Q] Found NMEA ID that isn't implemented: %s\n\n", sentence_id_buff);
    }
    return SUCCESS;
}

bool zoe_z8q_continue_parsing()
{
    // printk("gxgga.new_data: %d\n", zoe_m8q_data.gxgga.new_data);
    // printk("gxrmc.new_data: %d\n", zoe_m8q_data.gxrmc.new_data);

    if ((true == zoe_m8q_data.gxgga.new_data) &&
        (true == zoe_m8q_data.gxrmc.new_data)) {
        return false;
    } else {
        return true;
    }
}


static int zoe_m8q_process_rx_buffer() 
{

    uint8_t* nmea_begin_ptr = rx_buff;

    while (true) {
        uint8_t temp_buff[BUFF_LEN]; 

        //Find beginning of NMEA string
        nmea_begin_ptr = strchr(nmea_begin_ptr, '$');
        
        if (nmea_begin_ptr == NULL) {
            // printk("[ZOE_M8Q] Buffer does not contain a NMEA string\n\n");
            buffer_used_it = 0;
            return 0;
        }

        //Find the end the next NMEA string
        uint8_t* nmea_end_ptr = strchr(nmea_begin_ptr, '\r');
        uint8_t* nmea_empty_ptr = strchr(nmea_begin_ptr, 0xff);

        // Beginning of NMEA string has been found
        // Check for end of NMEA string
        if (NULL != nmea_end_ptr) {
            uint32_t sentence_length = nmea_end_ptr-nmea_begin_ptr;

            //Copy NMEA string to temp_buffer
            strncpy(temp_buff, nmea_begin_ptr, sentence_length);
            temp_buff[sentence_length] = 0;
            zoe_m8q_process_nmea_string(temp_buff);

            nmea_begin_ptr++;

        // Incomplete NMEA string at the end of buffer
        } else if ((NULL == nmea_end_ptr) && (NULL == nmea_empty_ptr)){  

            uint8_t* nmea_empty_ptr = strchr(nmea_begin_ptr, 0xff);

            // printk("[ZOE_M8Q] Null found\n");
            int partial_str_len = (&(rx_buff[BUFF_LEN]) - nmea_begin_ptr);

            //Copy incomplete NMEA string to beginning of rx_buff
            strncpy(rx_buff, nmea_begin_ptr, partial_str_len);

            //Clear the rest of the buffer
            for (int i=partial_str_len; i<BUFF_LEN; i++) {
                rx_buff[i] = 0;
            }

            buffer_used_it = partial_str_len;
            
            return 0;

        // ERROR empty bytes before NMEA string terminator
        } else {
            printk("[ZOE_M8Q] Empty bytes before NMEA string terminator!\n");
            buffer_used_it = 0;
            return 0;
        }
    }
    return 0;
}

static int zoe_m8q_sample_fetch(const struct device *dev, 
                                enum sensor_channel chan)
{
    struct zoe_m8q_data *data = dev->data;
    int return_value = SUCCESS;

    //Clear data flags
    zoe_m8q_data.gxgga.new_data = false;
    zoe_m8q_data.gxrmc.new_data = false;   
    buffer_used_it = 0; 

    while(zoe_z8q_continue_parsing() && (SUCCESS == return_value)) {

        //Setup buffers
        const struct spi_buf rx_buf_struct = {
            .buf = &rx_buff[buffer_used_it],
            .len = (BUFF_LEN - buffer_used_it)
        };

        const struct spi_buf_set rx = {
            .buffers = &rx_buf_struct,
            .count = 1
        };

        // printk("Before transcieve buffer_used: %d\n", buffer_used_it);
        // printk("Before tranceive rx buff:\n[%s]\n\n", rx_buff);

        //Get data from SPI
        return_value = spi_transceive(data->spi, &data->spi_cfg, NULL, &rx);

        // printk("After tranceive rx buff:\n[%s]\n\n", rx_buff);

        if (return_value != 0) {
            printk("[ZOE_M8Q:ERR] SPI transceive error: 0x%02x\n", return_value);
        } else {
            return_value = zoe_m8q_process_rx_buffer();
        }
    }
    return return_value;
}

static int zoe_m8q_channel_get(const struct device *dev,
                                enum sensor_channel chan,
                                struct sensor_value *val)
{

    struct zoe_m8q_data *data = dev->data;

    switch (chan) {
        case SENSOR_CHAN_GPS_DATA_VALID:
            val->val1 = data->gxrmc.data_valid;
            break;

    	case SENSOR_CHAN_GPS_MODE:
            val->val1 = data->gxrmc.gps_mode;
            break;

        case SENSOR_CHAN_GPS_TIME:
            val->val1 = data->gxrmc.time;
            break;

        case SENSOR_CHAN_GPS_LATITUDE:
            if('N' == data->gxrmc.lat_dir) {
                val->val1 = data->gxrmc.lat_high;
                val->val2 = data->gxrmc.lat_low;
            }
            else {
                val->val1 = -1 * data->gxrmc.lat_high;
                val->val2 = -1 * data->gxrmc.lat_low;
            }
            break;

        case SENSOR_CHAN_GPS_LONGITUDE:
            if('E' == data->gxrmc.lon_dir) {
                val->val1 = data->gxrmc.lon_high;
                val->val2 = data->gxrmc.lon_low;
            }
            else {
                val->val1 = -1 * data->gxrmc.lon_high;
                val->val2 = -1 * data->gxrmc.lon_low;
            }
            break;

        case SENSOR_CHAN_GPS_ALTITUDE:
            val->val1 = data->gxgga.altitude;
            break;

        case SENSOR_CHAN_GPS_GROUNDSPEED:
            val->val1 = data->gxrmc.ground_speed;
            break;

        case SENSOR_CHAN_GPS_TRUE_COURSE:
            val->val1 = data->gxrmc.true_course;
            break;
        default:
            return -ENOTSUP;
    }

	return 0;
}


