#include <device.h>
#include <drivers/sensor.h>
#include "photo_resistor.h"

#define BAD_ANALOG_READ 6969

#define DT_DRV_COMPAT swym_sprlight
const struct device * pr_dev = NULL;
static struct adc_channel_cfg pr_channel_conf = {
	.gain = PR_ADC_GAIN,
	.reference = PR_ADC_REFERENCE,
	.acquisition_time = PR_ADC_AQ_TIME,
	.channel_id = 0,
	.differential = 0

};

int16_t pr_buffer[PR_BUF_SIZE];

int spr_init(const struct device * dev){

    

    pr_channel_conf.channel_id = 2;

    #ifdef CONFIG_ADC_NRFX_SAADC
		pr_channel_conf.input_positive = SAADC_CH_PSELP_PSELP_AnalogInput0
					     + 2;
    #endif
    struct spr_data *dta = dev->data;
    struct spr_config *cfg = dev->config;
    cfg->adc = device_get_binding(PR_ADC_NAME);
    dta->ch_cfg = pr_channel_conf;

    int err = adc_channel_setup(cfg->adc,&dta->ch_cfg);

    if(0 != err){
        printk("[photores] error: couldn't set up adc channel %d\n",err);
        return err;
    }

    memset(pr_buffer,0,sizeof(pr_buffer));
    printk("[photores] Photoresistor initialized.");
    return 0;
}

int16_t photo_resistor_get_sample(const struct device * dev,uint8_t channel){
    const struct adc_sequence seq = {
        .options = NULL,
        .channels = BIT(channel),
        .buffer = pr_buffer,
        .buffer_size = sizeof(pr_buffer),
        .resolution = PR_ADC_RESOLUTION,
        .oversampling = 0,
        .calibrate = 0
    };
    int err;
    int16_t read_val = BAD_ANALOG_READ;

   

    if(NULL == dev){
        printk("[photores] error: couldn't get dev for sample\n");
        return read_val;
    }

    err = adc_read(dev,&seq);

    if(0 != err){
        printk("[photores] error: couldn't read sample\n");
        return read_val;
    }

    read_val = pr_buffer[0];

    return read_val;
}
int spr_sample_fetch(const struct device *dev,enum sensor_channel chan){
    struct spr_config * cfg = dev->config;
    return (int)photo_resistor_get_sample(cfg->adc,cfg->adc_channel);
}
int spr_channel_get(const struct device *dev,enum sensor_channel chan,struct sensor_value *val){
    //Single Channel sensor, sample_fetch handles getting light sensor values.

    return 0;
}

static const struct sensor_driver_api spr_api = {
    .sample_fetch = &spr_sample_fetch,
    .channel_get = &spr_channel_get
};
static struct spr_data spr_data;
static struct spr_config spr_cfg = {
    .adc = NULL,
    .adc_channel = 2
};
DEVICE_DT_INST_DEFINE(0,&spr_init,NULL,&spr_data,&spr_cfg,POST_KERNEL,CONFIG_SENSOR_INIT_PRIORITY,&spr_api);
