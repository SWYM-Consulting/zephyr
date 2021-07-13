/******************************************************************************************
 *
 * @file		photo_resistor.h
 *
 * @brief		Driver for analog photo resistor on SWYM Ambient Board
 *
 * @details		
 *
 * @author(s)	Jordan Hardy
 *
 * @created		7/6/2021   (MM/DD/YYYY)
 * 
 * @copyright		© 2021 SWYM, LLC. All Rights Reserved
 *
 *****************************************************************************************/

#ifndef PHOTO_RESISTOR_H__
#define PHOTO_RESISTOR_H__

#include <devicetree.h>
#include <drivers/adc.h>

#define PR_ADC_NAME DT_LABEL(DT_ALIAS(adcctrl))

#define PR_ADC_RESOLUTION 10
#define PR_ADC_GAIN		  ADC_GAIN_1_6
#define PR_ADC_REFERENCE  ADC_REF_INTERNAL
#define PR_ADC_AQ_TIME	  ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS,40)
#define PR_BUF_SIZE		  6


struct spr_data {
	struct adc_channel_cfg ch_cfg;
	uint16_t raw;
};

struct spr_config{
	const struct device *adc;
	uint8_t adc_channel;
};


int photo_resistor_init(int channel);


#endif
