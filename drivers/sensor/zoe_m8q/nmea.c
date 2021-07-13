/******************************************************************************
 *
 * @file            nmea.c
 *
 * @brief           Process NMEA strings and place them in data structure
 *
 * @details         GPS definitons are taken from here: http://aprs.gids.nl/nmea/
 * 
 * @author(s)       Marshall Wingerson
 *
 * @created         06/23/2021   (MM/DD/YYYY)
 *
 * @copyright       © 2020 SWYM, LLC. All Rights Reserved.
 *
 *****************************************************************************/

#include "nmea.h"
#include <stdlib.h>

int add_missing_fields(char *buffer) 
{
    char *buff_ptr = buffer;

    char temp_buffer[BUFF_LEN] = {0};
    char *temp_ptr = temp_buffer;

    // printk("missing fields: %s\n", temp_buffer);

    //string copy and add 0s in empty fields
    while ((0 != *buff_ptr) && (buff_ptr != &buffer[BUFF_LEN])) {     
        *temp_ptr = *buff_ptr;
        
        if ((',' == buff_ptr[0]) && (',' == buff_ptr[1])) {
            temp_ptr++;
            *temp_ptr = '0';
        } 
        
        buff_ptr++;
        temp_ptr++;

        // printk("inside: %s\n", temp_buffer);
    }

    // printk("missing fields after: %s\n", temp_buffer);

    strcpy(buffer, temp_buffer);

    return 0;
}

int compute_checksum(char *nmea_ptr) 
{
    int return_value = 0;

    nmea_ptr++; //Advance ptr after '$'

    while('*' != *nmea_ptr) {
        return_value ^= *nmea_ptr;
        nmea_ptr++;
    }

    return return_value;
}

int8_t ascii_to_hex(char c) 
{
    int8_t return_value;

    if ((c >= '0') && (c <= '9')) {
        return_value = (c - '0');
    } else if ((c >= 'A') && (c <= 'F')) {
        return_value = (c - 'A' + 10);
    } else if ((c >= 'a') && (c <= 'f')) {
        return_value = (c - 'a' + 10);
    } else
        return_value = -1;

    // printk("0c%c == 0x%x\n", c, return_value);

    return return_value;
}

bool nmea_is_valid(uint8_t* buffer) 
{
    bool return_value;
    uint8_t computed_checksum = compute_checksum(buffer);

    uint8_t temp_buff[BUFF_LEN] = {0};
    char *temp_ptr = 0;

    strcpy(temp_buff, buffer);


    char *str_ptr = strtok_r(temp_buff, "*", &temp_ptr);
    str_ptr = strtok_r((char *)NULL, "*", &temp_ptr);
    
    if (NULL == str_ptr) {
        printk("[nmea] Couldn't find '*' in NMEA string\n");
        return false;
    }

    uint8_t nmea_checksum = ascii_to_hex(str_ptr[0]) << 4;
    nmea_checksum |= ascii_to_hex(str_ptr[1]);

    // printk("computed checksum:\t0x%x\n", computed_checksum);
    // printk("nmea_checksum:\t\t0x%x\n", nmea_checksum);

    if (computed_checksum == nmea_checksum)
        return_value = true;
    else 
        return_value = false;
    
    return return_value;
}

void process_gxgga(uint8_t* buffer, struct nmea_gxgga *gxgga) 
{
    // printk("[ZOE_M8Q] Found GXGGA ID!!!\n\n\n");

    add_missing_fields(buffer);

    char *temp_ptr = 0;

    // printk("NMEA STR: %s\n", buffer);

    char *str_ptr = strtok_r(buffer, ",*", &temp_ptr);
    // printk("ID: %s\n", str_ptr);

    //Get time
    str_ptr = strtok_r((char *)NULL, ",*", &temp_ptr);
    gxgga->time = atoi(str_ptr);
    // printk("Time: %d\n", gxgga->time);

    //Get Latitude
    str_ptr = strtok_r((char *)NULL, ".,*", &temp_ptr);
    gxgga->lat_high = atoi(str_ptr);

    str_ptr = strtok_r((char *)NULL, ".,*", &temp_ptr);
    gxgga->lat_low = atoi(str_ptr);

    str_ptr = strtok_r((char *)NULL, ",*", &temp_ptr);
    gxgga->lat_dir = str_ptr[0];

    // printk("lat_high: %d\n", gxgga->lat_high);
    // printk("lat_low: %d\n",  gxgga->lat_low);
    // printk("lat_dir: %c\n",  gxgga->lat_dir);

    //Get Longitude
    str_ptr = strtok_r((char *)NULL, ".,*", &temp_ptr);
    gxgga->lon_high = atoi(str_ptr);

    str_ptr = strtok_r((char *)NULL, ".,*", &temp_ptr);
    gxgga->lon_low = atoi(str_ptr);

    str_ptr = strtok_r((char *)NULL, ",*", &temp_ptr);
    gxgga->lon_dir = str_ptr[0];

    // printk("lon_high: %d\n", gxgga->lon_high);
    // printk("lon_low: %d\n",  gxgga->lon_low);
    // printk("lon_dir: %c\n",  gxgga->lon_dir);

    //Get GPS Fix
    str_ptr = strtok_r((char *)NULL, ",*", &temp_ptr);
    gxgga->gps_fix = atoi(str_ptr);

    // printk("gps_fix: %d\n",  gxgga->gps_fix);

    //Get number of satellites
    str_ptr = strtok_r((char *)NULL, ",*", &temp_ptr);
    gxgga->num_of_sats = atoi(str_ptr);

    // printk("num_of_sats: %d\n",  gxgga->num_of_sats);

    //Get HDOP
    str_ptr = strtok_r((char *)NULL, ".,*", &temp_ptr);
    gxgga->hdop = atoi(str_ptr) * 100;

    str_ptr = strtok_r((char *)NULL, ".,*", &temp_ptr);
    gxgga->hdop += atoi(str_ptr);

    // printk("hdop: %d\n", gxgga->hdop);

    //Get Altitude
    str_ptr = strtok_r((char *)NULL, ".,*", &temp_ptr);
    gxgga->altitude = atoi(str_ptr) * 10;

    str_ptr = strtok_r((char *)NULL, ".,*", &temp_ptr);
    gxgga->altitude += atoi(str_ptr);

    str_ptr = strtok_r((char *)NULL, ".,*", &temp_ptr);

    // printk("altitude: %d\n", gxgga->altitude);

    //Get Height of geoid 
    str_ptr = strtok_r((char *)NULL, ".,*", &temp_ptr);    
    gxgga->geoid_separation = atoi(str_ptr) * 10;

    str_ptr = strtok_r((char *)NULL, ".,*", &temp_ptr);
    if(gxgga->geoid_separation >= 0)
        gxgga->geoid_separation += atoi(str_ptr);
    else 
        gxgga->geoid_separation -= atoi(str_ptr);

    str_ptr = strtok_r((char *)NULL, ".,*", &temp_ptr);

    //Get age of differential data
    str_ptr = strtok_r((char *)NULL, ".,*", &temp_ptr);
    gxgga->age_of_diff_data = atoi(str_ptr);

    // printk("age_of_diff_data: %d\n", gxgga->age_of_diff_data);

    //Get differential station ID
    str_ptr = strtok_r((char *)NULL, ".,*", &temp_ptr);
    gxgga->diff_ref_station_id = atoi(str_ptr);

    // printk("diff_ref_station_id: %d\n", gxgga->diff_ref_station_id);

    //New data has been collected
    gxgga->new_data = true;
}

void process_gxrmc(uint8_t* buffer, struct nmea_gxrmc *gxrmc) 
{
    // printk("[ZOE_M8Q] Found GXRMC ID!!!\n\n\n");

    add_missing_fields(buffer);

    char *temp_ptr = 0;

    // printk("NMEA STR: %s\n", buffer);

    char *str_ptr = strtok_r(buffer, ",*", &temp_ptr);
    // printk("ID: %s\n", str_ptr);

    //Get time
    str_ptr = strtok_r((char *)NULL, ",*", &temp_ptr);
    gxrmc->time = atoi(str_ptr);
    // printk("Time: %d\n", gxrmc->time);

    //Get data valid
    str_ptr = strtok_r((char *)NULL, ",*", &temp_ptr);
    gxrmc->data_valid = str_ptr[0];
    // printk("data_valid: %c\n", gxrmc->data_valid);

    //Get latitude
    str_ptr = strtok_r((char *)NULL, ".,*", &temp_ptr);
    gxrmc->lat_high = atoi(str_ptr);

    str_ptr = strtok_r((char *)NULL, ".,*", &temp_ptr);
    gxrmc->lat_low = atoi(str_ptr);

    str_ptr = strtok_r((char *)NULL, ",*", &temp_ptr);
    gxrmc->lat_dir = str_ptr[0];

    // printk("lat_high: %d\n", gxrmc->lat_high);
    // printk("lat_low: %d\n",  gxrmc->lat_low);
    // printk("lat_dir: %c\n",  gxrmc->lat_dir);

    //Get Longitude
    str_ptr = strtok_r((char *)NULL, ".,*", &temp_ptr);
    gxrmc->lon_high = atoi(str_ptr);

    str_ptr = strtok_r((char *)NULL, ".,*", &temp_ptr);
    gxrmc->lon_low = atoi(str_ptr);

    str_ptr = strtok_r((char *)NULL, ",*", &temp_ptr);
    gxrmc->lon_dir = str_ptr[0];

    // printk("lon_high: %d\n", gxrmc->lon_high);
    // printk("lon_low: %d\n",  gxrmc->lon_low);
    // printk("lon_dir: %c\n",  gxrmc->lon_dir);

    //Get groundspeed
    str_ptr = strtok_r((char *)NULL, ".,*", &temp_ptr);
    gxrmc->ground_speed = atoi(str_ptr) * 1000;

    str_ptr = strtok_r((char *)NULL, ".,*", &temp_ptr);
    gxrmc->ground_speed += atoi(str_ptr);

    // printk("ground_speed: %d\n", gxrmc->ground_speed);

    //Get true_course
    str_ptr = strtok_r((char *)NULL, ".,*", &temp_ptr);
    gxrmc->true_course = atoi(str_ptr);

    // printk("true_course: %d\n", gxrmc->true_course);

    //Get UTC Date
    str_ptr = strtok_r((char *)NULL, ".,*", &temp_ptr);
    gxrmc->utc_date = atoi(str_ptr);

    // printk("utc_date: %d\n", gxrmc->utc_date);

    //Get magnetic variance
    str_ptr = strtok_r((char *)NULL, ".,*", &temp_ptr);
    gxrmc->mag_variance = atoi(str_ptr);

    // printk("mag_variance: %d\n", gxrmc->mag_variance);

    //Get magnetic variance direction
    str_ptr = strtok_r((char *)NULL, ".,*", &temp_ptr);
    gxrmc->mag_variance_dir = str_ptr[0];

    // printk("mag_variance_dir: %c\n", gxrmc->mag_variance_dir);

   //Get GPS mode
    str_ptr = strtok_r((char *)NULL, ".,*", &temp_ptr);
    gxrmc->gps_mode = str_ptr[0];

    // printk("gps_mode: %c\n", gxrmc->gps_mode);

    //New data has been collected
    gxrmc->new_data = true;
}
