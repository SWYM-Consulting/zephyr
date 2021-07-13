/******************************************************************************
 *
 * @file            nmea.h
 *
 * @brief           Process NMEA strings and place them in data structure
 *
 * @details         GPS definitons are taken from here: http://aprs.gids.nl/nmea/
 * 
 * @author(s)       Marshall Wingerson
 *
 * @created         06/23/2021   (MM/DD/YYYY)
 *
 *****************************************************************************/

#ifndef NMEA_H_
#define NMEA_H_
    
#include <zephyr/types.h>
#include <stddef.h>

#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <stdio.h>
#include <string.h>

#define SUCCESS 0
#define FAIL -1

#define BUFF_LEN 200

// GPGGA/GNGGA - Global Positioning System Fix Data
struct nmea_gxgga {
    bool new_data;
    
    uint32_t time;      // GMT time
    uint16_t lat_high;  // x in xxxxx.00000
    uint16_t lat_low;   // x in 00000.xxxxx
    char lat_dir;       // N or S
    
    uint16_t lon_high;  // x in xxxxx.00000
    uint16_t lon_low;   // x in 00000.xxxxx
    char lon_dir;       // E or W
    
    uint8_t gps_fix;    // 0 = Invalid
                        // 1 = GPS fix
                        // 2 = DGPS fix
    uint8_t num_of_sats;
    uint16_t hdop;              // 123 = 1.23
    uint16_t altitude;          // 12345 = 123.45m
    int16_t geoid_separation;   //

    uint16_t age_of_diff_data;
    uint16_t diff_ref_station_id;
};

// GPRMC/GNRMC - Recommended minimum specific GPS/Transit data
struct nmea_gxrmc {
    bool new_data;

    uint32_t time;      // GMT time

    char data_valid;    // A-OK
                        // V-Invalid

    uint16_t lat_high;  // x in xxxxx.00000
    uint16_t lat_low;   // x in 00000.xxxxx
    char lat_dir;       // N or S

    uint16_t lon_high;  // x in xxxxx.00000
    uint16_t lon_low;   // x in 00000.xxxxx
    char lon_dir;       // E or W

    uint16_t ground_speed;  // knots 123.456 == 123456
    uint16_t true_course;   // 123.4 == 1234

    uint16_t utc_date;      // UTC date of fix 110206 == 11 Feb 2006

    uint8_t mag_variance;   // degrees 
    char mag_variance_dir;  // E or W

    char gps_mode;          // A-Autonomous
                            // D-Differential
};

// Helper functions
bool nmea_is_valid(uint8_t* buffer);

void process_gxgga(uint8_t* buffer, struct nmea_gxgga *gxgga);
void process_gxrmc(uint8_t* buffer, struct nmea_gxrmc *gxrmc); 

#endif  // NMEA_H_