/*
 * gps.cpp
 *
 *  Created on: 2015¦~11¤ë19¤é
 *      Author: YuYu
 */

#include "gps.hpp"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "io.hpp"
#include "uart3.hpp"

#ifdef ECU_GEOGRAPHICAL

// Debugs
#define DEBUG       0
#define DEBUG_1     0

// Uart3
#define BAUD_RATE_9600      9600
#define BAUD_RATE_57600     57600
#define RX_SIZE             1000
#define TX_SIZE             1000

// GPS
#define MINRANGE_LONGITUDE  -120    // range of location in bay area, california
#define MAXRANGE_LONGITUDE  -123
#define MINRANGE_LATITUDE   36
#define MAXRANGE_LATITUDE   38
#define MAX_LONGITUDE       180
#define MAX_LATITUDE        90
#define DELIMITER           ","
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
#define PMTK_SET_NMEA_UPDATE_2HZ  "$PMTK220,500*2B"
#define PMTK_SET_BAUD_57600 "$PMTK251,57600*2C"
#define PMTK_SET_NMEA_UPDATE_200_MILLIHERTZ  "$PMTK220,5000*1B"
#define PMTK_API_SET_FIX_CTL_200_MILLIHERTZ  "$PMTK300,5000,0,0,0,0*18"

#define TEST_LAT    37.2958770
#define TEST_LONG   -121.9602980


static Uart3 *s_gps;
float e_lati = TEST_LAT;    // extern variables that will store coordinates from bridge
float e_longi = TEST_LONG;  // extern variables that will store coordinates from bridge

static float s_latitude = 37; // for gps module, default values
static float s_longitude = -121; // for gps module, default values
static float *s_gps_data; // pointer to where data is stored
static float s_results[2] = {0}; // [0] = bearing  ,  [1] = distance

static float compute_bearing(float gps_lat, float gps_long, float dst_lat, float dst_long); // calculate bearing
static float compute_distance(float gps_lat, float gps_long, float dst_lat, float dst_long); // calculate distance
static void tokenize(char *line);  // parse RMC sentence for latitude and longitude
static float convert_nmea_sentence_to_decimal_coord(float coordinates, const char *val); // self-explanatory
static float to_radian(float degree);
static float to_degree(float radian);


bool gps_init(void)
{
    bool success = false;
    
    s_gps = &(Uart3::getInstance());

    success = s_gps->init(BAUD_RATE_9600, RX_SIZE, TX_SIZE); // Baud rate, rx Queue size, tx Queue size

    if(success)
    {
        // update fix rate to 5 seconds
        s_gps->putline(PMTK_SET_NMEA_UPDATE_200_MILLIHERTZ, portMAX_DELAY);
        s_gps->putline(PMTK_API_SET_FIX_CTL_200_MILLIHERTZ, portMAX_DELAY);
        // Output only $GPRMC sentences
        s_gps->putline(PMTK_SET_NMEA_OUTPUT_RMCONLY, portMAX_DELAY);
        // update 10Hz
        s_gps->putline(PMTK_SET_NMEA_UPDATE_10HZ, portMAX_DELAY);
        // set baudrate
        s_gps->putline(PMTK_SET_BAUD_57600, portMAX_DELAY);
        success = s_gps->init(BAUD_RATE_57600, RX_SIZE, TX_SIZE); // Baud rate, rx Queue size, tx Queue size
    }
    else
    {
        puts("UART3 driver init failed!!!!!!!!!!! FIX ME");
    }

    return success;
}

float* get_bearing_and_distance(void)
{
    // recieve buffer, upper half is parsing NMEA RMC for latitude and longitude
    char rx[80] ={'\0'};
    s_gps->gets(rx, 80, 10);
#if DEBUG
   printf("%s\r\n", rx);
#endif
    if (strstr(rx, "$GPRMC")) // look for $GPRMC in string
    {
        tokenize(rx); // split into tokens (parsing)
#if DEBUG_1 || DEBUG
        printf("\nlat: %f      long: %f\r\n", s_latitude, s_longitude);
#endif
    }

    // bottom half is for computing bearing and distance and passing it to geoTask

    float bearing, distance;

    bearing = compute_bearing(s_latitude, s_longitude, e_lati, e_longi);
    distance = compute_distance(s_latitude, s_longitude, e_lati, e_longi);
    s_results[0] = bearing;
    s_results[1] = distance;
#if DEBUG_1
    // printf("bearing: %f         distance: %f\r\n", s_results[0], s_results[1]);
#endif
   s_gps_data = s_results;

    return s_gps_data;
}

float compute_bearing(float gps_lat, float gps_long, float dst_lat, float dst_long)
{
    // bearing = atan2( sin(longDiff_rad)*cos(lat2_rad),
    //cos(lat1_rad)*sin(lat2_rad)-sin(lat1_rad)*cos(lat2_rad)*cos(longDiff_rad))

    float lat1_rad = to_radian(gps_lat);
    float lat2_rad = to_radian(dst_lat);
    float longDiff_rad = to_radian(dst_long - gps_long);

    float y = sin(longDiff_rad) * cos(lat2_rad);
    float x = cos(lat1_rad) * sin(lat2_rad)
            - sin(lat1_rad) * cos(lat2_rad) * cos(longDiff_rad);
    float bearing = fmodf(to_degree(atan2(y, x)) + 360, 360);

    return bearing;
}

float compute_distance(float gps_lat, float gps_long, float dst_lat, float dst_long)
{
    //haversine formula: a = sin^2(latDiff_rad/2) + cos(lat1_rad) * cos(lat2_rad) * sin^2(longDiff_rad/2)
    // formula: c = 2 *atan2(sqrt(a), sqrt(1-a))
    // d = R*c

    float earth_radius = 6371000; //meters
    float lat1_rad = to_radian(gps_lat);
    float lat2_rad = to_radian(dst_lat);
    float latDiff_rad = to_radian(dst_lat - gps_lat);
    float longDiff_rad = to_radian(dst_long - gps_long);

    float a = sin(latDiff_rad / 2) * sin(latDiff_rad / 2)
            + cos(lat1_rad) * cos(lat2_rad) * sin(longDiff_rad / 2)
                    * sin(longDiff_rad / 2);

    float c = 2.0 * atan2(sqrt(a), sqrt(1 - a));
    float d = earth_radius * c;

    return d;
}

void tokenize(char *line)
{
    const char delimiter[2] = DELIMITER;
    char *token;
    int counter = 0;

    // token= strtok(line, delimiter);
    float latTemp = s_latitude;
    float longTemp = s_longitude;
    while((token = strtok_r(line, delimiter, &line)))
    // while(token)
    {
#if DEBUG
        printf("counter : %d   token: %s\r\n", counter, token);
#endif
        if (counter == 3)
        {/*convert the 3trd token latitude*/
            latTemp = atof(token);
            latTemp = convert_nmea_sentence_to_decimal_coord(latTemp, "m"); //"m" for meridian
        }
        /*If 5th token == South multiply by -1*/
        if (counter == 4)
        {
            if (*token == 'S')
            {
                latTemp *= (-1);
            }
        }
        s_latitude = latTemp;
        /*convert the 5th token longitude*/
        if (counter == 5)
        {
            longTemp = atof(token);
            longTemp = convert_nmea_sentence_to_decimal_coord(longTemp, "p"); //"p" for parallel
        }
        /*If 6th token == West multiply by -1*/
        if (counter == 6)
        {
            if (*token == 'W')
            {
                longTemp *= (-1);
            }
        }
        s_longitude = longTemp;
        // token = strtok(NULL, delimiter);
        ++counter;
        // if(counter >=7)
        // {
        //     break;
        // }
    }
}

float convert_nmea_sentence_to_decimal_coord(float coordinates, const char *val)
{
    /* Sample from gps 5153.6605*/
    /* Check limits*/
    if ((*val == 'm') && (coordinates < 0.0 && coordinates > MAX_LATITUDE))
    {
        return 0;
    }
    if (*val == 'p' && (coordinates < 0.0 && coordinates > MAX_LONGITUDE))
    {
        return 0;
    }
    int b; //to store the degrees
    float c; //to store the decimal

    /*Calculate the value in format nn.nnnnnn*/
    /*Explanations at:
     http://www.mapwindow.org/phorum/read.php?3,16271,16310*/

    b = coordinates / 100; // 51 degrees
    c = (coordinates / 100 - b) * 100; //(51.536605 - 51)* 100 = 53.6605
    c /= 60; // 53.6605 / 60 = 0.8943417
    c += b; // 0.8943417 + 51 = 51.8943417
    return c;
}

float to_radian(float degree)
{
    return degree * (M_PI / 180.0);
}

float to_degree(float radian)
{
    return radian * (180.0 / M_PI);
}

#endif // ECU_GEOGRAPHICAL