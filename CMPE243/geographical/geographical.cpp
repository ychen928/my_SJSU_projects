#include "FreeRTOS.h"
#include "semphr.h"
#include "_can_dbc/generated_can.h"
#include "geographical.hpp"
#include "io.hpp"
#include <stdio.h>
#include "can.h"
#include "../Build_Config.hpp"
#include <stdbool.h>
#include "compass.hpp"
#include "gps.hpp"

#ifdef ECU_GEOGRAPHICAL

#define MOCK_GEO            1

// Enable these defines when module is connected to board
#define COMPASS             0
#define GPS                 0

#define CAN_HANDLE      	can1
#define GPS_NOT_FIXED_ERROR 1000000
#define SMA_WINDOW          10

#define DEFAULT_HEADING 	50.000
#define DEFAULT_BEARING 	50.000
#define DEFAULT_DISTANCE 	50.000
#define MOCK_HEADING_0		350.000
#define MOCK_BEARING_0 		120.000
#define MOCK_DISTANCE_0		20.000
#define MOCK_HEADING_1		220.123
#define MOCK_BEARING_1 		90.987
#define MOCK_DISTANCE_1		100.101

#if COMPASS
SemaphoreHandle_t compass_semaphore;
static void compass_data_ready(void)
{
    xSemaphoreGiveFromISR(compass_semaphore, pdFALSE);
}

#endif

// Globals
static can_msg_t can_msg = {0};


void geographical_init(void)
{
    bool success = false;
#if COMPASS
    compass_semaphore = xSemaphoreCreateBinary();
    xSemaphoreTake(compass_semaphore,0);
#endif

    success = CAN_init(CAN_HANDLE, CAN_BAUDRATE, RECEIVER_QUEUE_SIZE, SENDER_QUEUE_SIZE, NULL, NULL);

    if(success)
    {
        // As of now, geographical unit will accept any message, later on filter will be added if needed
        CAN_bypass_filter_accept_all_msgs();

        // Make node (ECU) active on CAN bus by sending reset
        CAN_reset_bus(CAN_HANDLE);
    }
    else
    {
        puts("CAN driver init failed!!!!!!!!!!! FIX ME");
    }
#if GPS
    if(success)
    {
    	success = gps_init();
    }
    else
    {
        LE.on(3);
    }
#endif

#if COMPASS
    if(success)
    {
        success = compass_init(compass_data_ready);
    }
    else
    {
        LE.on(4);
    }	
#endif
    return;
}

void periodic_geographical_1HZ(void)
{
    if (CAN_is_bus_off(CAN_HANDLE)) 
    {
        CAN_reset_bus(CAN_HANDLE);
    }

    LE.toggle(1);
    return;
}

void periodic_geographical_10HZ(void)
{
	static uint32_t count = 0;
    bool gps_data_valid = false;
    bool heading_data_valid = false;
	bool success = false;
    float heading;
	float *gps_data;

#if COMPASS
    if(xSemaphoreTake(compass_semaphore, 0) == pdTRUE)
    {
        heading = get_heading();
        heading_data_valid = true;
        // printf("h: %f\r\n", heading);
    }
    else
    {
        heading_data_valid = false;
    }

#endif // COMPASS

#if GPS
    static float distance_moving_avg[SMA_WINDOW] = {0};
    static float running_count = 0;
    static int sma_count = 0;
    static float distance_sma = 0;


    gps_data = get_bearing_and_distance();
    float bearing = gps_data[0];
    float distance = gps_data[1];

    // Parsing error
    if(distance > GPS_NOT_FIXED_ERROR)
    {
        LE.toggle(2);
        gps_data_valid = false;
    }
    else
    {
        // Simple running average (SMA) of distance values to filter bad parsing
        // Build up initial value
        if(sma_count < SMA_WINDOW)
        {
            distance_moving_avg[sma_count] = distance;
            running_count += distance_moving_avg[sma_count++];
        }
        else
        {
            // printf("sma: %f\r\n", distance_sma);

            running_count += distance;
            running_count -= distance_moving_avg[sma_count % SMA_WINDOW];
            distance_moving_avg[sma_count++ % SMA_WINDOW] = distance;

        }
        // Calculate SMA based on last 10 values
        if(sma_count% SMA_WINDOW == 0)
        {
            distance_sma = running_count/SMA_WINDOW;
        }

        // If current distance is less than or equal to twice the SMA, it is valid
        if(distance <= distance_sma*2)
        {
            gps_data_valid = true;
            LE.off(2);
        }
        else
        {
            gps_data_valid = false;
            LE.on(2);
        }
    }
#endif // GPS

#if GPS && COMPASS
    if(gps_data_valid && heading_data_valid)
    {
        // printf("d: %f  sma: %f\r\n", distance, distance_sma);
        // printf("h: %f b: %f  d: %f\r\n" ,heading, bearing, distance);

        static GEOGRAPHICAL_CMD_t geo_cmd = {.GEOGRAPHICAL_CMD_heading=DEFAULT_HEADING, .GEOGRAPHICAL_CMD_bearing=DEFAULT_BEARING
        , .GEOGRAPHICAL_CMD_distance=DEFAULT_DISTANCE};

        geo_cmd.GEOGRAPHICAL_CMD_heading = heading;
        geo_cmd.GEOGRAPHICAL_CMD_bearing = bearing;
        geo_cmd.GEOGRAPHICAL_CMD_distance = distance;
    }
#endif // GPS && COMPASS

#if MOCK_GEO
	static GEOGRAPHICAL_CMD_t geo_cmd = {.GEOGRAPHICAL_CMD_heading=DEFAULT_HEADING, .GEOGRAPHICAL_CMD_bearing=DEFAULT_BEARING
		, .GEOGRAPHICAL_CMD_distance=DEFAULT_DISTANCE};

	if(SW.getSwitch(1))
	{
		puts("pressed 1");
		geo_cmd.GEOGRAPHICAL_CMD_heading = MOCK_HEADING_0;
		geo_cmd.GEOGRAPHICAL_CMD_bearing = MOCK_BEARING_0;
		geo_cmd.GEOGRAPHICAL_CMD_distance = MOCK_DISTANCE_0;
	}
	else if(SW.getSwitch(2))
	{
		puts("pressed 2");
		geo_cmd.GEOGRAPHICAL_CMD_heading = MOCK_HEADING_1;
		geo_cmd.GEOGRAPHICAL_CMD_bearing = MOCK_BEARING_1;
		geo_cmd.GEOGRAPHICAL_CMD_distance = MOCK_DISTANCE_1;
	}
#endif // MOCK_GEO

	dbc_msg_hdr_t msg_hdr = dbc_encode_GEOGRAPHICAL_CMD(can_msg.data.bytes, &geo_cmd);
	can_msg.msg_id = msg_hdr.mid;
	can_msg.frame_fields.data_len = msg_hdr.dlc;

	success = CAN_tx(CAN_HANDLE, &can_msg,0);
	if(success)
	{
		// puts("sent");
	}
    count++;
	return;
}
void periodic_geographical_100HZ(void)
{

	return;
}

#endif // ECU_GEOGRAPHICAL