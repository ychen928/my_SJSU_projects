/*
 * compass.cpp
 *
 *  Created on: 2015/10/15
 *      Author: YuYu
 */

#include "compass.hpp"
#include <stdio.h>
#include <math.h>
#include <i2c2.hpp>
#include <stdbool.h>
#include "eint.h"
#include <string.h>
#include "io.hpp" 

#ifdef ECU_GEOGRAPHICAL

// Writes X,Y,Z values to a file in order to graph its value, for calibration purposes
#define WRITE_TO_FILE       0

#define I2C_ADDR_COMPASS    0x3C
#define ID_REG_A            0x48
#define ID_REG_B            0x34
#define ID_REG_C            0x33

#define SAMPLE_SIZE         200

typedef enum
{
    one_sample      = (0x0 << 5),
    two_samples     = (0x1 << 5),
    four_samples    = (0x2 << 5),
    eight_samples   = (0x3 << 5)
} avg_sample; //configA_reg, samples averaged/ measurement output

typedef enum
{
    _0_75       = (0x0 << 2),
    _1_5        = (0x1 << 2),
    _3          = (0x2 << 2),
    _7_5        = (0x3 << 2),
    _15         = (0x4 << 2),
    _30         = (0x5 << 2),
    _75         = (0x6 << 2)
} dout_rate; // configA_reg, Data output rate bits, rate at which data is written to all three data output register

typedef enum
{
    normal      = (0x0 << 0),
    positive    = (0x1 << 0),
    negative    = (0x2 << 0)
} measurement_bias; // configA_reg, measurement flow of device, incorporate an applied bias into measurement

typedef enum
{
    __88        = (0x0 << 5),
    _1_3        = (0x1 << 5),
    _1_9        = (0x2 << 5),
    _2_5        = (0x3 << 5),
    _4          = (0x4 << 5),
    _4_7        = (0x5 << 5),
    _5_6        = (0x6 << 5),
    _8_1        = (0x7 << 5)
} gain; //configB_reg, configure gain bits, +-Ga, _ means decimal point between number

typedef enum
{
    continuous  = (0x0 << 0),
    single      = (0x1 << 0)
} operation_mode; //mode_reg, mode of operation

typedef enum
{
    config_regA =   0x0,
    config_regB =   0x1,
    mode_reg    =   0x2,
    x_msb       =   0x3,
    x_lsb       =   0x4,
    z_msb       =   0x5,
    z_lsb       =   0x6,
    y_msb       =   0x7,
    y_lsb       =   0x8,
    stat_reg    =   0x9,
    ID_regA     =   0xA,
    ID_regB     =   0xB,
    ID_regC     =   0xC
} register_list;


// magnetic declination for San Jose, California : declination is POSITIVE
#define DECLINATION_DEGREE  13.0
#define DECLINATION_MINUTE  20.0

static I2C2 *s_compass;
static compass_data_ready_cb s_data_ready = 0;
static float digital_res = 0;

static float const cal_matrix[3][3] = 
{
    {1.125, 0.004, 0.015},
    {0.017, 1.111, 0.123},
    {0.016, -0.022, 1.234}
};

static float const cal_bias[3] = 
{
    6.316,
    -150.048,
    209.469   
};

static void set_digital_resolution(gain gain_val);
static float compute_heading(void);
static void transform(float *uncal_val, float *cal_val, int size);

static void compass_rdy_cb(void)
{
    if(s_data_ready)
    {
        s_data_ready();
    }
}

bool compass_init(compass_data_ready_cb cb)
{
    bool success = false;
    uint8_t id_reg[3] = {0};
    uint8_t config_A, config_B, mode;

    if(!cb)
    {
        success = false;
    }
    else
    {
        s_data_ready = cb;
        // data ready when interrupt generated is low, P2.1
        eint3_enable_port2(1 , eint_falling_edge, (void_func_t)compass_rdy_cb);

        s_compass = &(I2C2::getInstance());

        s_compass->readRegisters(I2C_ADDR_COMPASS, ID_regA, &id_reg[0], sizeof(id_reg));

        // whoami registers
        if((id_reg[0] == ID_REG_A) && (id_reg[1] == ID_REG_B) && (id_reg[2] == ID_REG_C))
        {
            success = true;
        }
        else
        {
            printf("id_regA: %x id_regB: %x id_regC: %x\r\n", id_reg[0], id_reg[1], id_reg[2]);
            puts("failed I2C compass init");
            LE.on(4);
        }
        if(success)
        {    
            // Configure configuration register A
            s_compass->writeReg(I2C_ADDR_COMPASS, config_regA, eight_samples | _15 | normal); // 8 samples, 15Hz, no bias
            config_A = s_compass->readReg(I2C_ADDR_COMPASS, config_regA); 
            if(config_A != (eight_samples | _15 | normal))
            {
                success = false;
                printf("INIT failed reading register configuration A: %#2X\n", config_A);
            }

            if(success)
            {
                // Configure configuration register B
                s_compass->writeReg(I2C_ADDR_COMPASS, config_regB, _1_3); // gain: +- 4.7Ga
                set_digital_resolution(_1_3);
                config_B = s_compass->readReg(I2C_ADDR_COMPASS, config_regB);
                if(config_B != _1_3)
                {
                    success = false;
                    printf("INIT failed reading register configuration B: %#2X\n", config_B);
                }
            }

            if(success)
            {
                // Configure mode register
                s_compass->writeReg(I2C_ADDR_COMPASS, mode_reg, continuous); // continuous mode
                mode = s_compass->readReg(I2C_ADDR_COMPASS, mode_reg);
                if(mode != continuous)
                {
                    success = false;
                    printf("INIT failed reading register mode register: %#2X\n", mode);
                }
            }

            if(success)
            {
                uint8_t buffer[6] = { 0 };
                printf("statusB: %#2X\n", s_compass->readReg(I2C_ADDR_COMPASS, stat_reg));
                s_compass->readRegisters(I2C_ADDR_COMPASS, x_msb, &buffer[0], sizeof(buffer)); // flush all 6 data registers to reset stat reg to 0x11
                printf("statusA: %#2X\n", s_compass->readReg(I2C_ADDR_COMPASS, stat_reg));
            }
        }
    }
    return success;
}

static void set_digital_resolution(gain gain_val)
{
    switch(gain_val)
    {
        case __88:
            digital_res = 0.73; 
            break;
        case _1_3:
            digital_res = 0.92;
            break;
        case _1_9:
            digital_res = 1.22;
            break; 
        case _2_5:
            digital_res = 1.52;
            break; 
        case _4:  
            digital_res = 2.27;
            break;   
        case _4_7:
            digital_res = 2.56;
            break;    
        case _5_6:
            digital_res = 3.03;
            break;    
        case _8_1:
            digital_res = 4.35;
            break;
        default:
            digital_res = 0.92;
            break;
    }
}

float get_heading(void)
{
    static float head = 0;
    // uint8_t status =0;
    
    head = compute_heading();

    // status = s_compass->readReg(I2C_ADDR_COMPASS, stat_reg);
    // printf("status: %#2X\n", status);

    return head;
}


float compute_heading(void)
{
#if (WRITE_TO_FILE)
    FILE *dataxyz = fopen("0:data.txt", "a");
#endif

    float uncal[3] = {0};
    float cal[3] = {0};
    int16_t x = s_compass->readReg(I2C_ADDR_COMPASS, x_msb) << 8
            | (s_compass->readReg(I2C_ADDR_COMPASS, x_lsb) & 0xFF);
    x *= digital_res;


    int16_t z = s_compass->readReg(I2C_ADDR_COMPASS, z_msb) << 8
            | (s_compass->readReg(I2C_ADDR_COMPASS, z_lsb) & 0xFF);
    z *= digital_res;


    int16_t y = s_compass->readReg(I2C_ADDR_COMPASS, y_msb) << 8
            | (s_compass->readReg(I2C_ADDR_COMPASS, y_lsb) & 0xFF);
    y *= digital_res;

    uncal[0] = x;
    uncal[1] = y;
    uncal[2] = z;
    transform(uncal, cal, 3);

    float heading = (float) atan2(float(cal[1]), (float(cal[0]))); // in radian

    // http://magnetic-declination.com/
    // formula: (degree + (minute/60.0)) / (180/M_PI)
    float declination_angle = (DECLINATION_DEGREE + (DECLINATION_MINUTE/60.0)) / (180 / M_PI);
    heading += declination_angle;

    // to degrees
    if (heading < 0)
    {
        heading += (2 * M_PI);
        heading *= 180 / M_PI;
    }
    else
        heading *= 180 / M_PI;

    // printf("%d,%d,%d\r\n", x, y, z);
    // printf("%f,%f,%f\r\n", cal[0], cal[1], cal[2]);
#if (WRITE_TO_FILE)
    fprintf(dataxyz,"%d           %d            %d            %f\n",x,y,z, heading);
    fclose(dataxyz);
#endif

    return heading;
}

void transform(float *uncal_val, float *cal_val, int size)
{
    int i = 0;
    int j = 0;
    float result[3] = {0};
    for(i = 0; i < size; i++)
    {
        uncal_val[i] -= cal_bias[i];
    }

    for(i = 0; i < size; i++)
    {
        for(j = 0; j < size; j++)
        {
            result[i] += cal_matrix[i][j] *uncal_val[j];
        }
    }

    for(i = 0; i < size; i++)
    {
        cal_val[i] = result[i];
    }

}

#endif // ECU_GEOGRAPHICAL