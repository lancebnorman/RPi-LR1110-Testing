/*

    Interface for communicating between Raspberry Pi 4 Model B and other sensors over I2C
    Currently supporting:
        - SHT31 Temp sensor
        - LIS3DH Accelerometer

 */

#include <stdio.h>
#include "bcm2835/bcm2835.h"
#include "lis3dh/lis3dh.h"
#include "lis3dh/lis3dh_types.h"

#define LIS3DH_I2C_ADDRESS_1 0x18

// Initialize IO library
int init_io()
{
    if (!bcm2835_init())
    {
        printf("bcm2835 I/O init failed\n");
        return 1;
    }
    return 0;
}

// Close IO library
void close_io()
{
    bcm2835_close();
}

// Initialize I2C
int init_i2c()
{
    if (!bcm2835_i2c_begin())
    {
        printf("i2c init failed\n");
        return 1;
    }
    bcm2835_i2c_setClockDivider(BCM2835_I2C_CLOCK_DIVIDER_626);
    bcm2835_i2c_set_baudrate(10);	

    return 0;
}

void close_i2c()
{
    bcm2835_i2c_end();
}

// Read temp/humidity from SHT31 Temp Sensor 
int read_temp()
{
    // Set slave address to 0x24
    bcm2835_i2c_setSlaveAddress(0x44);

    // send command to read temp and relative humidity
    char read_cmd[2] = {0};
    read_cmd[0] = 0x2C;
    read_cmd[1] = 0x06;
    bcm2835_i2c_write(read_cmd, 2);

    delay(10);

    // read 6 bytes into data
    char data[6] = {0};
    bcm2835_i2c_read(data, 6);

    // convert raw data and print
    double cTemp = (((data[0] * 256) + data[1]) * 175.0) / 65535.0 - 45.0;
    double fTemp = (((data[0] * 256) + data[1]) * 315.0) / 65535.0 - 49.0;
    double humidity = (((data[3] * 256) + data[4])) * 100.0 / 65535.0;
    printf("Temp (C) : %f\n", cTemp);
    printf("Temp (F) : %.2f F \n", fTemp);
    printf("Relative Humidity : %.2f RH \n\n", humidity);

    return 1;
}

static lis3dh_sensor_t* accelerometer;

// Initialize and configure lis3dh accelerometer 
void init_accel()
{
    // Initialize lis3dh accelerometer  
    accelerometer = lis3dh_init_sensor (0, LIS3DH_I2C_ADDRESS_1, 0);

    // configure HPF and reset the reference by dummy read
    lis3dh_config_hpf (accelerometer, lis3dh_hpf_normal, 0, true, true, true, true);
    lis3dh_get_hpf_ref (accelerometer);
    
    // enable ADC inputs and temperature sensor for ADC input 3
    lis3dh_enable_adc (accelerometer, true, true);
    
    // LAST STEP: Finally set scale and mode to start measurements
    lis3dh_set_scale(accelerometer, lis3dh_scale_2_g);
    lis3dh_set_mode (accelerometer, lis3dh_odr_400, lis3dh_high_res, true, true, true);
}

// Read acclerometer data and output over serial port
void read_accel()
{
    lis3dh_float_data_t  data;
    lis3dh_get_float_data(accelerometer, &data);
    printf("LIS3DH (xyz)[g] ax=%+7.3f ay=%+7.3f az=%+7.3f\n\n",
                data.ax, data.ay, data.az);
}