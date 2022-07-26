#include <stdio.h>

// LR1110 Interface
#include "lr1110.h"

// Sensor Interface
#include "sensors.h"

int main(int argc, char **argv)
{
    printf("\n\n\n");
    
    init_io();

    init_spi();

    init_i2c();

    read_temp();

    init_accel();

    read_accel();

    init_lr1110();

    scan_wifi();

    wifi_fetch_and_print_scan_results();

    //init_lr1110();

    delay(100);
    
    scan_gnss_autonomous();

    gnss_fetch_and_print_results();
    
    close_spi();
    
    close_i2c();
    
    close_io();

    return 0;
    
}
