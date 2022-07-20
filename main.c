#include "bcm2835.h"
#include "lr1110_hal.h"
#include "lr1110_gnss.h"
#include "lr1110_wifi.h"
#include "lr1110_wifi_types.h"
#include "lr1110_system.h"
#include "lr1110_system_types.h"
#include "lr1110_gnss_types.h"
#include <stdio.h>
#include <time.h>

#define LR1110_GNSS_READ_FW_VERSION_OC 1030

//Raspberry pi bcm2835 configuration

// SNIFF LED on RPi pin GPIO 18
#define SNIFF 18

// BUSY on RPi pin GPIO 23
#define BUSY 23

// NSS on RPi pin GPIO 24
#define NSS 24

// NRESET on RPi pin GPIO 26
#define RESET 26

// T_IRQ on RPi pin GPIO 6
#define T_IRQ 6

// LNA_CTRL on RPi pin GPIO 5
#define LNA_CTRL 5

// Dummy Byte
#define LR1110_NOP ( 0x00 )

radio_t *radio = NULL;

// Initialize I2C

int init_i2c()
{
    if (!bcm2835_i2c_begin())
    {
        printf("i2c initialization failed\n");
        return 1;
    }
    bcm2835_i2c_setClockDivider(BCM2835_I2C_CLOCK_DIVIDER_150);
    bcm2835_i2c_set_baudrate(400000);	

    printf("began i2c\n");
    return 0;
}

// SHT31 Temp Sensor 

static uint8_t sht31_crc(uint8_t *data) {

    uint8_t crc = 0xff;
    int i, j;
    for(i = 0; i < 2; i++) {
        crc ^= data[i];
        for(j = 0; j < 8; j++) {
            if(crc & 0x80) {
                crc <<= 1;
                crc ^= 0x131;
            }
            else
                crc <<= 1;
        }
    }
    return crc;
}

#define MAX_LEN 32

int read_temp()
{
    // Set slave address to 0x24
    bcm2835_i2c_setSlaveAddress(0x44);

    // Command to read temp and relative humidity
    char read_cmd[2] = {0};
    read_cmd[0] = 0x2C;
    read_cmd[1] = 0x06;
    bcm2835_i2c_write(read_cmd, 2);

    delay(10);

    char data[6] = {0};
    bcm2835_i2c_read(data, 6);
    printf("data[0] = %d", data[0]);

    double cTemp = (((data[0] * 256) + data[1]) * 175.0) / 65535.0 - 45.0;
    double fTemp = (((data[0] * 256) + data[1]) * 315.0) / 65535.0 - 49.0;
    double humidity = (((data[3] * 256) + data[4])) * 100.0 / 65535.0;

    printf("Temperature in Celsius : %f\n", cTemp);
    printf("Temperature in Fahrenheit : %.2f F \n", fTemp);
    printf("Relative Humidity is : %.2f RH \n", humidity);

    return 1;
}

// Initialize SPI
int init_spi()
{

    if (!bcm2835_spi_begin())
    {
        printf("spi initialization failed\n");
        return 1;
    }

    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);      // The default

    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                   // The default
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_65536); // The default
    bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                      // The default
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);      // the default

    bcm2835_gpio_fsel(SNIFF, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(NSS, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(RESET, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_set_pud(RESET, BCM2835_GPIO_PUD_DOWN);

    bcm2835_gpio_fsel(BUSY, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_set_pud(BUSY, BCM2835_GPIO_PUD_UP);

    /*
    bcm2835_gpio_fsel(T_IRQ, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_set_pud(T_IRQ, BCM2835_GPIO_PUD_OFF);
    */

   bcm2835_gpio_fsel(LNA_CTRL, BCM2835_GPIO_FSEL_OUTP);

    return 0;
}

void init_lr1110()
{
    // System reset
    lr1110_system_reset(radio);

    //bcm2835_gpio_write(RESET, HIGH);

    // Configure DC-DC converter
    lr1110_system_set_regmode( radio, LR1110_SYSTEM_REGMODE_DCDC_CONVERTER );

    /*
    // Configure RF switch??
    lr1110_system_rfswitch_config_t rf_switch_configuration = {0};
    rf_switch_configuration.enable  = LR1110_SYSTEM_RFSW0_HIGH | LR1110_SYSTEM_RFSW1_HIGH | LR1110_SYSTEM_RFSW2_HIGH | LR1110_SYSTEM_RFSW3_HIGH ;
    rf_switch_configuration.standby = 0;
    rf_switch_configuration.rx      = LR1110_SYSTEM_RFSW0_HIGH;
    rf_switch_configuration.tx_hp   = LR1110_SYSTEM_RFSW1_HIGH;
    rf_switch_configuration.tx      = LR1110_SYSTEM_RFSW0_HIGH | LR1110_SYSTEM_RFSW1_HIGH;
    rf_switch_configuration.wifi    = LR1110_SYSTEM_RFSW3_HIGH; 
    rf_switch_configuration.gnss    = LR1110_SYSTEM_RFSW2_HIGH; 

    lr1110_system_set_dio_as_rf_switch( radio, &rf_switch_configuration );
    */

    // Initialize TCXO control
    lr1110_system_set_tcxo_mode( radio , 0x07, 500);

    // Configure Low frequency clock
    lr1110_system_config_lfclk( radio, LR1110_SYSTEM_LFCLK_XTAL, false);
    //lr1110_system_config_lfclk( radio, LR1110_SYSTEM_LFCLK_RC, false);

    // clear errors and calibrate system
    lr1110_system_clear_errors(radio);
    lr1110_system_calibrate(radio, 0x3F);

    //fetch new errors
    uint16_t errors;
    lr1110_system_get_errors(radio, &errors);
    lr1110_system_clear_errors(radio);
    lr1110_system_clear_irq(radio, LR1110_SYSTEM_IRQ_ALL_MASK);

    get_system_version();

    bcm2835_gpio_write(LNA_CTRL, HIGH);

    delay(500);

    //lr1110_system_set_dio_irq_params(radio, LR1110_SYSTEM_IRQ_ALL_MASK, 0);
    //lr1110_system_clear_irq(radio, LR1110_SYSTEM_IRQ_ALL_MASK);

}

const char* lr1110_wifi_channel_to_str( const lr1110_wifi_channel_t value )
{
    switch( value )
    {
    case LR1110_WIFI_NO_CHANNEL:
    {
        return ( const char* ) "LR1110_WIFI_NO_CHANNEL";
    }

    case LR1110_WIFI_CHANNEL_1:
    {
        return ( const char* ) "LR1110_WIFI_CHANNEL_1";
    }

    case LR1110_WIFI_CHANNEL_2:
    {
        return ( const char* ) "LR1110_WIFI_CHANNEL_2";
    }

    case LR1110_WIFI_CHANNEL_3:
    {
        return ( const char* ) "LR1110_WIFI_CHANNEL_3";
    }

    case LR1110_WIFI_CHANNEL_4:
    {
        return ( const char* ) "LR1110_WIFI_CHANNEL_4";
    }

    case LR1110_WIFI_CHANNEL_5:
    {
        return ( const char* ) "LR1110_WIFI_CHANNEL_5";
    }

    case LR1110_WIFI_CHANNEL_6:
    {
        return ( const char* ) "LR1110_WIFI_CHANNEL_6";
    }

    case LR1110_WIFI_CHANNEL_7:
    {
        return ( const char* ) "LR1110_WIFI_CHANNEL_7";
    }

    case LR1110_WIFI_CHANNEL_8:
    {
        return ( const char* ) "LR1110_WIFI_CHANNEL_8";
    }

    case LR1110_WIFI_CHANNEL_9:
    {
        return ( const char* ) "LR1110_WIFI_CHANNEL_9";
    }

    case LR1110_WIFI_CHANNEL_10:
    {
        return ( const char* ) "LR1110_WIFI_CHANNEL_10";
    }

    case LR1110_WIFI_CHANNEL_11:
    {
        return ( const char* ) "LR1110_WIFI_CHANNEL_11";
    }

    case LR1110_WIFI_CHANNEL_12:
    {
        return ( const char* ) "LR1110_WIFI_CHANNEL_12";
    }

    case LR1110_WIFI_CHANNEL_13:
    {
        return ( const char* ) "LR1110_WIFI_CHANNEL_13";
    }

    case LR1110_WIFI_CHANNEL_14:
    {
        return ( const char* ) "LR1110_WIFI_CHANNEL_14";
    }

    case LR1110_WIFI_ALL_CHANNELS:
    {
        return ( const char* ) "LR1110_WIFI_ALL_CHANNELS";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr1110_wifi_signal_type_result_to_str( const lr1110_wifi_signal_type_result_t value )
{
    switch( value )
    {
    case LR1110_WIFI_TYPE_RESULT_B:
    {
        return ( const char* ) "LR1110_WIFI_TYPE_RESULT_B";
    }

    case LR1110_WIFI_TYPE_RESULT_G:
    {
        return ( const char* ) "LR1110_WIFI_TYPE_RESULT_G";
    }

    case LR1110_WIFI_TYPE_RESULT_N:
    {
        return ( const char* ) "LR1110_WIFI_TYPE_RESULT_N";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr1110_wifi_frame_type_to_str( const lr1110_wifi_frame_type_t value )
{
    switch( value )
    {
    case LR1110_WIFI_FRAME_TYPE_MANAGEMENT:
    {
        return ( const char* ) "LR1110_WIFI_FRAME_TYPE_MANAGEMENT";
    }

    case LR1110_WIFI_FRAME_TYPE_CONTROL:
    {
        return ( const char* ) "LR1110_WIFI_FRAME_TYPE_CONTROL";
    }

    case LR1110_WIFI_FRAME_TYPE_DATA:
    {
        return ( const char* ) "LR1110_WIFI_FRAME_TYPE_DATA";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr1110_wifi_datarate_to_str( const lr1110_wifi_datarate_t value )
{
    switch( value )
    {
    case LR1110_WIFI_DATARATE_1_MBPS:
    {
        return ( const char* ) "LR1110_WIFI_DATARATE_1_MBPS";
    }

    case LR1110_WIFI_DATARATE_2_MBPS:
    {
        return ( const char* ) "LR1110_WIFI_DATARATE_2_MBPS";
    }

    case LR1110_WIFI_DATARATE_6_MBPS:
    {
        return ( const char* ) "LR1110_WIFI_DATARATE_6_MBPS";
    }

    case LR1110_WIFI_DATARATE_9_MBPS:
    {
        return ( const char* ) "LR1110_WIFI_DATARATE_9_MBPS";
    }

    case LR1110_WIFI_DATARATE_12_MBPS:
    {
        return ( const char* ) "LR1110_WIFI_DATARATE_12_MBPS";
    }

    case LR1110_WIFI_DATARATE_18_MBPS:
    {
        return ( const char* ) "LR1110_WIFI_DATARATE_18_MBPS";
    }

    case LR1110_WIFI_DATARATE_24_MBPS:
    {
        return ( const char* ) "LR1110_WIFI_DATARATE_24_MBPS";
    }

    case LR1110_WIFI_DATARATE_36_MBPS:
    {
        return ( const char* ) "LR1110_WIFI_DATARATE_36_MBPS";
    }

    case LR1110_WIFI_DATARATE_48_MBPS:
    {
        return ( const char* ) "LR1110_WIFI_DATARATE_48_MBPS";
    }

    case LR1110_WIFI_DATARATE_54_MBPS:
    {
        return ( const char* ) "LR1110_WIFI_DATARATE_54_MBPS";
    }

    case LR1110_WIFI_DATARATE_6_5_MBPS:
    {
        return ( const char* ) "LR1110_WIFI_DATARATE_6_5_MBPS";
    }

    case LR1110_WIFI_DATARATE_13_MBPS:
    {
        return ( const char* ) "LR1110_WIFI_DATARATE_13_MBPS";
    }

    case LR1110_WIFI_DATARATE_19_5_MBPS:
    {
        return ( const char* ) "LR1110_WIFI_DATARATE_19_5_MBPS";
    }

    case LR1110_WIFI_DATARATE_26_MBPS:
    {
        return ( const char* ) "LR1110_WIFI_DATARATE_26_MBPS";
    }

    case LR1110_WIFI_DATARATE_39_MBPS:
    {
        return ( const char* ) "LR1110_WIFI_DATARATE_39_MBPS";
    }

    case LR1110_WIFI_DATARATE_52_MBPS:
    {
        return ( const char* ) "LR1110_WIFI_DATARATE_52_MBPS";
    }

    case LR1110_WIFI_DATARATE_58_MBPS:
    {
        return ( const char* ) "LR1110_WIFI_DATARATE_58_MBPS";
    }

    case LR1110_WIFI_DATARATE_65_MBPS:
    {
        return ( const char* ) "LR1110_WIFI_DATARATE_65_MBPS";
    }

    case LR1110_WIFI_DATARATE_7_2_MBPS:
    {
        return ( const char* ) "LR1110_WIFI_DATARATE_7_2_MBPS";
    }

    case LR1110_WIFI_DATARATE_14_4_MBPS:
    {
        return ( const char* ) "LR1110_WIFI_DATARATE_14_4_MBPS";
    }

    case LR1110_WIFI_DATARATE_21_7_MBPS:
    {
        return ( const char* ) "LR1110_WIFI_DATARATE_21_7_MBPS";
    }

    case LR1110_WIFI_DATARATE_28_9_MBPS:
    {
        return ( const char* ) "LR1110_WIFI_DATARATE_28_9_MBPS";
    }

    case LR1110_WIFI_DATARATE_43_3_MBPS:
    {
        return ( const char* ) "LR1110_WIFI_DATARATE_43_3_MBPS";
    }

    case LR1110_WIFI_DATARATE_57_8_MBPS:
    {
        return ( const char* ) "LR1110_WIFI_DATARATE_57_8_MBPS";
    }

    case LR1110_WIFI_DATARATE_65_2_MBPS:
    {
        return ( const char* ) "LR1110_WIFI_DATARATE_65_2_MBPS";
    }

    case LR1110_WIFI_DATARATE_72_2_MBPS:
    {
        return ( const char* ) "LR1110_WIFI_DATARATE_72_2_MBPS";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

void print_mac_address( const char* prefix, lr1110_wifi_mac_address_t mac )
{
    printf( "%s%02x:%02x:%02x:%02x:%02x:%02x\n", prefix, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5] );
}

void wifi_fetch_and_print_scan_basic_complete_results( const void* radio )
{

    uint8_t n_results = 0;
    lr1110_wifi_get_nb_results(radio, &n_results);

    printf("numb results : %d\n", n_results);

    for( uint8_t result_index = 0; result_index < n_results; result_index++ )
    {
        lr1110_wifi_basic_complete_result_t local_result = { 0 };

        lr1110_wifi_read_basic_complete_results(radio, result_index, 1, &local_result);

        lr1110_wifi_mac_origin_t mac_origin    = LR1110_WIFI_ORIGIN_BEACON_FIX_AP;
        lr1110_wifi_channel_t    channel       = LR1110_WIFI_NO_CHANNEL;
        bool                     rssi_validity = false;
        lr1110_extract_channel_info(local_result.channel_info_byte, &channel, &rssi_validity, &mac_origin);

        lr1110_wifi_frame_type_t     frame_type     = LR1110_WIFI_FRAME_TYPE_MANAGEMENT;
        lr1110_wifi_frame_sub_type_t frame_sub_type = 0;
        bool                         to_ds          = false;
        bool                         from_ds        = false;
        lr1110_extract_frame_type_info(local_result.frame_type_info_byte, &frame_type, &frame_sub_type, &to_ds, &from_ds );

        printf( "Result %u/%u\n", result_index + 1, n_results );
        print_mac_address( "  -> MAC address: ", local_result.mac_address );
        printf( "  -> Channel: %s\n", lr1110_wifi_channel_to_str( channel ) );
        printf( "  -> MAC origin: %s\n", ( rssi_validity ? "From gateway" : "From end device" ) );
        /*
        printf(
            "  -> Signal type: %s\n",
            lr1110_wifi_signal_type_result_to_str(
                lr1110_wifi_extract_signal_type_from_data_rate_info( local_result.data_rate_info_byte ) ) );
                */
        printf( "  -> Frame type: %s\n", lr1110_wifi_frame_type_to_str( frame_type ) );
        printf( "  -> Frame sub-type: 0x%02X\n", frame_sub_type );
        printf( "  -> FromDS/ToDS: %s / %s\n", ( ( from_ds == true ) ? "true" : "false" ),
                              ( ( to_ds == true ) ? "true" : "false" ) );
        printf( "  -> Phi Offset: %i\n", local_result.phi_offset );
        printf( "  -> Timestamp: %lu us\n", local_result.timestamp_us );
        printf( "  -> Beacon period: %u TU\n", local_result.beacon_period_tu );
        printf( "\n" );
    }
}

/**
 * @brief Assistance position latitude
 */
#define GNSS_ASSISTANCE_POSITION_LATITUDE ( 40.7 )

/**
 * @brief Assistance position longitude
 */
#define GNSS_ASSISTANCE_POSITION_LONGITUDE ( -74 )

/**
 * @brief GNSS input parameters
 */

#define GNSS_INPUT_PARAMETERS ( 0x07 )

/**
 * @brief Maximum number of SV to search per scan
 */
#define GNSS_MAX_SV ( 20 )

/**
 * @brief Maximum number of scan to execute in a row
 */
#define GNSS_MAX_SCANS ( 100 )

/*
typedef struct
{
    lr1110_gnss_scan_mode_t   scan_mode;
    lr1110_gnss_search_mode_t effort_mode;
    uint8_t                   input_parameters;
    uint8_t                   max_sv;
} gnss_configuration_t;

const gnss_configuration_t gnss_configuration = {
    .scan_mode        = 0x01,
    .effort_mode      = 0x01,
    .input_parameters = GNSS_INPUT_PARAMETERS,
    .max_sv           = GNSS_MAX_SV,
};
*/

typedef struct
{
    lr1110_gnss_solver_assistance_position_t assistance_position;
    lr1110_gnss_scan_mode_t                  scan_mode;
    lr1110_gnss_search_mode_t                effort_mode;
    uint8_t                                  input_parameters;
    uint8_t                                  max_sv;
} gnss_configuration_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

const gnss_configuration_t gnss_configuration = {
    .assistance_position = { .latitude  = GNSS_ASSISTANCE_POSITION_LATITUDE,
                             .longitude = GNSS_ASSISTANCE_POSITION_LONGITUDE },
    //.scan_mode           = 3,
    .scan_mode           = LR1110_GNSS_SINGLE_SCAN_MODE,
    .effort_mode         = LR1110_GNSS_OPTION_BEST_EFFORT,
    .input_parameters    = GNSS_INPUT_PARAMETERS,
    .max_sv              = GNSS_MAX_SV,
};

#define APP_PARTIAL_SLEEP true
#define NAV_MAX_LENGTH ( 300 )
//#define IRQ_MASK ( LR1110_SYSTEM_IRQ_GNSS_SCAN_DONE )

#define HAL_DBG_TRACE_ARRAY( msg, array, len )                             \
do                                                                         \
{                                                                          \
    printf("%s - (%lu bytes):\n", msg, ( uint32_t )len );    \
    for( uint32_t i = 0; i < ( uint32_t )len; i++ )                        \
    {                                                                      \
        if( ( ( i % 16 ) == 0 ) && ( i > 0 ) )                             \
        {                                                                  \
            printf("\n");                                    \
        }                                                                  \
        printf( " %02X", array[i] );                         \
    }                                                                      \
    printf( "\n" );                                          \
} while ( 0 );

static uint32_t number_of_scan = 0;

static void gnss_fetch_and_print_results()
{
    printf( "== Scan #%lu ==\n", number_of_scan );

    lr1110_gnss_timings_t gnss_timing = { 0 };
    lr1110_gnss_get_timings( radio, &gnss_timing);
    printf( "Timings:\n" );
    printf( "  - radio: %u ms\n", gnss_timing.radio_ms );
    printf( "  - computation: %u ms\n", gnss_timing.computation_ms );

    uint8_t                          n_sv_detected            = 0;
    lr1110_gnss_detected_satellite_t sv_detected[GNSS_MAX_SV] = { 0 };
    lr1110_gnss_get_nb_detected_satellites( radio, &n_sv_detected );
    if( n_sv_detected > GNSS_MAX_SV )
    {
        printf( "More SV detected than configured (detected %u, max is %u)\n", n_sv_detected,
                             GNSS_MAX_SV );
        return;
    }

    lr1110_gnss_get_detected_satellites(radio, n_sv_detected, sv_detected);

    uint16_t result_size         = 0;
    uint8_t  nav[NAV_MAX_LENGTH] = { 0 };
    lr1110_gnss_get_result_size( radio, &result_size );
    printf("result size = %d\n", result_size);
    if( result_size > NAV_MAX_LENGTH )
    {
        printf( "Result size too long (size %u, max is %u)\n", result_size, NAV_MAX_LENGTH );
        return;
    }

    lr1110_gnss_read_results( radio, nav, result_size);
   
    printf( "Detected %u SV(s):\n", n_sv_detected );
    for( uint8_t index_sv = 0; index_sv < n_sv_detected; index_sv++ )
    {
        const lr1110_gnss_detected_satellite_t* local_sv = &sv_detected[index_sv];
        printf( "  - SV %u: CNR: %i\n", local_sv->satellite_id, local_sv->cnr);
    }

    //printf( "NAV message : %u\n", nav );
    HAL_DBG_TRACE_ARRAY("NAV message", nav, result_size);
}

void myGPS_scan(const void* context)
{
    //delay(10000);
    time_t now = time(0);
    const lr1110_gnss_date_t gnss_time = (uint32_t)now;
    printf("now: %u \n", gnss_time);
    uint8_t inter_cap_delay_s;

    lr1110_gnss_set_scan_mode(context, LR1110_GNSS_SINGLE_SCAN_MODE, &inter_cap_delay_s);
    //lr1110_gnss_set_scan_mode(context, gnss_time, &inter_cap_delay_s);

    //lr1110_gnss_set_assistance_position( context, &gnss_configuration.assistance_position );

    lr1110_gnss_scan_autonomous( context, 0, gnss_configuration.input_parameters, gnss_configuration.max_sv );

    // Assisted scan
    //lr1110_gnss_scan_assisted( context, now, gnss_configuration.effort_mode, gnss_configuration.input_parameters, gnss_configuration.max_sv );

}

//#define WIFI_CHANNEL_MASK ( 0x0421 )
#define WIFI_CHANNEL_MASK ( 0x0421 )
#define WIFI_MAX_RESULTS ( 10 )
#define WIFI_MAX_NUMBER_OF_SCAN ( 0 )
//#define WIFI_SIGNAL_TYPE ( LR1110_WIFI_TYPE_SCAN_B )
#define WIFI_SIGNAL_TYPE ( 4 )
#define WIFI_SCAN_MODE ( LR1110_WIFI_SCAN_MODE_BEACON_AND_PACKET )
#define WIFI_RESULT_FORMAT ( LR1110_WIFI_RESULT_FORMAT_BASIC_COMPLETE )
#define WIFI_NB_SCAN_PER_CHANNEL ( 10 )
#define WIFI_TIMEOUT_PER_SCAN ( 90 )
#define WIFI_ABORT_ON_TIMEOUT ( true )
#define WIFI_TIMEOUT_PER_CHANNEL ( 1000 )


typedef struct
{
    lr1110_wifi_channel_mask_t channel_mask;
    uint8_t                    max_result;
} wifi_configuration_scan_base_t;

typedef struct
{
    wifi_configuration_scan_base_t base;
    lr1110_wifi_signal_type_scan_t signal_type;
    lr1110_wifi_mode_t             scan_mode;
    uint8_t                        nb_scan_per_channel;
    uint16_t                       timeout_per_scan;
    bool                           abort_on_timeout;
} wifi_configuration_scan_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static const wifi_configuration_scan_t wifi_configuration = {
    .signal_type         = WIFI_SIGNAL_TYPE,
    .base.channel_mask   = WIFI_CHANNEL_MASK,
    .scan_mode           = WIFI_SCAN_MODE,
    .base.max_result     = WIFI_MAX_RESULTS,
    .nb_scan_per_channel = WIFI_NB_SCAN_PER_CHANNEL,
    .timeout_per_scan    = WIFI_TIMEOUT_PER_SCAN,
    .abort_on_timeout    = WIFI_ABORT_ON_TIMEOUT,
};

void myWifi_scan(const void* radio)
{
    lr1110_wifi_reset_cumulative_timing(radio);

    

    lr1110_wifi_scan(radio, wifi_configuration.signal_type, wifi_configuration.base.channel_mask,
                                        wifi_configuration.scan_mode, wifi_configuration.base.max_result,
                                        wifi_configuration.nb_scan_per_channel, wifi_configuration.timeout_per_scan,
                                        wifi_configuration.abort_on_timeout);
}

void wifi_fetch_and_print_results()
{

    lr1110_wifi_cumulative_timings_t cumulative_timings = { 0 };
    lr1110_wifi_read_cumulative_timing( radio, &cumulative_timings);

    printf( "== Scan #%lu ==\n", number_of_scan );
    printf( "Cummulative timings:\n" );
    printf( "  -> Demodulation: %lu us\n", cumulative_timings.demodulation_us );
    printf( "  -> Capture: %lu us\n", cumulative_timings.rx_capture_us );
    printf( "  -> Correlation: %lu us\n", cumulative_timings.rx_correlation_us );
    printf( "  -> Detection: %lu us\n", cumulative_timings.rx_detection_us );
    printf( "  => Total : %lu us\n",
                          cumulative_timings.demodulation_us + cumulative_timings.rx_capture_us +
                              cumulative_timings.rx_correlation_us + cumulative_timings.rx_detection_us );
    printf( "\n" );

    wifi_fetch_and_print_scan_basic_complete_results( radio);
}
void get_system_version()
{
    lr1110_system_version_t system_version;
    lr1110_system_get_version(radio, &system_version);
    printf( "  - Firmware = 0x%04X\n", system_version.fw );
    printf( "  - Hardware = 0x%02X\n", system_version.hw );
    printf("  - Type     = 0x%02X (0x01 for LR1110, 0x02 for LR1120)\n", system_version.type );
}

void get_gnss_version()
{
    lr1110_gnss_version_t gnss_version;
    lr1110_gnss_read_firmware_version(radio, &gnss_version);
    printf("gnss firmware : %u\n", gnss_version.gnss_firmware);
    printf("gnss almanac : %u\n", gnss_version.gnss_almanac);
}

void print_system_misc()
{
    uint16_t temp = 0;
    lr1110_system_get_temp(radio, &temp );
    printf("lr1110 temp = %u\n", (unsigned int)temp);

    uint8_t vbat = 0;
    lr1110_system_get_vbat(radio, &vbat );
    printf("lr1110 vbat = %u\n", (unsigned int)vbat);
}

int main(int argc, char **argv)
{
    if (!bcm2835_init())
    {
        printf("cant init bcm lib\n");
        return 1;
    }

    init_spi();

    init_i2c();

    read_temp();

    init_lr1110();

    myWifi_scan(radio);

    while(bcm2835_gpio_lev(BUSY) == HIGH){
        printf("Im busy\n");
    };

    wifi_fetch_and_print_results(radio);

    init_lr1110();
    
    myGPS_scan(radio);
    
    while(bcm2835_gpio_lev(BUSY) == HIGH){
        printf("Im busy\n");
    };

    gnss_fetch_and_print_results();
    
    bcm2835_spi_end();
    
    bcm2835_i2c_end();
    bcm2835_close();

    return 0;
    
}
