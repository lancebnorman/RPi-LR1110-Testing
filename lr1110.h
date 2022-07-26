/*
    Interface for communicating between Raspberry Pi 4 Model B and LR1110 over SPI
 */

#include <stdio.h>

// BCM2835 library
#include "bcm2835/bcm2835.h"

// LR1110 module libraries
#include "lr1110/lr1110_hal.h"
#include "lr1110/lr1110_gnss.h"
#include "lr1110/lr1110_wifi.h"
#include "lr1110/lr1110_wifi_types.h"
#include "lr1110/lr1110_system.h"
#include "lr1110/lr1110_system_types.h"
#include "lr1110/lr1110_gnss_types.h"

//Raspberry pi bcm2835 <---> LR1110 configuration

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

// Initialize SPI bus
int init_spi()
{
    // Begin spi operations via bcm2835 
    if (!bcm2835_spi_begin())
    {
        printf("spi initialization failed\n");
        return 1;
    }

    // Configure SPI bus
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);      // The default
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                   // The default
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_65536); // The default
    bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                      // The default
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);      // the default

    return 0;
}

// End SPI bus
void close_spi()
{
    bcm2835_spi_end();
}

radio_t *radio = NULL;

// Get firmware version and type
void get_system_version()
{
    lr1110_system_version_t system_version;
    lr1110_system_get_version(radio, &system_version);
    printf( "  - Firmware = 0x%04X\n", system_version.fw );
    printf( "  - Hardware = 0x%02X\n", system_version.hw );
    printf("  - Type     = 0x%02X (0x01 for LR1110, 0x02 for LR1120)\n\n", system_version.type );
}

// Initialize lr1110 
void init_lr1110()
{
    printf("----------- Initializing LR1110 ----------- \n\n");

    // Configure gpio outputs
    bcm2835_gpio_fsel(SNIFF, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(NSS, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(RESET, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_set_pud(RESET, BCM2835_GPIO_PUD_DOWN);
    bcm2835_gpio_fsel(LNA_CTRL, BCM2835_GPIO_FSEL_OUTP);

    // Configure BUSY and IRQ as outputs
    bcm2835_gpio_fsel(BUSY, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_set_pud(BUSY, BCM2835_GPIO_PUD_UP);
    bcm2835_gpio_fsel(T_IRQ, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_set_pud(T_IRQ, BCM2835_GPIO_PUD_OFF);
   
    // System reset
    lr1110_system_reset(radio);

    // Configure DC-DC converter
    lr1110_system_set_regmode( radio, LR1110_SYSTEM_REGMODE_DCDC_CONVERTER );

    // Configure RF switch
    lr1110_system_rfswitch_config_t rf_switch_configuration = {0};
    rf_switch_configuration.enable  = LR1110_SYSTEM_RFSW0_HIGH | LR1110_SYSTEM_RFSW1_HIGH | LR1110_SYSTEM_RFSW2_HIGH | LR1110_SYSTEM_RFSW3_HIGH ;
    rf_switch_configuration.standby = 0;
    rf_switch_configuration.rx      = LR1110_SYSTEM_RFSW0_HIGH;
    rf_switch_configuration.tx_hp   = LR1110_SYSTEM_RFSW1_HIGH;
    rf_switch_configuration.tx      = LR1110_SYSTEM_RFSW0_HIGH | LR1110_SYSTEM_RFSW1_HIGH;
    rf_switch_configuration.wifi    = LR1110_SYSTEM_RFSW3_HIGH; 
    rf_switch_configuration.gnss    = LR1110_SYSTEM_RFSW2_HIGH; 
    lr1110_system_set_dio_as_rf_switch( radio, &rf_switch_configuration );
    
    // Initialize TCXO control
    lr1110_system_set_tcxo_mode( radio , 0x07, 1000);

    // Configure Low frequency clock
    //lr1110_system_config_lfclk( radio, LR1110_SYSTEM_LFCLK_XTAL, false);
    lr1110_system_config_lfclk( radio, LR1110_SYSTEM_LFCLK_RC, false);

    // Clear errors and calibrate system
    lr1110_system_clear_errors(radio);
    lr1110_system_calibrate(radio, 0x3F);

    // Fetch new errors
    uint16_t errors;
    lr1110_system_get_errors(radio, &errors);
    lr1110_system_clear_errors(radio);
    lr1110_system_clear_irq(radio, LR1110_SYSTEM_IRQ_ALL_MASK);

    // Retreive and print system version
    get_system_version();

    bcm2835_gpio_write(LNA_CTRL, HIGH);

    delay(500);

    lr1110_system_set_dio_irq_params(radio, LR1110_SYSTEM_IRQ_ALL_MASK, 0);
    lr1110_system_clear_irq(radio, LR1110_SYSTEM_IRQ_ALL_MASK);

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

// --------------------------- Passive GNS Scanning -------------------------------

#define GNSS_INPUT_PARAMETERS ( 0x07 )
#define GNSS_MAX_SV ( 10 )
#define GNSS_MAX_SCANS ( 100 )
#define NAV_MAX_LENGTH ( 300 )

#define HAL_DBG_TRACE_ARRAY( msg, array, len )                             \
do                                                                         \
{                                                                          \
    printf("%s - (%u bytes):\n", msg, ( uint32_t )len );    \
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

// Initiate an autonomous gnss scan
void scan_gnss_autonomous()
{
    printf("----------- Starting Autonomous GNSS Scan ----------- \n Scanning...\n\n ");

    // Current time
    uint8_t inter_cap_delay_s;

    //lr1110_gnss_set_scan_mode(context, LR1110_GNSS_SINGLE_SCAN_MODE, &inter_cap_delay_s);
    lr1110_gnss_set_scan_mode(radio, 3, &inter_cap_delay_s);

    lr1110_gnss_scan_autonomous( radio, 0, GNSS_INPUT_PARAMETERS, GNSS_MAX_SV );

    while(bcm2835_gpio_lev(BUSY) == HIGH){};

}

// TODO
// Initiate an assited gnss scan
void scan_gnss_assisted()
{
    /*
    printf("----------- Starting Assisted GNSS Scan ----------- \n Scanning...\n\n ");

    // Current time
    time_t now = time(0);
    const lr1110_gnss_date_t gnss_time = (uint32_t)now;
    uint8_t inter_cap_delay_s;

    lr1110_gnss_set_scan_mode(context, LR1110_GNSS_SINGLE_SCAN_MODE, &inter_cap_delay_s);

    while(bcm2835_gpio_lev(BUSY) == HIGH){};
    */
}

// Get gnss scan results currently in lr1110 buffer and print  
static void gnss_fetch_and_print_results()
{

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

    HAL_DBG_TRACE_ARRAY("NAV message", nav, result_size);
}

// Get and print gnss firmware and almanac version
void get_gnss_version()
{
    lr1110_gnss_version_t gnss_version;
    lr1110_gnss_read_firmware_version(radio, &gnss_version);
    printf("gnss firmware : %u\n", gnss_version.gnss_firmware);
    printf("gnss almanac : %u\n", gnss_version.gnss_almanac);
}

// --------------------------- Passive WiFi Scanning -------------------------------

#define WIFI_CHANNEL_MASK ( 0x0421 )
#define WIFI_MAX_RESULTS ( 10 )
#define WIFI_MAX_NUMBER_OF_SCAN ( 0 )
#define WIFI_SIGNAL_TYPE ( 4 )
#define WIFI_SCAN_MODE ( LR1110_WIFI_SCAN_MODE_BEACON_AND_PACKET )
#define WIFI_RESULT_FORMAT ( LR1110_WIFI_RESULT_FORMAT_BASIC_COMPLETE )
#define WIFI_NB_SCAN_PER_CHANNEL ( 10 )
#define WIFI_TIMEOUT_PER_SCAN ( 90 )
#define WIFI_ABORT_ON_TIMEOUT ( true )
#define WIFI_TIMEOUT_PER_CHANNEL ( 1000 )

// Initiate a wifi scan and wait until complete by checking BUSY signal
void scan_wifi()
{   
    printf("----------- Starting Wifi Scan ----------- \n\n");
    lr1110_wifi_reset_cumulative_timing(radio);

    lr1110_wifi_scan(radio, WIFI_SIGNAL_TYPE, WIFI_CHANNEL_MASK, WIFI_SCAN_MODE, WIFI_MAX_RESULTS, 
                            WIFI_NB_SCAN_PER_CHANNEL, WIFI_TIMEOUT_PER_SCAN, WIFI_ABORT_ON_TIMEOUT);

    while(bcm2835_gpio_lev(BUSY) == HIGH){};
}

// Print mac address as formatted string
void print_mac_address( const char* prefix, lr1110_wifi_mac_address_t mac )
{
    printf( "%s%02x:%02x:%02x:%02x:%02x:%02x\n", prefix, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5] );
}

// Get wifi scan results currently in lr1110 buffer and print  
void wifi_fetch_and_print_scan_results()
{
    // Get number of results from scan
    uint8_t n_results = 0;
    lr1110_wifi_get_nb_results(radio, &n_results);
    printf("numb results : %d\n\n", n_results);

    // Get basic complete results from lr1110_wifi library for each result
    for( uint8_t result_index = 0; result_index < n_results; result_index++ )
    {
        // Read basic result parameters
        lr1110_wifi_basic_complete_result_t local_result = { 0 };
        lr1110_wifi_read_basic_complete_results(radio, result_index, 1, &local_result);

        // Extract channel info
        lr1110_wifi_mac_origin_t mac_origin = LR1110_WIFI_ORIGIN_BEACON_FIX_AP;
        lr1110_wifi_channel_t    channel = LR1110_WIFI_NO_CHANNEL;
        bool rssi_validity = false;
        lr1110_extract_channel_info(local_result.channel_info_byte, &channel, &rssi_validity, &mac_origin);

        // Extract frame info
        lr1110_wifi_frame_type_t     frame_type     = LR1110_WIFI_FRAME_TYPE_MANAGEMENT;
        lr1110_wifi_frame_sub_type_t frame_sub_type = 0;
        bool                         to_ds          = false;
        bool                         from_ds        = false;
        lr1110_extract_frame_type_info(local_result.frame_type_info_byte, &frame_type, &frame_sub_type, &to_ds, &from_ds );

        // Format and print relevent info
        printf( "Result %u/%u\n", result_index + 1, n_results );
        print_mac_address( "  -> MAC address: ", local_result.mac_address );
        //printf( "  -> Channel: %s\n", channel );
        printf( "  -> MAC origin: %s\n", ( rssi_validity ? "From gateway" : "From end device" ) );
        //printf( "  -> Frame type: %s\n", frame_type );
        printf( "  -> Frame sub-type: 0x%02X\n", frame_sub_type );
        printf( "  -> FromDS/ToDS: %s / %s\n", ( ( from_ds == true ) ? "true" : "false" ),
                              ( ( to_ds == true ) ? "true" : "false" ) );
        printf( "  -> Phi Offset: %i\n", local_result.phi_offset );
        printf( "  -> Timestamp: %llu us\n", local_result.timestamp_us );
        printf( "  -> Beacon period: %u TU\n", local_result.beacon_period_tu );
        printf( "\n" );
    }
}

// Print scan timing results
void wifi_get_timings()
{
    // Read cumulative timings
    lr1110_wifi_cumulative_timings_t cumulative_timings = { 0 };
    lr1110_wifi_read_cumulative_timing( radio, &cumulative_timings);

    // Format and print relevent info
    printf( "Cummulative timings:\n" );
    printf( "  -> Demodulation: %u us\n", cumulative_timings.demodulation_us );
    printf( "  -> Capture: %u us\n", cumulative_timings.rx_capture_us );
    printf( "  -> Correlation: %u us\n", cumulative_timings.rx_correlation_us );
    printf( "  -> Detection: %u us\n", cumulative_timings.rx_detection_us );
    printf( "  => Total : %u us\n",
                          cumulative_timings.demodulation_us + cumulative_timings.rx_capture_us +
                              cumulative_timings.rx_correlation_us + cumulative_timings.rx_detection_us );
    printf( "\n" );

}