// BLE advertisement template
//
// Advertises device name: EE149

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "nrf.h"
#include "app_util.h"
#include "nrf_twi_mngr.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "buckler.h"
#include "simple_ble.h"
#include "display.h"
#include "lsm9ds1.h"

#include "opt3004.h"

// Create a timer
// APP_TIMER_DEF(adv_timer);

// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

// Intervals for advertising and connections
static simple_ble_config_t ble_config = {
        // c0:98:e5:49:xx:xx
        .platform_id       = 0x49,    // used as 4th octect in device BLE address
        .device_id         = 0x5354,  // TODO: replace with your lab bench number
        .adv_name          = "EE149", // Note that this name is not displayed to save room in the advertisement for data.
        .adv_interval      = MSEC_TO_UNITS(1000, UNIT_0_625_MS),
        .min_conn_interval = MSEC_TO_UNITS(500, UNIT_1_25_MS),
        .max_conn_interval = MSEC_TO_UNITS(1000, UNIT_1_25_MS),
};

void light_timer_callback() {
    printf("Light timer fired!\n");
    // TODO: implement this function!
    // Use Simple BLE function to read light sensor and put data in advertisement
}

float read_tilt(){
	lsm9ds1_measurement_t g = lsm9ds1_read_accelerometer();
	float x_val = g.x_axis;
 	float y_val = g.y_axis;
 	float z_val = g.z_axis;

 	float psi = atan(y_val/sqrt(x_val*x_val + z_val*z_val)) * 180/ M_PI;

 	return psi;

}


/*******************************************************************************
 *   State for this application
 ******************************************************************************/
// Main application state
simple_ble_app_t* simple_ble_app;

int main(void) {
  ret_code_t error_code = NRF_SUCCESS;

  // Initialize

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized\n");

  // initialize i2c master (two wire interface)
  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  i2c_config.scl = BUCKLER_SENSORS_SCL;
  i2c_config.sda = BUCKLER_SENSORS_SDA;
  i2c_config.frequency = NRF_TWIM_FREQ_100K;
  error_code = nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);
  APP_ERROR_CHECK(error_code);
  lsm9ds1_init(&twi_mngr_instance);

  /*
  opt3004_config_t config = {
    .range_number = OPT3004_AUTORANGE,
    .conversion_time = OPT3004_CONVERSION_100MS,
    .latch_interrupt = 1,
    .interrupt_polarity = OPT3004_INTERRUPT_ACTIVE_LO,
    .fault_count = OPT3004_FAULT_COUNT_1,
  };

  // initialize opt3004 driver
  opt3004_init(&twi_mngr_instance);
  error_code = opt3004_config(config);
  // set up opt3004 to read continuously
  opt3004_continuous();

  printf("opt3004 initialized: %ld\n", error_code);
  */

  // Setup BLE
  simple_ble_app = simple_ble_init(&ble_config);

  // simple_ble_adv_only_name();
  unsigned char payload[24] = {0};
  int y_tilt = 0;

  // Set a timer to read the light sensor and update advertisement data every second.
  /*
  app_timer_init();
  app_timer_create(&adv_timer, APP_TIMER_MODE_REPEATED, (app_timer_timeout_handler_t) light_timer_callback);
  app_timer_start(adv_timer, APP_TIMER_TICKS(1000), NULL); // 1000 milliseconds
  */

  while(1) {
  	y_tilt = floor(read_tilt());
  	payload[0] = floor((y_tilt / 10));
  	payload[1] = (y_tilt % 10);
  	simple_ble_adv_manuf_data(payload, 2);
   
    nrf_delay_ms(100);
    // Sleep while SoftDevice handles BLE
    //power_manage();
  }
}

