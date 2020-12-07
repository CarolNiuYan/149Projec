// Robot Template app
//
// Framework for creating applications that control the Kobuki robot

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_util.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_spi.h"

#include "buckler.h"
#include "display.h"
#include "kobukiActuator.h"
#include "kobukiSensorPoll.h"
#include "kobukiSensorTypes.h"
#include "kobukiUtilities.h"
#include "lsm9ds1.h"
#include "simple_ble.h"

#include "states.h"
#include "gpio.h"

#define MAX_WS 130
#define BKL_BT0 28

// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

// global variables
KobukiSensors_t sensors = {0};
int ws_L = 0;
int ws_R = 0;

// BLE configuration
// This is mostly irrelevant since we are scanning only
static simple_ble_config_t ble_config = {
        // BLE address is c0:98:e5:49:00:00
        .platform_id       = 0x49,    // used as 4th octet in device BLE address
        .device_id         = 0x5352,  // Last two octets of device address
        .adv_name          = "EE149", // irrelevant in this example
        .adv_interval      = MSEC_TO_UNITS(1000, UNIT_0_625_MS), // send a packet once per second (minimum is 20 ms)
        .min_conn_interval = MSEC_TO_UNITS(500, UNIT_1_25_MS), // irrelevant if advertising only
        .max_conn_interval = MSEC_TO_UNITS(1000, UNIT_1_25_MS), // irrelevant if advertising only
};
simple_ble_app_t* simple_ble_app;


/*
 * BLE scanner callback
 */
void ble_evt_adv_report(ble_evt_t const* p_ble_evt) {
  ble_gap_evt_adv_report_t const* adv_report = &(p_ble_evt->evt.gap_evt.params.adv_report);
  uint8_t* data = NULL;
  uint8_t  index = 0;
  uint8_t  len = 0;
  uint8_t  type = 0;
  uint8_t* payload = NULL;
  uint8_t  payload_len = 0;
  int i;

  // DONE: filter on Peer address
  // replace with condition on peer address
  if (adv_report->peer_addr.addr[0] == 0x54 &&
      adv_report->peer_addr.addr[1] == 0x53 &&
      // .......
      adv_report->peer_addr.addr[5] == 0xc0) {

    // printf("Match!!\n");
    // printf("Data length %d\n", adv_report->data.len);
    // if address matches C0:98:E5:49:53:54, loop until we find field 0xFF
    data = adv_report->data.p_data;
    do {
      if (index >= adv_report->data.len) {
        printf("Cannot parse data\n");
        return;
      }
      // get length of field, type of field:
      // if type is 0xFF, we found it!
      len = data[index];
      type = data[index+1];
      payload = &data[index+2];
      payload_len = len - 1;
      index += len + 1;
    } while (type != 0xFF);
    // Adv is now located.

    if (payload_len < 3 || payload_len > adv_report->data.len) {
        printf("Invalid payload_len %d\n", payload_len);
        return;
    }

    // Printing
    if (false) {
      // Print peer addr
      printf("Message from [ ");
      for (i = BLE_GAP_ADDR_LEN-1; i >= 0; i--) {
        printf("%X ", adv_report->peer_addr.addr[i]);
      }
      printf("]:\n");

      // Print data
      printf("Company:[%X][%X]\n", payload[1], payload[0]);
      printf("Data:");
      for (i = 2; i < payload_len; i++) {
        printf("[%X]", payload[i]);
      }
      printf("\n\n");
    }

    // Save data
    /*
     * payload[2] = LW_speed,
     * payload[3] = LW_dir
     * payload[4] = RW_speed,
     * payload[5] = RW_dir
     */
    ws_L = payload[2];
    if (ws_L > MAX_WS) ws_L = MAX_WS;
    if (payload[3]) ws_L = -ws_L;
    ws_R = payload[4];
    if (ws_R > MAX_WS) ws_R = MAX_WS;
    if (payload[5]) ws_R = -ws_R;
  }
}



void print_state(states current_state){
	switch(current_state){
	case OFF:
		display_write("OFF", DISPLAY_LINE_0);
		break;
	case DRIVING:
		display_write("DRIVING", DISPLAY_LINE_0);
		break;
    }
}

void print_ws(void) {
  char buf[16] = {0};
  snprintf(buf, 16, "L:%d  R:%d", ws_L, ws_R);
	display_write(buf, DISPLAY_LINE_1);
}

bool any_button_pressed(void) {
  if (!gpio_read(BKL_BT0)) {
    nrf_delay_ms(200); //button de-bouncer
    return true;
  }
  return is_button_pressed(&sensors);
}

int main(void) {
  ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");

  // Setup BLE
  simple_ble_app = simple_ble_init(&ble_config);
  advertising_stop();

  // initialize LEDs
  /*
  nrf_gpio_pin_dir_set(23, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(24, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(25, NRF_GPIO_PIN_DIR_OUTPUT);
  */

  // initialize display
  nrf_drv_spi_t spi_instance = NRF_DRV_SPI_INSTANCE(1);
  nrf_drv_spi_config_t spi_config = {
    .sck_pin = BUCKLER_LCD_SCLK,
    .mosi_pin = BUCKLER_LCD_MOSI,
    .miso_pin = BUCKLER_LCD_MISO,
    .ss_pin = BUCKLER_LCD_CS,
    .irq_priority = NRFX_SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
    .orc = 0,
    .frequency = NRF_DRV_SPI_FREQ_4M,
    .mode = NRF_DRV_SPI_MODE_2,
    .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
  };
  error_code = nrf_drv_spi_init(&spi_instance, &spi_config, NULL, NULL);
  APP_ERROR_CHECK(error_code);
  display_init(&spi_instance);
  printf("Display initialized!\n");

  // initialize i2c master (two wire interface)
  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  i2c_config.scl = BUCKLER_SENSORS_SCL;
  i2c_config.sda = BUCKLER_SENSORS_SDA;
  i2c_config.frequency = NRF_TWIM_FREQ_100K;
  error_code = nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);
  APP_ERROR_CHECK(error_code);
  lsm9ds1_init(&twi_mngr_instance);
  printf("IMU initialized!\n");

  // Button input
  gpio_config(BKL_BT0, INPUT);

  // initialize Kobuki
  kobukiInit();
  ws_L = 0;
  ws_R = 0;
  printf("Kobuki initialized!\n");
  states state = OFF;

  // BLE Scanning start
  scanning_start();
  printf("BLE Scanning enabled!\n");

  // loop forever, running state machine
  while (1) {
    //XXX
    //nrf_delay_ms(500);
    // read sensors from robot
    kobukiSensorPoll(&sensors);
    print_ws();

    // state machine
    switch(state) {
      case OFF: {
        print_state(state);

        // transition logic
        if (any_button_pressed()) {
          state = DRIVING;
        } else {
          state = OFF;
          // perform state-specific actions here
          kobukiDriveDirect(0, 0);
        }
        break;
      }
      case DRIVING: {
        print_state(state);

        // transition logic
        if (any_button_pressed()) {
          state = OFF;
        } else {
          state = DRIVING;
          // perform state-specific actions here
          kobukiDriveDirect(ws_L, ws_R);
        }
        break;
      }
      default: {
        state = OFF;
      }
    }
  }
}

