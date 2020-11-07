// BLE RX app
//
// Receives BLE advertisements with data

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "app_util.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "simple_ble.h"

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

void ble_evt_adv_report(ble_evt_t const* p_ble_evt) {
  ble_gap_evt_adv_report_t const* adv_report = &(p_ble_evt->evt.gap_evt.params.adv_report);
  uint8_t* data = NULL;
  uint8_t  index = 0;
  uint8_t  len = 0;
  uint8_t  type = 0;
  uint8_t* payload = NULL;
  uint8_t  payload_len = 0;
  int i;

  // DONE: extract the fields we care about (Peer address and data)
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
      // DONE: get length of field
      // DONE: get type of field: if type is 0xFF, we found it!
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
}

int main(void) {
  ret_code_t error_code = NRF_SUCCESS;

  // Initialize

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized\n");

  // Setup BLE
  // Note: simple BLE is our own library. You can find it in `nrf5x-base/lib/simple_ble/`
  simple_ble_app = simple_ble_init(&ble_config);
  advertising_stop();

  // TODO: Start scanning
  scanning_start();

  while(1) {
    // Sleep while SoftDevice handles BLE
    power_manage();
  }
}



