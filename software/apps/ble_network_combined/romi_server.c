//#include <curses.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>

#include "nrf.h"
#include "app_util.h"
#include "nrf_twi_mngr.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "gpio.h"

#include "buckler.h"
#include "simple_ble.h"
#include "display.h"
#include "lsm9ds1.h"

#include "opt3004.h"

#include "socket.h"
#include "ui.h"
#include "network.h"

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

#define ADDRESS "73.63.163.204"
#define PORT 53535
simple_ble_app_t* simple_ble_app;

/*
 * Get arm speed (in ticks) from tilt
 */
#define X_DEADZ 10
#define Y_DEADZ 10
#define CAP 200
void get_as(struct tilt *tilt, int* as_lift, int* as_tilt)
{
  // Lift
  if (fabs(tilt->psi) < Y_DEADZ) {
    *as_lift = 0;
  } else {
    *as_lift = ((int) floor(tilt->psi)) * 3;
  }
  // Tilt
  if (fabs(tilt->theta) < X_DEADZ) {
    *as_tilt = 0;
  } else {
    *as_tilt = ((int) floor(tilt->theta)) * 2;
  }
  *as_lift = (*as_lift) > CAP ? CAP : (((*as_lift) < -CAP) ? -CAP : *as_lift);
  *as_tilt = (*as_tilt) > CAP ? CAP : (((*as_tilt) < -CAP) ? -CAP : *as_tilt);
}

void display_arm(struct tilt *tilt, int arm_lift, int arm_tilt, int graper) {
  char buf[16] = {0};
  snprintf(buf, 16, "L:%d T:%d G:%d", arm_lift, arm_tilt, graper);
  display_write(buf, DISPLAY_LINE_1);
  snprintf(buf, 16, "Y:%d  X:%d", (int)tilt->psi, (int)tilt->theta);
  display_write(buf, DISPLAY_LINE_0);
}

int main (int argc, char** argv) {
	int rc;
	// Check the argument number is correct
  if(argc != 1) {
    fprintf(stderr, "Usage: <server>\n");
    exit(1);
  }

  // Declare sockets and port
  int server_socket_fd;
  unsigned short server_port;
  int socket_fd;
  float client_msg[2]; // [ws_L, ws_R]

  // Initialize the home game
  open_control_room(&server_port, &server_socket_fd);

  // Initialize the home control display
  init_home(server_port);
  // Accept connection
  accept_connection(server_socket_fd, &socket_fd);
  

  //Set up robot side
  ret_code_t error_code = NRF_SUCCESS;

  // Initialize
  struct tilt tilt_angle;
  int ws_L = 0;
  int ws_R = 0;
  //int arm_lift = arm_lift_neutral;
  //int arm_tilt = arm_tilt_neutral;
  int as_lift = 0;
  int as_tilt = 0;
  bool as_grapper = 0;
  gpio_config(22, INPUT);
  gpio_config(28, INPUT);

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
  display_write("INIT", DISPLAY_LINE_0);


  // Setup BLE
  #define BLE_ADV_SIZE 16
  simple_ble_app = simple_ble_init(&ble_config);
  unsigned char payload[BLE_ADV_SIZE] = {0};
  for (int i=0; i<BLE_ADV_SIZE; i++) {
    payload[i] = 0xFF;
  }


  while(1) {
    // take care of arm speed 
    get_as(&tilt_angle, &as_lift, &as_tilt);
    if (button_pressed()) as_grapper = !as_grapper;
    display_arm(&tilt_angle, as_lift, as_tilt, as_grapper);
    payload[9] = as_grapper;
    payload[10] = abs(as_lift);
    payload[11] = (as_lift < 0) ? 1 : 0;
    payload[12] = abs(as_tilt);
    payload[13] = (as_tilt < 0) ? 1 : 0; 

    //obtain data of wheel speed
    get_client_input(socket_fd, &client_msg);

    int ws_L = client_msg[0];
    int ws_R = client_msg[1];
    // printf("L: %d,  R: %d\n", ws_L, ws_R);
    payload[2] = abs(ws_L);
    payload[3] = (ws_L < 0) ? 1 : 0;
    payload[4] = abs(ws_R);
    payload[5] = (ws_R < 0) ? 1 : 0;

    simple_ble_adv_manuf_data(&payload[2], BLE_ADV_SIZE - 2);
    nrf_delay_ms(100);
}
