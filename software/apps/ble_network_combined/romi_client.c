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

//#include "gpu.h"
//#include "info.h"
//#include "util.h"

#define ADDRESS "73.63.163.204"
#define PORT 53535

struct tilt
{
  float theta;
  float psi;
  float phi;
};

void read_tilt_avg(struct tilt *result){
  static float x_val_0;
  static float x_val_1;
  static float x_val_2;
  static float y_val_0;
  static float y_val_1;
  static float y_val_2;
  static float z_val_0;
  static float z_val_1;
  static float z_val_2;

  lsm9ds1_measurement_t g = lsm9ds1_read_accelerometer();
  x_val_2 = x_val_1;
  x_val_1 = x_val_0;
  x_val_0 = g.x_axis;
  y_val_2 = y_val_1;
  y_val_1 = y_val_0;
  y_val_0 = g.y_axis;
  z_val_2 = z_val_1;
  z_val_1 = z_val_0;
  z_val_0 = g.z_axis;

  float x_val = (x_val_2 + x_val_1 + x_val_0) / 3.0;
  float y_val = (y_val_2 + y_val_1 + y_val_0) / 3.0;
  float z_val = (z_val_2 + z_val_1 + z_val_0) / 3.0;

  result->psi = atan(y_val/sqrt(x_val*x_val + z_val*z_val)) * 180/ M_PI;
  result->theta = atan(x_val/sqrt(y_val*y_val + z_val*z_val)) * 180/ M_PI;
}

/*
 * Get wheel speed from accel (Smoothed version)
 */
#define X_DEADZ 10
#define Y_DEADZ 10
#define X_MULT  3
#define Y_MULT  4
void get_ws_smooth(struct tilt *tilt, int *ws_L, int* ws_R)
{
  // Forward / backward
  if (fabs(tilt->psi) <= Y_DEADZ) {
    *ws_L = 0;
  } else if (tilt->psi > 0) {
    // Forwrd
    *ws_L = ((int) floor(tilt->psi - Y_DEADZ)) * Y_MULT;
  } else {
    // Backward
    *ws_L = ((int) floor(tilt->psi + Y_DEADZ)) * Y_MULT;
  }
  *ws_R = *ws_L;
  // Turn left / right
  if (fabs(tilt->theta) <= X_DEADZ) {
    // Don't turn
  } else if (tilt->theta > 0) {
    // Turn left
    *ws_L -= ((int) floor(tilt->theta - X_DEADZ)) * X_MULT;
    *ws_R += ((int) floor(tilt->theta - X_DEADZ)) * X_MULT;
  } else {
    // Turn right
    *ws_L -= ((int) floor(tilt->theta + X_DEADZ)) * X_MULT;
    *ws_R += ((int) floor(tilt->theta + X_DEADZ)) * X_MULT;
  }
  *ws_R = (*ws_R) > CAP ? CAP : (((*ws_R) < -CAP) ? -CAP : *ws_R);
  *ws_L = (*ws_L) > CAP ? CAP : (((*ws_L) < -CAP) ? -CAP : *ws_L);
}
#undef X_DEADZ
#undef Y_DEADZ
#undef CAP

/*
 * Display data on LCD
 */
void display_ws(struct tilt *tilt, int ws_L, int ws_R) {
  char buf[16] = {0};
  snprintf(buf, 16, "L:%d  R:%d", ws_L, ws_R);
  display_write(buf, DISPLAY_LINE_1);
  snprintf(buf, 16, "Y:%d  X:%d", (int)tilt->psi, (int)tilt->theta);
  display_write(buf, DISPLAY_LINE_0);
}

int main (int argc, char** argv) {
	int rc;
	// Check the argument number is correct
  if(argc != 1) {
    fprintf(stderr, "Usage: <Client>\n");
    exit(1);
  }

  // Declare sockets and port
  int server_socket_fd;
  unsigned short server_port;
  int socket_fd;
  int client_msg[3];



  // Unpack arguments
  char* peer_hostname = ADDRESS;
  unsigned short peer_port = PORT;

  // Join the host's game
  join_control_room(peer_hostname, peer_port, &socket_fd);
  printf("Sucessfully joined control server %s at %d\n", peer_hostname, peer_port);

  // getting wheel speed data and transmit signal
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
    read_tilt_avg(&tilt_angle);

    get_ws_smooth(&tilt_angle, &ws_L, &ws_R);
    display_ws(&tilt_angle, ws_L, ws_R);

    send_input(socket_fd, ws_L, ws_R);
    nrf_delay_ms(100);
}
