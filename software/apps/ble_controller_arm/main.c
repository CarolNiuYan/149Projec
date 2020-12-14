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
#include "gpio.h"

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

struct tilt
{
  float theta;
  float psi;
  float phi;
};

void read_tilt(struct tilt *result){
  lsm9ds1_measurement_t g = lsm9ds1_read_accelerometer();
  float x_val = g.x_axis;
   float y_val = g.y_axis;
   float z_val = g.z_axis;

   result->psi = atan(y_val/sqrt(x_val*x_val + z_val*z_val)) * 180/ M_PI;
  result->theta = atan(x_val/sqrt(y_val*y_val + z_val*z_val)) * 180/ M_PI;
}

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
 * Get wheel speed from accel
 */
#define X_DEADZ 10
#define Y_DEADZ 10
#define CAP 150
void get_ws(struct tilt *tilt, int *ws_L, int* ws_R)
{
  // Forward / backward
  if (fabs(tilt->psi) < Y_DEADZ) {
    *ws_L = 0;
    *ws_R = 0;
  } else {
    *ws_L = ((int) floor(tilt->psi)) * 3;
    *ws_R = *ws_L;
  }
  // Turn left / right
  if (fabs(tilt->theta) < X_DEADZ) {
    // Don't turn
  } else {
    *ws_L -= ((int) floor(tilt->theta)) * 2;
    *ws_R += ((int) floor(tilt->theta)) * 2;
  }
  *ws_R = (*ws_R) > CAP ? CAP : (((*ws_R) < -CAP) ? -CAP : *ws_R);
  *ws_L = (*ws_L) > CAP ? CAP : (((*ws_L) < -CAP) ? -CAP : *ws_L);
}
#undef X_DEADZ
#undef Y_DEADZ

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

bool button_pressed(void) {
  if (!gpio_read(28)) {
    nrf_delay_ms(100); //button de-bouncer
    return true;
  }
  return false;
}

#if 0  //Not used for now
/*
 * Arm parameters
 * XXX Kept in Sync with Robot's code 
 */
typedef enum {
  LIFT_RAISE_PWM = 50,      // corresponds to 1ms
  LIFT_LOWER_PWM = 90,      // corresponds to 1.8ms (1.9ms is wiping the floor)
  TILT_UP_PWM = 95,        // corresponds to 1.9ms
  TILT_DOWN_PWM = 70,      // corresponds to 1.4ms (1.2ms is fighting against lifter)
} arm_state_pwm_t;
const int arm_lift_neutral = (LIFT_RAISE_PWM + LIFT_LOWER_PWM) / 2;
const int arm_tilt_neutral = (TILT_UP_PWM + TILT_DOWN_PWM) / 2;

/*
 * Get arm position from accel
 */
#define X_DEADZ 5
#define Y_DEADZ 5
void get_arm_pos(struct tilt *tilt, int *arm_lift, int* arm_tilt)
{
  // Lift
  if (fabs(tilt->psi) <= Y_DEADZ) {
    *arm_lift = arm_lift_neutral;
  } else if (tilt->psi > 0) {
    // Lower
    *arm_lift = arm_lift_neutral + ((int) floor(tilt->psi - Y_DEADZ)) / 2;
  } else {
    // Higher
    *arm_lift = arm_lift_neutral + ((int) floor(tilt->psi + Y_DEADZ)) / 2;
  }
  if (*arm_lift > LIFT_LOWER_PWM) *arm_lift = LIFT_LOWER_PWM;
  if (*arm_lift < LIFT_RAISE_PWM) *arm_lift = LIFT_RAISE_PWM;

  // Tilt
  if (fabs(tilt->theta) <= X_DEADZ) {
    *arm_tilt = arm_tilt_neutral;
  } else if (tilt->theta > 0) {
    // Up
    *arm_tilt = arm_tilt_neutral + ((int) floor(tilt->theta - Y_DEADZ)) / 3;
  } else {
    // Down
    *arm_tilt = arm_tilt_neutral + ((int) floor(tilt->theta + Y_DEADZ)) / 3;
  }
  if (*arm_tilt > TILT_UP_PWM) *arm_tilt = TILT_UP_PWM;
  if (*arm_tilt < TILT_DOWN_PWM) *arm_tilt = TILT_DOWN_PWM;
}
#endif

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

void display_arm(struct tilt *tilt, int arm_lift, int arm_tilt, int graper) {
  char buf[16] = {0};
  snprintf(buf, 16, "L:%d T:%d G:%d", arm_lift, arm_tilt, graper);
  display_write(buf, DISPLAY_LINE_1);
  snprintf(buf, 16, "Y:%d  X:%d", (int)tilt->psi, (int)tilt->theta);
  display_write(buf, DISPLAY_LINE_0);
}


/*******************************************************************************
 *   State for this application
 ******************************************************************************/
// Main application state
simple_ble_app_t* simple_ble_app;

int main(void) {
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
  int i;
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
  for (i=0; i<BLE_ADV_SIZE; i++) {
    payload[i] = 0xFF;
  }

  // Set a timer to read the light sensor and update advertisement data every second.
  /*
  app_timer_init();
  app_timer_create(&adv_timer, APP_TIMER_MODE_REPEATED, (app_timer_timeout_handler_t) light_timer_callback);
  app_timer_start(adv_timer, APP_TIMER_TICKS(1000), NULL); // 1000 milliseconds
  */

  while(1) {

    // read tilt
    read_tilt_avg(&tilt_angle);
    // printf("Xt: %d,  Yt: %d\n", (int)tilt_angle.theta, (int)tilt_angle.psi);

    if (gpio_read(22)) {
      get_ws_smooth(&tilt_angle, &ws_L, &ws_R);
      display_ws(&tilt_angle, ws_L, ws_R);
      // printf("L: %d,  R: %d\n", ws_L, ws_R);
      payload[2] = abs(ws_L);
      payload[3] = (ws_L < 0) ? 1 : 0;
      payload[4] = abs(ws_R);
      payload[5] = (ws_R < 0) ? 1 : 0;
      for (i=6; i<14; i++) {payload[i] = 0xFF;}
    } else {
      /*
       * (Not used)
        get_arm_pos(&tilt_angle, &arm_lift, &arm_tilt);
        display_arm(&tilt_angle, arm_lift, arm_tilt, 0);
        payload[7] = arm_lift;
        payload[8] = arm_tilt;
      */
      get_as(&tilt_angle, &as_lift, &as_tilt);
      if (button_pressed()) as_grapper = !as_grapper;
      display_arm(&tilt_angle, as_lift, as_tilt, as_grapper);
      payload[9] = as_grapper;
      payload[10] = abs(as_lift);
      payload[11] = (as_lift < 0) ? 1 : 0;
      payload[12] = abs(as_tilt);
      payload[13] = (as_tilt < 0) ? 1 : 0;
      for (i=2; i<=6; i++) {payload[i] = 0xFF;}
    }
    simple_ble_adv_manuf_data(&payload[2], BLE_ADV_SIZE - 2);
    nrf_delay_ms(100);
  }
}

