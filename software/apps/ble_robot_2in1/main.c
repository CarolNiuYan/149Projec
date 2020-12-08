// Robot Template app
//
// Framework for creating applications that control the Kobuki robot

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_pwm.h"
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
#define BKL_LED2 23

// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

// global variables
states state = OFF;
KobukiSensors_t sensors = {0};
int ws_L = 0;
int ws_R = 0;
int arm_tilt = (TILT_UP_PWM + TILT_DOWN_PWM) / 2;
int arm_lift = (LIFT_RAISE_PWM + LIFT_LOWER_PWM) / 2;
int arm_grip = 0;
int as_lift = 0;
int as_tilt = 0;
uint16_t curr_arm_tilt;
uint16_t curr_arm_lift;
bool cur_arm_grip;

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



// PWM generator instance
APP_PWM_INSTANCE(PWM2,2);
// PWM callback function
void funny_function(uint32_t pwm_id) { /*Do nothing*/ }


// Arm Actions
void lift_to_pwm(int pwm) {
  if (!pwm) {
    while (app_pwm_channel_duty_set(&PWM2, 0, 0) == NRF_ERROR_BUSY);
    return;
  }
  if (curr_arm_lift < pwm * TICK_MULT) curr_arm_lift+=100;
  else if (curr_arm_lift > pwm * TICK_MULT) curr_arm_lift-=100;
  while (app_pwm_channel_duty_ticks_set(&PWM2, 0, curr_arm_lift) == NRF_ERROR_BUSY);
}
void tilt_to_pwm(int pwm) {
  if (!pwm) {
    while (app_pwm_channel_duty_set(&PWM2, 1, 0) == NRF_ERROR_BUSY);
    return;
  }
  if (curr_arm_tilt < pwm * TICK_MULT) curr_arm_tilt+=100;
  else if (curr_arm_tilt > pwm * TICK_MULT) curr_arm_tilt-=100;
  while (app_pwm_channel_duty_ticks_set(&PWM2, 1, curr_arm_tilt) == NRF_ERROR_BUSY);
}

void lift_move(int arm_speed) {
  curr_arm_lift += arm_speed;
  if (curr_arm_lift < LIFT_RAISE_TICK || curr_arm_lift > LIFT_LOWER_TICK) {
    curr_arm_lift -= arm_speed;
  }
  while (app_pwm_channel_duty_ticks_set(&PWM2, 0, curr_arm_lift) == NRF_ERROR_BUSY);
}
void tilt_move(int arm_speed) {
  curr_arm_tilt += arm_speed;
  if (curr_arm_tilt < TILT_DOWN_TICK || curr_arm_tilt > TILT_UP_TICK) {
    curr_arm_tilt -= arm_speed;
  }
  while (app_pwm_channel_duty_ticks_set(&PWM2, 1, curr_arm_tilt) == NRF_ERROR_BUSY);
}

// Grip action
void update_grip(int grip) {
  int i, pulse_width;
  if (grip > 0 && !cur_arm_grip) {
    pulse_width = GRIP_CLOSE;
    cur_arm_grip = true;
  } else if (grip == 0 && cur_arm_grip) {
    pulse_width = GRIP_OPEN;
    cur_arm_grip = false;
  } else {
    return;
  }
  // generate a short PWM signal
  for (i=0; i<8; i++) {
		gpio_set(BKL_LED2);
		nrf_delay_us(pulse_width);
		gpio_clear(BKL_LED2);
		nrf_delay_us(BASE_WIDTH - pulse_width);
	}
}

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
     * payload[6] = Switch (Not used)
     * payload[7] = Lift_PWM (Not used)
     * payload[8] = Tilt_PWM (Not used)
     * payload[9] = Gripper_bool
     * payload[10] = Lift_speed_tick
     * payload[11] = Lift_speed_dir
     * payload[12] = Tilt_speed_tick
     * payload[13] = Tilt_speed_dir
     */
    /*
     * Driving
     */
    if (payload[2] < 255) {
      ws_L = payload[2];
      if (ws_L > MAX_WS) ws_L = MAX_WS;
      if (payload[3]) ws_L = -ws_L;
    }
    if (payload[4] < 255) {
      ws_R = payload[4];
      if (ws_R > MAX_WS) ws_R = MAX_WS;
      if (payload[5]) ws_R = -ws_R;
    }

    /*
     * Arm actions
     */
    /* (Not used)
    if (payload[7] < 255) {
      arm_lift = payload[7];
      if (arm_lift < LIFT_RAISE_PWM) arm_lift = LIFT_RAISE_PWM;
      else if (arm_lift > LIFT_LOWER_PWM) arm_lift = LIFT_LOWER_PWM;
    }
    if (payload[8] < 255) {
      arm_tilt = payload[8];
      if (arm_tilt < TILT_DOWN_PWM) arm_tilt = TILT_DOWN_PWM;
      else if (arm_tilt > TILT_UP_PWM) arm_tilt = TILT_UP_PWM;
    }
    */
    if (payload[9] < 255) {
      arm_grip = payload[9];
    }
    if (payload[10] < 255 && payload[7] == 255) {
      as_lift = payload[10];
      if (payload[11]) as_lift = -as_lift;
    }
    if (payload[12] < 255 && payload[8] == 255) {
      as_tilt = payload[12];
      if (payload[13]) as_tilt = -as_tilt;
    }
  }
}


void update_display(void) {
  char buf[16] = {0};
  const char* state_str = "NULL";

  switch(state){
  case OFF:
    state_str = "OFF";
    break;
  case DRIVING:
    state_str = "RUN";
    break;
  }
  snprintf(buf, 16, "%s L%d R%d", state_str, ws_L, ws_R);
	display_write(buf, DISPLAY_LINE_0);
  snprintf(buf, 16, "L:%d T:%d G:%d",
           curr_arm_lift / TICK_MULT,
           curr_arm_tilt / TICK_MULT,
           cur_arm_grip);
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
  gpio_config(BKL_LED2, OUTPUT);
  gpio_clear(BKL_LED2);

  // initialize Kobuki
  kobukiInit();
  ws_L = 0;
  ws_R = 0;
  printf("Kobuki initialized!\n");
  state = OFF;

  // BLE Scanning start
  scanning_start();
  printf("BLE Scanning enabled!\n");

  // Init PWM2 generator
  app_pwm_config_t pwm2_config = {
  .pins ={BUCKLER_LED0, BUCKLER_LED1},
  .pin_polarity = {APP_PWM_POLARITY_ACTIVE_HIGH, APP_PWM_POLARITY_ACTIVE_HIGH},
  .num_of_channels = 2,
  .period_us = PERIOD_PWM,
  };
  error_code = app_pwm_init(&PWM2, &pwm2_config, funny_function);
  APP_ERROR_CHECK(error_code);
  app_pwm_enable(&PWM2);
  printf("Checking tick cycles\n");
  ASSERT(app_pwm_cycle_ticks_get(&PWM2) == TICK_MULT * 100);
  printf("PWM generator Inited!\n");

  // Init arm location
  arm_tilt = (TILT_UP_PWM + TILT_DOWN_PWM) / 2;
  arm_lift = (LIFT_RAISE_PWM + LIFT_LOWER_PWM) / 2;
  arm_grip = 0;
  curr_arm_tilt = arm_tilt * TICK_MULT;
  curr_arm_lift = arm_lift * TICK_MULT;
  cur_arm_grip = false;


  // loop forever, running state machine
  while (1) {
    //XXX
    nrf_delay_ms(10);
    // read sensors from robot
    kobukiSensorPoll(&sensors);
    update_display();

    // state machine
    switch(state) {
      case OFF: {
        // transition logic
        if (any_button_pressed()) {
          state = DRIVING;
        } else {
          state = OFF;
          // perform state-specific actions here
          kobukiDriveDirect(0, 0);
          lift_to_pwm(0);
          tilt_to_pwm(0);
        }
        break;
      }
      case DRIVING: {
        // transition logic
        if (any_button_pressed()) {
          state = OFF;
        } else {
          state = DRIVING;
          // perform state-specific actions here
          kobukiDriveDirect(ws_L, ws_R);
          lift_move(as_lift);
          tilt_move(as_tilt);
          update_grip(arm_grip);
          /* (not used)
          lift_to_pwm(arm_lift);
          tilt_to_pwm(arm_tilt);
          */
        }
        break;
      }
      default: {
        state = OFF;
      }
    }
  }
}

