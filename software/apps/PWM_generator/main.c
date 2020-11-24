// Robot Template app
//
// Framework for creating applications that control the Kobuki robot

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <app_pwm.h>
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
#include "gpio.h"

// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

// global variables
KobukiSensors_t sensors = {0};
int ws_L = 0;
int ws_R = 0;

typedef enum {
  GRIP_OPEN = 2,       // this is actually 2.5 which corresponds to 0.5ms
  GRIP_CLOSE = 11,     // corresponds to 2.4ms
  LIFT_RAISE = 5,      // corresponds to 1ms
  LIFT_LOWER = 9,      // this is actully 9.5 which corresponds to 1.9ms
  PIVOT_UP = 9,        // this is actully 9.5 which corresponds to 1.9ms
  PIVOT_DOWN = 6,      // corresponds to 1.2ms 
} robot_state_t;

APP_PWM_INSTANCE(PWM2,2);
int GRIP_DEFAULT = GRIP_CLOSE;
int PIVOT_DEFAULT = PIVOT_UP;
int LIFT_DEFAULT = LIFT_RAISE;
// APP_PWM_INSTANCE(PWM2,1);
// app_pwm_config_t pwm2_config = {
//   .pins ={BUCKLER_LED1, APP_PWM_NOPIN},
//   .pin_polarity = {APP_PWM_POLARITY_ACTIVE_LOW, APP_PWM_POLARITY_ACTIVE_LOW},
//   .num_of_channels = 1,
//   .period_us = 20000,
// };


//app_pwm_config_t pwm1_config = APP_PWM_DEFAULT_CONFIG_1CH(20000L, BUCKLER_LED0);
//app_pwm_config_t pwm2_config = APP_PWM_DEFAULT_CONFIG_1CH(20000L, BUCKLER_LED1);
//pwm1_config.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;
//pwm2_config.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;

void funny_function(uint32_t pwm_id)    // PWM callback function
{
    //printf("%d\n", pwm_id);
}

// LIFT_SERVO
void raise_lift_servo(){
  while(LIFT_DEFAULT > LIFT_RAISE){
    printf("raising--> pulse_value: %dus\n", LIFT_DEFAULT*200 );
    LIFT_DEFAULT -= 1;
    while (app_pwm_channel_duty_set(&PWM2, 0, LIFT_DEFAULT)== NRF_ERROR_BUSY);
    nrf_delay_ms(20);
  }
  while (app_pwm_channel_duty_set(&PWM2, 0, LIFT_DEFAULT)== NRF_ERROR_BUSY);
  
}
void lower_lift_servo(){
  while(LIFT_DEFAULT < LIFT_LOWER){
    printf("lowering--> pulse_value: %dus\n", LIFT_DEFAULT*200  );
    LIFT_DEFAULT += 1;
    while (app_pwm_channel_duty_set(&PWM2, 0, LIFT_DEFAULT)== NRF_ERROR_BUSY);
    nrf_delay_ms(20);
  }
  while (app_pwm_channel_duty_set(&PWM2, 0, LIFT_DEFAULT)== NRF_ERROR_BUSY);
  
}

/// PIVOT_SERVO
void up_pivot_servo(){
  while(PIVOT_DEFAULT < PIVOT_UP){
    PIVOT_DEFAULT += 1;
    while (app_pwm_channel_duty_set(&PWM2, 1, PIVOT_DEFAULT)== NRF_ERROR_BUSY);
    nrf_delay_ms(20);
  }
  while (app_pwm_channel_duty_set(&PWM2, 1, PIVOT_DEFAULT)== NRF_ERROR_BUSY);
  
}
void down_pivot_servo(){
  while (PIVOT_DEFAULT > PIVOT_DOWN){
    PIVOT_DEFAULT -= 1;
    while (app_pwm_channel_duty_set(&PWM2, 1, PIVOT_DEFAULT)== NRF_ERROR_BUSY);
    nrf_delay_ms(20);
  }
  while (app_pwm_channel_duty_set(&PWM2, 1, 0)== NRF_ERROR_BUSY);
  
}
// GRIP_SERVO
void open_grip_servo(){
  while(GRIP_DEFAULT > GRIP_OPEN){
    printf("opening--> pulse_value: %dus\n", GRIP_DEFAULT*200 );
    GRIP_DEFAULT -= 1;
    while (app_pwm_channel_duty_set(&PWM2, 1, GRIP_DEFAULT)== NRF_ERROR_BUSY);
    nrf_delay_ms(200);
  }
  while (app_pwm_channel_duty_set(&PWM2, 1, 0)== NRF_ERROR_BUSY);
  
}
void close_grip_servo(){
  while(GRIP_DEFAULT < GRIP_CLOSE){
    printf("closing--> pulse_value: %dus\n", GRIP_DEFAULT*200 );
    GRIP_DEFAULT += 1;
    while (app_pwm_channel_duty_set(&PWM2, 1, GRIP_DEFAULT)== NRF_ERROR_BUSY);
    nrf_delay_ms(200);
  }
  while (app_pwm_channel_duty_set(&PWM2, 1, 0)== NRF_ERROR_BUSY);
}

int main(void) {
  ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");

  

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

  
  gpio_config(19, INPUT);
  gpio_set(19);

  app_pwm_config_t pwm2_config = {
  .pins ={BUCKLER_LED0, BUCKLER_LED1},
  .pin_polarity = {APP_PWM_POLARITY_ACTIVE_HIGH, APP_PWM_POLARITY_ACTIVE_HIGH},
  .num_of_channels = 2,
  .period_us = 20000,
  };


  error_code = app_pwm_init(&PWM2, &pwm2_config, funny_function);
  printf("before error check\n");
  APP_ERROR_CHECK(error_code);
  app_pwm_enable(&PWM2);
  int duty[5] = {5,9,6,3,12};
  int value = 0;
  // loop forever, running state machine
  while (1) {
    //XXX
    //nrf_delay_ms(500);
    // read sensors from robot
    lower_lift_servo();
    open_grip_servo();
    printf("gpio read value %d,",gpio_read(19));
    nrf_delay_ms(2000);   // increase this delay when testing !!!!!
    printf(" after open GRIP_DEFAULT %d,LIFT_DEFAULT %d\n",PIVOT_DEFAULT,LIFT_DEFAULT );
    raise_lift_servo();
    close_grip_servo();
    printf("gpio read value %d,",gpio_read(19));
    nrf_delay_ms(2000); //  increase this delay !!!!!!!
    printf(" after close GRIP_DEFAULT %d,LIFT_DEFAULT %d\n",GRIP_DEFAULT,LIFT_DEFAULT );
  }
}

