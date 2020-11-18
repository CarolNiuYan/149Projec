// Blink app
//
// Blinks the LEDs on Buckler

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_error.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_serial.h"

#include "buckler.h"
#include "gpio.h"

int main(void) {
	ret_code_t error_code = NRF_SUCCESS;

	// initialize RTT library
	error_code = NRF_LOG_INIT(NULL);
	APP_ERROR_CHECK(error_code);
	NRF_LOG_DEFAULT_BACKENDS_INIT();
	printf("Log initialized!\n");

	gpio_config(25, OUTPUT);

	// Gripper (low and high are no-op if increment=0)
	int pulse_width = 2400;
	int low = 510;
	int high = 2390;
	

	// Lifter (low and high are no-op if increment=0)
	/*
	int pulse_width = 1900;
	int low = 980;
	int high = 1890;
	*/

	// Pivot (low and high are no-op if increment=0)
	/*
	int pulse_width = 1900;
	int low = 1210; // On my setup it can move freely as low as 1400, OR it will fight against the lifter
	int high = 1890;
	*/
	
	// 50HZ base freq.
	int base_width = 20000;
	
	// int increment = -20; // Used to move it back and forth
	int increment = 0;

	while (1) {
		gpio_set(25);
		nrf_delay_us(pulse_width);
		gpio_clear(25);
		nrf_delay_us(base_width - pulse_width);
		// No-op if increment = 0
		pulse_width = pulse_width + increment;
		if (pulse_width < low || pulse_width > high) {increment = -increment;}
	}
}



