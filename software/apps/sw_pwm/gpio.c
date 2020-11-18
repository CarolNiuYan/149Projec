#include "gpio.h"

gpio_pin*	pins = 		(gpio_pin*) gpio_pin_base;
gpio_pin_cnf* pin_cfg =	(gpio_pin_cnf*) gpio_pin_cnf_base;

// Inputs: 
//  gpio_num - gpio number 0-31
//  dir - gpio direction (INPUT, OUTPUT)
void gpio_config(uint8_t gpio_num, gpio_direction_t dir) {
	uint32_t mask = 1 << 0 | 1 << 1;
	if (dir == INPUT) {
		pin_cfg->PIN_CNF[gpio_num] = pin_cfg->PIN_CNF[gpio_num] & ~mask;
	} else if (dir == OUTPUT) {
		pin_cfg->PIN_CNF[gpio_num] = pin_cfg->PIN_CNF[gpio_num] | mask;
	}
}

// Set gpio_num high
// Inputs: 
//  gpio_num - gpio number 0-31
void gpio_set(uint8_t gpio_num) {
	pins->OUTSET = (1 << gpio_num);
}

// Set gpio_num low
// Inputs: 
//  gpio_num - gpio number 0-31
void gpio_clear(uint8_t gpio_num) {
	pins->OUTCLR = (1 << gpio_num);
}

// Inputs: 
//  gpio_num - gpio number 0-31
bool gpio_read(uint8_t gpio_num) {
    // should return pin state
    bool state = (pins->IN) & (1<<gpio_num);
    return state;
}
