#include "gpio.h"

// Inputs: 
//  gpio_num - gpio number 0-31
//  dir - gpio direction (INPUT, OUTPUT)
registers *regs = (registers *) BASE;
void gpio_config(uint8_t gpio_num, gpio_direction_t dir) {
	if(dir == OUTPUT){
		// set the last 2 bits to 11 so the input buffer is disconnected
		regs->pin_cnf[gpio_num] = 3;
	} else if (dir == INPUT)
	{   // set last 2 bits to 00 so the input buffer is connected
		regs->pin_cnf[gpio_num] = dir;
	}
}

// Set gpio_num high
// Inputs: 
//  gpio_num - gpio number 0-31
void gpio_set(uint8_t gpio_num) {
	regs->outset = (1 << gpio_num);
}

// Set gpio_num low
// Inputs: 
//  gpio_num - gpio number 0-31
void gpio_clear(uint8_t gpio_num) {
	regs->outclr = (1 << gpio_num);
}

// Inputs: 
//  gpio_num - gpio number 0-31
bool gpio_read(uint8_t gpio_num) {
    // should return pin state
    return ( regs->in & (1 << gpio_num) ) >> gpio_num;
}
