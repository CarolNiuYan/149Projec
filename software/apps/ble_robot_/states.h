/*
 * states.h
 *
 *  Created on: Sep 22, 2018
 *      Author: shromonaghosh
 */

#ifndef STATES_H_
#define STATES_H_

#include <stdio.h>

typedef enum {
    OFF=0,
    DRIVING,
    ARM,
} states;


typedef enum {
  GRIP_OPEN = 490,       // this is actually 2.5 which corresponds to 0.5ms
  GRIP_CLOSE = 2390,     // corresponds to 2.4ms
  LIFT_RAISE = 1000,      // corresponds to 1ms
  LIFT_LOWER = 1890,      // this is actully 9.5 which corresponds to 1.9ms
  PIVOT_UP = 1890,        // this is actully 9.5 which corresponds to 1.9ms
  PIVOT_DOWN = 1350,      // actual is  1.2ms 
} arm_state_t;

#endif /* STATES_H_ */
