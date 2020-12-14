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
} states;

typedef enum {
  GRIP_OPEN = 510,       // corresponds to 0.5ms
  GRIP_CLOSE = 2200,     // corresponds to 2.2ms (2.4ms is the limit)
  LIFT_RAISE = 1000,      // corresponds to 1ms
  LIFT_LOWER = 1890,      // this is actully 9.5 which corresponds to 1.9ms
  TILT_UP = 1890,        // this is actully 9.5 which corresponds to 1.9ms
  TILT_DOWN = 1350,      // actual is  1.2ms
  BASE_WIDTH = 20000,    // 50HZ
} arm_state_t;

typedef enum {
  LIFT_RAISE_PWM = 50,      // corresponds to 1ms
  LIFT_LOWER_PWM = 88,      // corresponds to 1.8ms (1.9ms is wiping the floor)
  TILT_UP_PWM = 95,        // corresponds to 1.9ms
  TILT_DOWN_PWM = 70,      // corresponds to 1.4ms (1.2ms is fighting against lifter)
  PERIOD_PWM = 2000,       // 500HZ
} arm_state_pwm_t;

#define TICK_MULT 160
typedef enum {
  LIFT_RAISE_TICK = LIFT_RAISE_PWM * TICK_MULT,
  LIFT_LOWER_TICK = LIFT_LOWER_PWM * TICK_MULT,
  TILT_UP_TICK = TILT_UP_PWM * TICK_MULT,
  TILT_DOWN_TICK = TILT_DOWN_PWM * TICK_MULT,
} arm_state_tick_t;

#endif /* STATES_H_ */
