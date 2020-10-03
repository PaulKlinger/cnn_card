/* 
 * File:   led_status.h
 * Author: kling
 *
 * Created on 27 September 2020, 17:58
 */

#ifndef LED_STATUS_H
#define	LED_STATUS_H

#include <stdint.h>
#include "config.h"

#ifdef	__cplusplus
extern "C" {
#endif

/* 3-bit brightness values for each led*/
uint8_t led_status[LED_COUNT * PWM_BITS / 8 + 1];

uint8_t get_led_brightness(uint8_t row, uint8_t col);

void set_led_brightness(uint8_t row, uint8_t col, uint8_t val);

uint8_t value_to_pwm_level(float value, float max_value);

void init_pwm_data();

void update_pwm_pattern();

void turn_off_leds();

void run_pwm_cycle();


#ifdef	__cplusplus
}
#endif

#endif	/* LED_STATUS_H */

