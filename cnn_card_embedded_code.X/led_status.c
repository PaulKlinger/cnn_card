#include "led_status.h"
#include "config.h"
#include <stdint.h>


/* 3-bit brightness values for each led*/
uint8_t led_status[LED_COUNT * PWM_BITS / 8 + 1] = {0};


/* gamma correction table
 * led_value ∈ [0,1)= (perceived brightness ∈ [0,1))^2.8 */
const uint8_t gamma_table[] = {
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,
  1,1,1,1,2,2,2,2,2,2,2,3,3,3,3,
  3,4,4,4,4,5,5,5,5,6,6,6,7,7,7,
  8,8,8,9,9,9,10,10,11,11,12,12,13,13,14,
  14,15,15,16,16,17,17,18,19,19,20,21,21,22,23,
  23,24,25,26,27,27,28,29,30,31,32,32,33,34,35,
  36,37,38,39,40,41,42,43,44,46,47,48,49,50,51,
  53,54,55,56,58,59,60,62,63 };


uint8_t value_to_pwm_level(float value, float max_value) {
    /* convert float value in [0, max_value] to pwm level */
    return gamma_table[(uint8_t) (value / max_value * 128)];
}


uint8_t get_led_brightness(uint8_t row, uint8_t col) {
    uint16_t start_idx = (row * 9 + col) * PWM_BITS;
    uint8_t out = 0;
    
    for (uint8_t i=0; i < PWM_BITS; i++) {
        out += (!!(led_status[(start_idx + i) / 8] & (1 << ((start_idx + i) % 8)))) << i;
    }
    
    return out;
}


void set_led_brightness(uint8_t row, uint8_t col, uint8_t val) {
    uint16_t start_idx = (row * 9 + col) * PWM_BITS;
    for (uint8_t i = 0; i < PWM_BITS; i++){
        if (val & (1 << i)) {
            led_status[(start_idx + i) / 8] |= (1 << ((start_idx + i) % 8));
        } else {
            led_status[(start_idx + i) / 8] &= ~(1 << ((start_idx + i) % 8));
        }
    }
}