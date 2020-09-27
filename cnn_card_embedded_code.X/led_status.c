#include "led_status.h"
#include "config.h"
#include <stdint.h>


/* 3-bit brightness values for each led*/
uint8_t led_status[LED_COUNT * PWM_BITS / 8 + 1] = {0};


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