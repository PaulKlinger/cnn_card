#include "led_control.h"
#include "../config.h"

#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>


/* PWM_BITS-bit brightness values for each led*/
uint8_t led_status[LED_STATUS_BYTES] = {0};


/* precomputed bytes to shift out during main loop*/
uint8_t row_shift_bytes[PWM_LEVELS * 9];
uint8_t col_shift_bytes[9];

/* brightness of row8 leds (not controlled via shift register) */
uint8_t row8_brightness[9];


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


void clear_led_brightness() {
    for (uint16_t i=0; i < sizeof(led_status); i++) {
        led_status[i] = 0;
    }
}


uint8_t get_led_brightness(uint8_t row, uint8_t col) {
    uint16_t start_idx = (row * 9 + col) * PWM_BITS;
    uint8_t out = 0;
    
    for (uint8_t i=0; i < PWM_BITS; i++) {
        out += (!!(led_status[(start_idx + i) / 8] & (1 << ((start_idx + i) % 8)))) << i;
    }
    
    return out;
}

bool get_led_on(uint8_t row, uint8_t col) {
    uint16_t start_idx = (row * 9 + col) * PWM_BITS;
    return led_status[(start_idx) / 8] & (1 << (start_idx % 8));
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


void set_filter_leds(uint8_t filter_idx) {
    if (filter_idx & 1) {PORTB.OUT &= ~PIN6_bm;}
    else {PORTB.OUT |= PIN6_bm;}
    
    if (filter_idx & (1 << 1)) {PORTB.OUT &= ~PIN4_bm;}
    else {PORTB.OUT |= PIN4_bm;}
    
    if (filter_idx & (1 << 2)) {PORTB.OUT &= ~PIN5_bm;}
    else {PORTB.OUT |= PIN5_bm;}
    
    if (filter_idx & (1 << 3)) {set_led_brightness(8, 0, MAX_PWM_LEVEL);}
    else {set_led_brightness(8, 0, 0);}
}


void init_pwm_data() {
    // set column select shift pattern for first 8 cols
    // don't compute this dynamically to make pwm cycles more even
    for (uint8_t col=0; col < 8; col++) {
        col_shift_bytes[col] = (1 << col);
    }
    col_shift_bytes[8] = 0;
}


void update_pwm_pattern() {
    for (uint8_t row=0; row <= 8; row++) {
        for (uint8_t col=0; col <= 8; col++) {
            uint8_t brightness = get_led_brightness(row, col);
            if (row == 8) {
                row8_brightness[col] = brightness;
            } else {
                for (uint8_t pwm_index=0; pwm_index < PWM_LEVELS; pwm_index++) {
                    if (pwm_index < brightness) {
                        row_shift_bytes[9 * pwm_index + col] &= ~(1 << (row % 8));
                    } else {
                        row_shift_bytes[9 * pwm_index + col] |= (1 << (row % 8));
                    }
                }
            }
        }
    }
}


void turn_off_leds() {
    /* shift out pattern for all leds off*/
    /* wait for tx buffer ready*/
    while (!(USART0.STATUS & USART_DREIF_bm)){}
    USART0.TXDATAL = 0;
    while (!(USART0.STATUS & USART_DREIF_bm)){}
    USART0.TXDATAL = 255;
    
    /* turn off GPIO controlled row/col 8 */
    VPORTC.OUT |= PIN5_bm;
    VPORTC.OUT &= ~PIN4_bm;
    
    /* turn off individual leds 
     * (change to input to remember value) */
    VPORTB.DIR &= ~(PIN4_bm | PIN5_bm | PIN6_bm);
}


void run_pwm_cycle() {
    /* Run a full PWM cycle: PWM_LEVELS * 9 columns different output states */
    
    // turn on single leds
    VPORTB.DIR |= (PIN4_bm | PIN5_bm | PIN6_bm);
    uint8_t set_vportc = VPORTC.OUT;
    uint8_t next_vportc = VPORTC.OUT;
    for (uint8_t pwm_idx=0; pwm_idx < PWM_LEVELS; pwm_idx++) {
        for (uint8_t col=0; col <= 8; col++){
            /* set column */
            if (col == 8) {
                next_vportc = VPORTC.OUT | PIN4_bm;
            } else {
                next_vportc = VPORTC.OUT & ~PIN4_bm;
            }

           /* set row8 */
            if (pwm_idx < row8_brightness[col]) {
                next_vportc &= ~PIN5_bm;
            } else {
                next_vportc |= PIN5_bm;
            }

            /* shift register output disable */
            VPORTC.OUT |= PIN2_bm;
            /* turn off row8, this ensures that led (8,8) isn't brighter
             * (it would be on during the wait loop below) */
            VPORTC.OUT |= PIN5_bm;

            /* wait for shift out to complete 
             * (RCK will be pulsed automatically) */
            /* TODO: replacing this with _delay_us(1) improves speed
             * significantly, but then col 7 is active longer */
            while (!(USART0.STATUS & USART_TXCIF_bm)) {}

            // need to clear transmit complete flag manually
            USART0.STATUS |= USART_TXCIF_bm;

            /* set col8 / row8 and enable shift register output */
            VPORTC.OUT = set_vportc;


            USART0.TXDATAL = col_shift_bytes[col];
            while (!(USART0.STATUS & USART_DREIF_bm)){}
            USART0.TXDATAL = row_shift_bytes[9 * pwm_idx + col];

            set_vportc = next_vportc;
        }
    }
    // pulse SCK & set portc for last iteration  
    _delay_us(4);
    VPORTC.OUT = set_vportc;
    _delay_us(8);
    // turn off leds (otherwise last col is brighter
    // because of button scan time)
    turn_off_leds();
}