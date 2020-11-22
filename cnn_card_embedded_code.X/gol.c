#include "gol.h"
#include "led_control.h"
#include "config.h"
#include "buttons.h"
#include "sleep.h"

#include <util/delay.h>
#include <stdbool.h>
#include <avr/io.h>


bool check_glider() {
    bool led_on;
    for (uint8_t row=0; row < 5; row++) {
        for (uint8_t col=0; col < 5; col++) {
            led_on = get_led_on(row, col);
            if ((row == 1 && col == 2) ||
                (row == 2 && col == 1) ||
                (row == 3 && col > 0 && col < 4)) {
                if (!led_on) {
                    return false;
                }
            } else {
                if (led_on) {
                    return false;
                }
            }
        }
    }
    return true;
}

static inline int8_t modulo(int8_t a, int8_t b){
    // correctly handle negative values
    return (a%b+b)%b;
}


void sync_board(bool *board) {
    for (uint8_t row=0; row < 5; row++) {
        for (uint8_t col=0; col < 5; col++) {
            board[row * 5 + col] = get_led_on(row, col);
        }
    }
}


void do_gol_step() {
    bool old_board[5 * 5] = {0};
    sync_board(old_board);
    uint8_t neighbors;
    for (int8_t row=0; row < 5; row++) {
        for (int8_t col=0; col < 5; col++) {
            neighbors = 0;
            for (int8_t dx=-1; dx<=1; dx++) {
                for (int8_t dy=-1; dy<=1; dy++) {
                    if (!(dy==0 && dx==0)
                        && old_board[
                            modulo(row+dy, 5) * 5 + modulo(col+dx, 5)
                            ]
                        ){
                        neighbors++;
                    }
                }
            }
            if (neighbors > 3 || neighbors < 2) {
                set_led_brightness(row, col, 0);
            } else if (neighbors == 3) {
                set_led_brightness(row, col, MAX_PWM_LEVEL);
            }
        }
    }
}

void clear_non_input_leds() {
    // turn off matrix leds outside input 5x5 grid
    for (uint8_t row=0; row <= 8; row++) {
        for (uint8_t col=0; col <= 8; col++) {
            if (row >= 5 || col >= 5) {
                set_led_brightness(row, col, 0);
            }
        }
    }
    // turn off individual leds
    VPORTB.OUT |= (PIN4_bm | PIN5_bm | PIN6_bm);
}


void run_gol() {
    clear_non_input_leds();
    
    // turn on center led of step indicator (- | - |...)
    set_led_brightness(5, 5, MAX_PWM_LEVEL);
    bool running = true;
    
    bool odd_step = true;
    
    while (1) {
        if (RTC.CNT > AUTO_SHUTDOWN_TIME_s) {
            turn_off_leds();
            go_to_sleep();
            return;
        }
        if (running) {
            do_gol_step();
            
            // update step indicator
            set_led_brightness(5, 4, odd_step ? 0 : MAX_PWM_LEVEL);
            set_led_brightness(5, 6, odd_step ? 0 : MAX_PWM_LEVEL);
            set_led_brightness(5, 2, odd_step ? MAX_PWM_LEVEL : 0);
            set_led_brightness(0, 8, odd_step ? MAX_PWM_LEVEL : 0);
            
            update_pwm_pattern();
            
            odd_step = !odd_step;
        }
        
        for (uint8_t i=0; i < 60; i++) {
            run_pwm_cycle();
            int8_t pressed = read_buttons();
            if (pressed != -1) {
                // reset auto shutdown timer
                reset_rtc_cnt();
            }
            
            if (pressed == PWR_BUTTON) {
                return;
            } else if (pressed == FILTER_BUTTON) {
                running = !running;
            } else if (pressed != -1){
                uint8_t row = pressed / 5;
                uint8_t col = pressed % 5;

                set_led_brightness(row, col,
                    get_led_on(row, col) ? 0 : MAX_PWM_LEVEL
                );
                update_pwm_pattern();
            }
        }
    }
}