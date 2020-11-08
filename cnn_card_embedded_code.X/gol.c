#include "gol.h"
#include "led_control.h"
#include "config.h"
#include "buttons.h"

#include <util/delay.h>
#include <stdbool.h>
#include <avr/io.h>


bool check_glider() {
    bool match = true;
    bool led_on;
    for (uint8_t row=0; row < 5; row++) {
        for (uint8_t col=0; col < 5; col++) {
            led_on = get_led_on(row, col);
            if ((row == 1 && col == 2) ||
                (row == 2 && col == 1) ||
                (row == 3 && col > 0 && col < 4)) {
                if (!led_on) {
                    match = false;
                }
            } else {
                if (led_on) {
                    match = false;
                }
            }
        }
    }
    return match;
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


void run_gol() {
    bool old_board[5 * 5] = {0};
    

    uint8_t neighbors;
    while (1) {
        sync_board(old_board);
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
        update_pwm_pattern();
        
        for (uint8_t i=0; i < 80; i++) {
            run_pwm_cycle();
            int8_t pressed = read_buttons();
            if (pressed == PWR_BUTTON) {
                return;
            }
        }
    }
}