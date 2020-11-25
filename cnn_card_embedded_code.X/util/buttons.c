#include "buttons.h"
#include "sleep.h"
#include "../config.h"

#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>

int8_t read_button_matrix() {
    int8_t pressed = -1;
    for (uint8_t row_index=0; row_index < 5; row_index++) {
        
        VPORTA.DIR = (1 << (row_index + 1));
        
        // not sure if .OUT gets reset on .DIR change
        // if not could skip this and just set all low
        VPORTA.OUT = ~(1 << (row_index + 1));
        
        /* TODO: check how short I can make this 
         * (need some delay or first col is not read correctly) */
        _delay_us(10);
        
        // this takes a variable amount of time, not sure if good idea
        if (~VPORTA.IN & PIN6_bm) {
            pressed = row_index * 5 + 0;
        } else if (~VPORTA.IN & PIN7_bm) {
            pressed = row_index * 5 + 1;
        } else if (~VPORTC.IN & PIN0_bm) {
            pressed = row_index * 5 + 2;
        } else if (~VPORTC.IN & PIN1_bm) {
            pressed = row_index * 5 + 3;
        } else if (~VPORTC.IN & PIN3_bm) { // yes, PIN3 is correct...
            pressed = row_index * 5 + 4;
        }
        
        if (pressed != -1) {
            /* we stop at the first button */
            break;
        }
    }
    return pressed;
}





int8_t read_buttons() {
    static int8_t last_pressed = -1;
    static uint8_t debounce_counter = 0;
    
    int8_t pressed = read_button_matrix();
    
    if (pressed == -1) {
        // test power & filter buttons (separate GPIOs)
        if (~VPORTB.IN & PIN7_bm) {
            pressed = FILTER_BUTTON;
        } else if (~VPORTB.IN & PIN3_bm) {
            pressed = PWR_BUTTON;
        }
    }
    
    if (pressed != -1) {
        /* found a button press */
        
        /* reset auto shutdown counter */
        reset_rtc_cnt();
        
        if (pressed == last_pressed) {
            if (debounce_counter != HELD_FLAG){
                debounce_counter++;
            }
        } else {
            debounce_counter = 0;
            last_pressed = pressed;
        }
    } else {
        // no button press detected
        debounce_counter = 0;
        last_pressed = -1;
    }
    
    if (debounce_counter == DEBOUNCE_THRESHOLD) {
        debounce_counter = HELD_FLAG;
        return pressed;
    }
    
    return -1;
}