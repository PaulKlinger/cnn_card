#include "sleep.h"
#include "config.h"

#include <util/delay.h>
#include <avr/io.h>
#include <avr/sleep.h>


void reset_rtc_cnt() {
    /* reset RTC (auto shutdown counter) */
    
    while (RTC.STATUS & RTC_CNTBUSY_bm) {
        /* wait for registers to be ready for writing*/
    }
    RTC.CNT = 0;
}


void go_to_sleep() {
    /* Go to sleep, wakeup on PWR button press
     * unfortunately synchronous pins like PB3 can only wake the chip
     * with BOTHEDGES or LEVEL interrupts, so we have to wait until the
     * button is released (and no longer bouncing) before going to sleep.
     */
    while (~PORTB.IN & PIN3_bm) {}
    _delay_ms(50);
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sleep_cpu();
    turn_off_leds();
    // turn off individual leds
    VPORTB.OUT |= (PIN4_bm | PIN5_bm | PIN6_bm);
    while (~PORTB.IN & PIN3_bm) {}
    _delay_ms(50);
    reset_rtc_cnt();
    
    run_startup_animation();
    reset_input_state();
}
