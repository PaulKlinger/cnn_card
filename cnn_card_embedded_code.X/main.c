

#include "config.h"

#include <avr/io.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>

#include "model.h"
#include "led_control.h"


FUSES = 
{
	.APPEND = 0,
	.BODCFG = ACTIVE_DIS_gc | LVL_BODLEVEL0_gc | SAMPFREQ_1KHZ_gc | SLEEP_DIS_gc,
	.BOOTEND = 0,
	.OSCCFG = FREQSEL_20MHZ_gc,
	.SYSCFG0 = CRCSRC_NOCRC_gc | RSTPINCFG_UPDI_gc,
	.SYSCFG1 = SUT_64MS_gc,
	.WDTCFG = PERIOD_OFF_gc | WINDOW_OFF_gc,
};


uint8_t filter_idx = 0;




ISR(PORTB_PORT_vect) {
    /* This is only used to wake the MCU, so we do nothing here. */
    /* clear interrupt flag */
    VPORTB.INTFLAGS |= PORT_INT3_bm;
}

ISR(RTC_PIT_vect)
{
    run_pwm_cycle();
    /* TRIGB interrupt flag has to be cleared manually */
    RTC.PITINTFLAGS = RTC_PI_bm;
}


void rtc0_init(){
    while (RTC.STATUS > 0) { /* Wait for all register to be synchronized */}
    RTC.CLKSEL = RTC_CLKSEL_INT32K_gc; /* Clock Select: Internal 32kHz OSC */
    
    /* 10 min period 
     * (should never reach this, as we reset on auto shutdown at <10 min)
     */
    RTC.PER = 60 * 10; 
    RTC.CTRLA = RTC_PRESCALER_DIV32768_gc/* Prescaling Factor: RTC Clock / 32768 */
                | 1 << RTC_RTCEN_bp /* Enable: enabled */
                | 0 << RTC_RUNSTDBY_bp; /* Run In Standby: disabled */
    
    while (RTC.PITSTATUS > 0) {}
    /* set PIT period (for PWM during calculations) but don't enable it
     * most of the time we'll just run PWM in main loop so it's faster.
     */
    RTC.PITINTCTRL = RTC_PI_bm;
    RTC.PITCTRLA = RTC_PERIOD_CYC512_gc; // = 64 Hz

}


void usart_spi_init(){
    PORTB.PIN0CTRL |= PORT_INVEN_bm;
    USART0.CTRLA = USART_RS4850_bm;
    USART0.CTRLB |= USART_TXEN_bm; /* enable TX (but not RX) */
    
    /* change mode to master SPI */
    USART0.CTRLC = USART_CMODE_MSPI_gc ; 
    
    /* set baud factor to 1 -> f_baud = 2MHz for CLK_PER = 4MHz*/
    USART0.BAUDL = (1 << 6);
}


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
    while (~PORTB.IN & PIN3_bm) {}
    _delay_ms(50);
    
    reset_rtc_cnt();
}


void set_filter_leds() {
    if (filter_idx & 1) {PORTB.OUT &= ~PIN6_bm;}
    else {PORTB.OUT |= PIN6_bm;}
    
    if (filter_idx & (1 << 1)) {PORTB.OUT &= ~PIN4_bm;}
    else {PORTB.OUT |= PIN4_bm;}
    
    if (filter_idx & (1 << 2)) {PORTB.OUT &= ~PIN5_bm;}
    else {PORTB.OUT |= PIN5_bm;}
    
    if (filter_idx & (1 << 3)) {set_led_brightness(8, 0, MAX_PWM_LEVEL);}
    else {set_led_brightness(8, 0, 0);}
    
    update_pwm_pattern();
}

void run_model_with_pwm() {
    // enable interrupt pwm during model calculations
    RTC.PITCTRLA |= RTC_PITEN_bm;

    run_model_and_set_led_brightness(filter_idx);

    // disable interrupt pwm again
    RTC.PITCTRLA &= ~RTC_PITEN_bm;
}


void read_buttons(){
    static int8_t last_pressed = -1;
    static uint8_t debounce_counter = 0;
    
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
        
        if (last_pressed == FILTER_BUTTON) {
            filter_idx = (filter_idx + 1) % N_FILTERS;
            run_model_with_pwm();
            set_filter_leds();
        } else if (last_pressed == PWR_BUTTON) {
            go_to_sleep();
        } else {
            uint8_t row = last_pressed / 5;
            uint8_t col = last_pressed % 5;
            
            set_led_brightness(row, col,
                (get_led_brightness(row, col) == 0) ? MAX_PWM_LEVEL : 0
            );
            
            run_model_with_pwm();
            update_pwm_pattern();
            
        }
    }
}


void main_loop() {
    
    for (uint8_t row=0; row < 9; row++) {
        for (uint8_t col=0; col < 9; col++) {
            set_led_brightness(row, col, 0);
        }
    }
    
    set_led_brightness(1, 1, MAX_PWM_LEVEL);
    set_led_brightness(1, 2, MAX_PWM_LEVEL);
    set_led_brightness(2, 2, MAX_PWM_LEVEL);
    set_led_brightness(3, 1, MAX_PWM_LEVEL);
    set_led_brightness(3, 2, MAX_PWM_LEVEL);
    set_led_brightness(3, 3, MAX_PWM_LEVEL);
    
    run_model_and_set_led_brightness(filter_idx);
    
    set_filter_leds();
    update_pwm_pattern();
    init_pwm_data();
    
    
    while (1) {
        run_pwm_cycle();
        if (RTC.CNT > AUTO_SHUTDOWN_TIME_s) {
            turn_off_leds();
            go_to_sleep();
        }
        
        read_buttons();
        
        // turn on single leds
        VPORTB.DIR |= (PIN4_bm | PIN5_bm | PIN6_bm);
    }
}


int main(void) {
      /* Configure clock prescaler for 4MHz  */
    _PROTECTED_WRITE(
            CLKCTRL.MCLKCTRLB,
            CLKCTRL_PDIV_2X_gc /* Prescaler division: 2X */
            | CLKCTRL_PEN_bm /* Prescaler enable: enabled */
            );
    
    /* Enable pullups for low power consumption (20uA -> 0.1uA (afair))*/
    PORTA.PIN0CTRL |= PORT_PULLUPEN_bm;
    PORTA.PIN1CTRL |= PORT_PULLUPEN_bm;
    PORTA.PIN2CTRL |= PORT_PULLUPEN_bm;
    PORTA.PIN3CTRL |= PORT_PULLUPEN_bm;
    PORTA.PIN4CTRL |= PORT_PULLUPEN_bm;
    PORTA.PIN5CTRL |= PORT_PULLUPEN_bm;
    PORTA.PIN6CTRL |= PORT_PULLUPEN_bm;
    PORTA.PIN7CTRL |= PORT_PULLUPEN_bm;
    
    PORTB.PIN0CTRL |= PORT_PULLUPEN_bm;
    PORTB.PIN1CTRL |= PORT_PULLUPEN_bm;
    PORTB.PIN2CTRL |= PORT_PULLUPEN_bm;
    PORTB.PIN3CTRL |= PORT_PULLUPEN_bm;
    PORTB.PIN4CTRL |= PORT_PULLUPEN_bm;
    PORTB.PIN5CTRL |= PORT_PULLUPEN_bm;
    PORTB.PIN6CTRL |= PORT_PULLUPEN_bm;
    PORTB.PIN7CTRL |= PORT_PULLUPEN_bm;
    
    
    PORTC.PIN0CTRL |= PORT_PULLUPEN_bm;
    PORTC.PIN1CTRL |= PORT_PULLUPEN_bm;
    PORTC.PIN2CTRL |= PORT_PULLUPEN_bm;
    PORTC.PIN3CTRL |= PORT_PULLUPEN_bm;
    PORTC.PIN4CTRL |= PORT_PULLUPEN_bm;
    PORTC.PIN5CTRL |= PORT_PULLUPEN_bm;
    PORTC.PIN6CTRL |= PORT_PULLUPEN_bm;
    PORTC.PIN7CTRL |= PORT_PULLUPEN_bm;
    
    
    
    
    /* Enable interrupt on power button
     * (this is only for wakeup)*/
    PORTB.PIN3CTRL = (PORTB.PIN3CTRL & ~PORT_ISC_gm) | PORT_ISC_BOTHEDGES_gc;
    sei();
    
    rtc0_init();
    
    //sei();
    //while (1) {
    
    //}
    
    usart_spi_init();
    
    // shift register output disable
    VPORTC.OUT |= PIN2_bm;
    // GPIO led row/col off
    VPORTC.OUT &= ~PIN4_bm;
    VPORTC.OUT |= PIN5_bm;
    
    VPORTB.DIR = (PIN0_bm | PIN1_bm | PIN2_bm | PIN4_bm | PIN5_bm | PIN6_bm);
    VPORTC.DIR |= (PIN2_bm | PIN4_bm | PIN5_bm);
    
    
    turn_off_leds();
    
    // sleep
    go_to_sleep();
    
    /* shift register output enable */
    VPORTC.OUT &= ~PIN2_bm;
    
    
    main_loop();
    
}
