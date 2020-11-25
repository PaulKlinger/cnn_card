

#include "config.h"

#include <avr/io.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>

#include "model/model.h"
#include "util/led_control.h"
#include "util/buttons.h"
#include "util/sleep.h"
#include "fun/startup_anim.h"
#include "fun/gol.h"


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
    /* interrupt triggered pwm, only used when the main loop is busy */
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


void run_model_with_pwm() {
    // enable interrupt pwm during model calculations
    RTC.PITCTRLA |= RTC_PITEN_bm;

    run_model_and_set_led_brightness(filter_idx);

    // disable interrupt pwm again
    RTC.PITCTRLA &= ~RTC_PITEN_bm;
}


void reset_input_state() {
    filter_idx = 0;
    clear_led_brightness();
    set_led_brightness(1, 1, MAX_PWM_LEVEL);
    set_led_brightness(1, 2, MAX_PWM_LEVEL);
    set_led_brightness(2, 2, MAX_PWM_LEVEL);
    set_led_brightness(3, 1, MAX_PWM_LEVEL);
    set_led_brightness(3, 2, MAX_PWM_LEVEL);
    set_led_brightness(3, 3, MAX_PWM_LEVEL);
    
    set_filter_leds(filter_idx);
    update_pwm_pattern();
    
    run_model_with_pwm();
    update_pwm_pattern();
}


void sleep_and_reset() {
    turn_off_leds();
    go_to_sleep();
    run_startup_animation();
    reset_input_state();
}


void handle_buttons(){
    int8_t pressed = read_buttons();
    if (pressed == -1) {
        return;
    } else if (pressed == FILTER_BUTTON) {
        if (check_glider()) {
            run_gol();
        } else {
            filter_idx = (filter_idx + 1) % N_FILTERS;
        }
        run_model_with_pwm();
        set_filter_leds(filter_idx);
        update_pwm_pattern();
    } else if (pressed == PWR_BUTTON) {
        sleep_and_reset();
    } else {
        uint8_t row = pressed / 5;
        uint8_t col = pressed % 5;

        set_led_brightness(row, col,
            get_led_on(row, col) ? 0 : MAX_PWM_LEVEL
        );

        run_model_with_pwm();
        update_pwm_pattern();
    }
}


void main_loop() {
    
    while (1) {
        run_pwm_cycle();
        handle_buttons();
        if (RTC.CNT > AUTO_SHUTDOWN_TIME_s) {
            sleep_and_reset();
        }
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
    
    usart_spi_init();
    
    // shift register output disable
    VPORTC.OUT |= PIN2_bm;
    // GPIO led row/col off
    VPORTC.OUT &= ~PIN4_bm;
    VPORTC.OUT |= PIN5_bm;
    
    // turn off individual leds
    VPORTB.OUT |= (PIN4_bm | PIN5_bm | PIN6_bm);
    
    VPORTB.DIR = (PIN0_bm | PIN1_bm | PIN2_bm | PIN4_bm | PIN5_bm | PIN6_bm);
    VPORTC.DIR |= (PIN2_bm | PIN4_bm | PIN5_bm);
    

    
    
    turn_off_leds();
    
    /* shift register output enable */
    VPORTC.OUT &= ~PIN2_bm;
    
    init_pwm_data();
    
    // sleep
    sleep_and_reset();
    
    main_loop();
    
}
