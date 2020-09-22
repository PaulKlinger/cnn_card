/*
 * File:   main.c
 * Author: kling
 *
 * Created on 30 May 2020, 14:04
 */

#define F_CPU 4000000

#include <avr/io.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>


FUSES = 
{
	.APPEND = 0,
	.BODCFG = ACTIVE_DIS_gc | LVL_BODLEVEL0_gc | SAMPFREQ_1KHZ_gc | SLEEP_DIS_gc,
	.BOOTEND = 0,
	.OSCCFG = FREQSEL_16MHZ_gc,
	.SYSCFG0 = CRCSRC_NOCRC_gc | RSTPINCFG_UPDI_gc,
	.SYSCFG1 = SUT_64MS_gc,
	.WDTCFG = PERIOD_OFF_gc | WINDOW_OFF_gc,
};


// config options
#define DEBOUNCE_THRESHOLD 5
#define AUTO_SHUTDOWN_TIME_s 60 * 2 // 2 min

#define LED_COUNT 5 * 5 + 4 * 4 + 3 * 3 + 2 * 2 + 16 + 10 + 4
#define PWM_BITS 5


// flag values
#define HELD_FLAG 100
#define FILTER_BUTTON 25
#define PWR_BUTTON 26

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


uint8_t row_shift_bytes[(1 << PWM_BITS) * 9];
uint8_t col_shift_bytes[9];

uint8_t row8_setting[9];

void init_col_shift_bytes() {
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
            
            for (uint8_t pwm_index=0; pwm_index < (1 << PWM_BITS); pwm_index++) {
                if (pwm_index < brightness) {
                    if (row == 8){
                        row8_setting[col] |= (1 << pwm_index);
                    } else {
                        row_shift_bytes[9 * pwm_index + col] &= ~(1 << (row % 8));
                    }
                } else {
                    if (row == 8) {
                        row8_setting[col] &= ~(1 << pwm_index);
                    } else {
                        row_shift_bytes[9 * pwm_index + col] |= (1 << (row % 8));
                    }
                }
            }
        }
    }
}


ISR(PORTB_PORT_vect) {
    /* clear interrupt flag */
    VPORTB.INTFLAGS |= PORT_INT3_bm;
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
}


void usart_spi_init(){
    USART0.CTRLA = 0;
    USART0.CTRLB |= USART_TXEN_bm; /* enable TX (but not RX) */
    
    /* change mode to master SPI */
    USART0.CTRLC = USART_CMODE_MSPI_gc ; 
    
    /* set baud factor to 1 -> f_baud = 2MHz for CLK_PER = 4MHz*/
    USART0.BAUDL = (1 << 6);
}

void usart_shift_out(char data) {
    while (!(USART0.STATUS & USART_DREIF_bm)){}
    USART0.TXDATAL = data;
    _delay_us(5); // wait until data has been shifted out, should do something useful here!
    VPORTB.OUT |= PIN0_bm;
    VPORTB.OUT &= ~PIN0_bm;
}

void usart_shift_out_2(char data1, char data2) {
    while (!(USART0.STATUS & USART_DREIF_bm)){}
    USART0.TXDATAL = data1;
    while (!(USART0.STATUS & USART_DREIF_bm)){}
    USART0.TXDATAL = data2;
    _delay_us(15); // wait until data has been shifted out, should do something useful here!
    VPORTB.OUT |= PIN0_bm;
    VPORTB.OUT &= ~PIN0_bm;
}

void turn_off_leds() {
    usart_shift_out_2(0, 255);
    VPORTC.OUT |= PIN5_bm;
    VPORTC.OUT &= ~PIN4_bm;
    VPORTB.OUT |= (PIN4_bm | PIN5_bm | PIN6_bm);
}

void reset_rtc_cnt() {
    // reset RTC (auto shutdown counter)
    while (RTC.STATUS & RTC_CNTBUSY_bm) {}
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


void read_buttons(){
    static int8_t last_pressed = -1;
    static uint8_t debounce_counter = 0;
    
    int8_t pressed = -1;
    for (uint8_t row_index=0; row_index < 5; row_index++) {
        
        VPORTA.DIR = (1 << (row_index + 1));
        
        // not sure if .OUT gets reset on .DIR change
        // if not could skip this and just set all low
        VPORTA.OUT = ~(1 << (row_index + 1));
        
        // TODO: check how short I can make this
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
            break;
        }
    }
    if (pressed == -1) {
        // test power & filter buttons
        if (~VPORTB.IN & PIN7_bm) {
            pressed = FILTER_BUTTON;
        } else if (~VPORTB.IN & PIN3_bm) {
            pressed = PWR_BUTTON;
        }
    }
    
    if (pressed != -1) {
        // found a button press
        
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
            // -- 
        } else if (last_pressed == PWR_BUTTON) {
            go_to_sleep();
        } else {
            uint8_t row = last_pressed / 5;
            uint8_t col = last_pressed % 5;

            set_led_brightness(
                    row, col,
                    (get_led_brightness(row, col) + 1) % (1 << PWM_BITS)
            );
            update_pwm_pattern();
        }
    }
}


void test_leds_and_buttons() {
    uint8_t set_vportc = VPORTC.OUT;
    uint8_t next_vportc = VPORTC.OUT;
    
    for (uint8_t row=0; row < 9; row++) {
        for (uint8_t col=0; col < 9; col++) {
            set_led_brightness(row, col, 1);
        }
    }
    
    update_pwm_pattern();
    init_col_shift_bytes();
    
    while (1) {
        for (uint8_t pwm_idx=0; pwm_idx < (1 << PWM_BITS); pwm_idx++) {
            for (uint8_t col_idx=0; col_idx <= 8; col_idx++){
                /* set column */
                if (col_idx == 8) {
                    next_vportc = VPORTC.OUT | PIN4_bm;
                } else {
                    next_vportc = VPORTC.OUT & ~PIN4_bm;
                }

               /* set row8 */
                if (row8_setting[col_idx] & (1 << pwm_idx)) {
                    next_vportc &= ~PIN5_bm;
                } else {
                    next_vportc |= PIN5_bm;
                }
                
                // pulse SCK, setting pattern from last iteration
                // this gives time for data to be shifted out while we do the
                // calcs above
                
                // if we are still too early wait until everything is shifted out
                // could do this in an ISR, but not sure if register store/load
                // would be slower than this wait
                while (~USART0.STATUS & USART_TXCIF_bm) {}
                // need to clear transmit complete flag manually
                USART0.STATUS |= USART_TXCIF_bm;
                VPORTB.OUT |= PIN0_bm;
                VPORTB.OUT &= ~PIN0_bm;
                VPORTC.OUT = set_vportc;
                
                USART0.TXDATAL = col_shift_bytes[col_idx];
                while (!(USART0.STATUS & USART_DREIF_bm)){}
                USART0.TXDATAL = row_shift_bytes[9 * pwm_idx + col_idx];
                
                set_vportc = next_vportc;
            }
            /* TODO: col 8 is brighter than others 
             * because of additional time to go to next pwm loop iteration.
             * Could just switch to combined loop?? Or handle in software?
             */
        }
        // pulse SCK & set portc for last iteration  
        _delay_us(8);
        VPORTC.OUT = set_vportc;
        VPORTB.OUT |= PIN0_bm;
        VPORTB.OUT &= ~PIN0_bm;
        
        
        // turn off leds (otherwise last col is brighter
        // because of button scan time)
        turn_off_leds();
        if (RTC.CNT > AUTO_SHUTDOWN_TIME_s) {
            turn_off_leds();
            go_to_sleep();
        }
        
        read_buttons();
        
        // turn on single leds
        VPORTB.OUT &= ~(PIN4_bm | PIN5_bm | PIN6_bm);
    }
}


int main(void) {
      /* Configure clock prescaler for 4MHz  */
    _PROTECTED_WRITE(
            CLKCTRL.MCLKCTRLB,
            CLKCTRL_PDIV_4X_gc /* Prescaler division: 4X */
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
    
    /* turn on single leds */
    VPORTB.OUT &= ~(PIN4_bm | PIN5_bm | PIN6_bm);
    
    test_leds_and_buttons();
    
}
