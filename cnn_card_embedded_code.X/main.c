#define F_CPU 4000000

#include <avr/io.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>

#include "tensor.h"
#include "model_weights.h"


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
#define N_FILTERS 16


// flag values
#define HELD_FLAG 100
#define FILTER_BUTTON 25
#define PWR_BUTTON 26

/* 3-bit brightness values for each led*/
uint8_t led_status[LED_COUNT * PWM_BITS / 8 + 1] = {0};

/* precomputed bytes to shift out during main loop*/
uint8_t row_shift_bytes[(1 << PWM_BITS) * 9];
uint8_t col_shift_bytes[9];

/* brightness of row8 leds (not controlled via shift register) */
uint8_t row8_brightness[9];

uint8_t filter_idx = 0;


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
            if (row == 8) {
                row8_brightness[col] = brightness;
            } else {
                for (uint8_t pwm_index=0; pwm_index < (1 << PWM_BITS); pwm_index++) {
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


ISR(PORTB_PORT_vect) {
    /* This is only used to wake the MCU, so we do nothing here. */
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
    PORTB.PIN0CTRL |= PORT_INVEN_bm;
    USART0.CTRLA = USART_RS4850_bm;
    USART0.CTRLB |= USART_TXEN_bm; /* enable TX (but not RX) */
    
    /* change mode to master SPI */
    USART0.CTRLC = USART_CMODE_MSPI_gc ; 
    
    /* set baud factor to 1 -> f_baud = 2MHz for CLK_PER = 4MHz*/
    USART0.BAUDL = (1 << 6);
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
    
    if (filter_idx & (1 << 3)) {set_led_brightness(8, 0, (1 << PWM_BITS) - 1);}
    else {set_led_brightness(8, 0, 0);}
    
    update_pwm_pattern();
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
            set_filter_leds();
        } else if (last_pressed == PWR_BUTTON) {
            go_to_sleep();
        } else {
            uint8_t row = last_pressed / 5;
            uint8_t col = last_pressed % 5;

            set_led_brightness(row, col,
                (get_led_brightness(row, col) == 0) ? ((1 << PWM_BITS) - 1) : 0
            );
            update_pwm_pattern();
        }
    }
}


void main_loop() {
    
    for (uint8_t row=0; row < 9; row++) {
        for (uint8_t col=0; col < 9; col++) {
            set_led_brightness(row, col, (1 << PWM_BITS) - 1);
        }
    }
    
    update_pwm_pattern();
    init_col_shift_bytes();
    
    while (1) {
        uint8_t set_vportc = VPORTC.OUT;
        uint8_t next_vportc = VPORTC.OUT;
        for (uint8_t pwm_idx=0; pwm_idx < (1 << PWM_BITS); pwm_idx++) {
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
        if (RTC.CNT > AUTO_SHUTDOWN_TIME_s) {
            turn_off_leds();
            go_to_sleep();
        }
        
        read_buttons();
        
        // turn on single leds
        VPORTB.DIR |= (PIN4_bm | PIN5_bm | PIN6_bm);
    }
}

const float test_input_data[] = {
       1, 1, 0, 0, 0,
       0, 0, 0, 0, 0,
       0, 0, 1, 0, 0,
       0, 0, 1, 1, 0,
       0, 0, 1, 0, 1
    };
const uint8_t test_input_shape[] = {5, 5, 1};
const struct float_4tensor test_input = {
    .data=(float*) test_input_data,
    .s0=5,
    .s1=5,
    .s2=1,
    .s3=1
};


void test_tensor_fns() {
    volatile float ker0102 = t4_get_value(&conv0_kernel, 0, 1, 0, 2);
    float res_data[4 * 4 * 4] = {0};
    struct float_4tensor res = {
        .data=(float*) res_data,
        .s0=4, // rows
        .s1=4, // cols
        .s2=4, // channels
        .s3=1
    };
    
    t4_convolve_2x2(&test_input, &conv0_kernel, &res);
    
    t4_add_conv_bias(&res, &conv0_bias);
    
    t4_relu(&res);
    
    volatile float res000 = t4_get_value(&res, 0, 0, 0, 0);
    volatile float res213 = t4_get_value(&res, 2, 1, 3, 0);
    
    
    
    float test_counter = 0;
    while (1){
        test_counter += 0.3;
        _delay_ms(10);
    }
}


int main(void) {
      /* Configure clock prescaler for 4MHz  */
    _PROTECTED_WRITE(
            CLKCTRL.MCLKCTRLB,
            CLKCTRL_PDIV_4X_gc /* Prescaler division: 4X */
            | CLKCTRL_PEN_bm /* Prescaler enable: enabled */
            );
    
    test_tensor_fns();
    
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
