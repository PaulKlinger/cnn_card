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


volatile uint8_t test_led_brightness = 7;


volatile  uint8_t led_row =0;
volatile  uint8_t led_pwm_cycle_counter =0;

ISR(PORTB_PORT_vect) {
    test_led_brightness = (test_led_brightness + 1) % 8;
    /* clear interrupt flag */
    VPORTB.INTFLAGS |= PORT_INT3_bm;
}



ISR(RTC_PIT_vect) 
{   
    VPORTB.OUT |= PIN0_bm;
    VPORTB.OUT &= ~PIN1_bm;
    
    led_pwm_cycle_counter++;
    if (led_pwm_cycle_counter == 8) {
        led_pwm_cycle_counter = 0;
        led_row = (led_row + 1) % 10;
    }
    
    if (led_row == 0 && led_pwm_cycle_counter <= test_led_brightness) {
        VPORTB.OUT &= ~PIN0_bm;
    }
    /* TRIGB interrupt flag has to be cleared manually */
    RTC.PITINTFLAGS = RTC_PI_bm;
}

void shift_out(char data) {
    /* latch (STCP) low */
    
    VPORTB.OUT &= ~(PIN1_bm | PIN2_bm | PIN3_bm);
    
    for (int i=0;i < 8; i++) {
        if (data & (1 << i)) {
            VPORTB.OUT |= PIN2_bm;
        } else {
            VPORTB.OUT &= ~PIN2_bm;
        }
        VPORTB.OUT |= PIN1_bm;
        VPORTB.OUT &= ~PIN1_bm;
    }
    
    VPORTB.OUT |= PIN3_bm;
}

void rtc0_init(){
    while (RTC.STATUS > 0) { /* Wait for all register to be synchronized */}
    RTC.CLKSEL = RTC_CLKSEL_INT32K_gc; /* Clock Select: Internal 32kHz OSC */
    while (RTC.PITSTATUS > 0) { /* Wait for all register to be synchronized */}
    RTC.PITCTRLA = RTC_PERIOD_CYC4_gc /* Period: RTC Clock Cycles 256 */
    				| 1 << RTC_PITEN_bp; /* Enable: enabled */
    RTC.PITINTCTRL = 1 << RTC_PI_bp; /* Periodic Interrupt: enabled */
    RTC.CTRLA = RTC_PRESCALER_DIV1_gc /* Prescaling Factor: RTC Clock / 1 */
    				| 1 << RTC_RTCEN_bp /* Enable: enabled */
    				| 0 << RTC_RUNSTDBY_bp; /* Run In Standby: disabled */
}


void usart_spi_init(){
    USART0.CTRLA = 0;
    USART0.CTRLB |= USART_TXEN_bm; /* enable TX (but not RX) */
    
    /* change mode to master SPI, LSb first */
    USART0.CTRLC = USART_CMODE_MSPI_gc | USART_UDORD_bm; 
    
    /* set baud factor to 1 -> f_baud = 2MHz for CLK_PER = 4MHz*/
    USART0.BAUDL = (1 << 6);
}

void usart_shift_out(char data) {
    VPORTB.OUT &= ~PIN3_bm;
    
    while (!(USART0.STATUS & USART_DREIF_bm)){}
    USART0.TXDATAL = data;
    
    VPORTB.OUT |= PIN3_bm;
}


int main(void) {
      /* Configure clock prescaler for 4MHz  */
    _PROTECTED_WRITE(
            CLKCTRL.MCLKCTRLB,
            CLKCTRL_PDIV_4X_gc /* Prescaler division: 4X */
            | CLKCTRL_PEN_bm /* Prescaler enable: enabled */
            );
    
    VPORTB.DIR = 0;
    
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
    
    
    
    
    //PORTB.PIN3CTRL |= PORT_PULLUPEN_bm;
    /* Enable interrupt falling */
    //PORTB.PIN3CTRL = (PORTB.PIN3CTRL & ~PORT_ISC_gm) | PORT_ISC_FALLING_gc;
    
    //VPORTA.DIR = PIN1_bm | PIN2_bm | PIN3_bm;
    //VPORTA.OUT = 0;
    
    //VPORTB.DIR |= PIN0_bm | PIN1_bm;
    //VPORTB.OUT |= (PIN0_bm | PIN1_bm);
    
    //rtc0_init();
    
    led_row = 0;
    led_pwm_cycle_counter = 0;
    
    //sei();
    //while (1) {
    
    //}
    
    usart_spi_init();
    
    VPORTB.DIR |= PIN1_bm | PIN2_bm | PIN3_bm ;
    while (1) {
        for (int i=0; i < 8; i++) {
            usart_shift_out(~(1 << i));
            _delay_us(1000);
        }
    }
    
    /*
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    
    sei();
    
    while (1) {
        for (int j=0; j < 3; j++){
            for (int i=0; i < 8; i++) {
                shift_out(~(1 << i));
                _delay_ms(100);
            }
        }
        shift_out(255);
        sleep_cpu();
    }

    */
}
