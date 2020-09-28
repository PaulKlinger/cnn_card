/* 
 * File:   config.h
 * Author: kling
 *
 * Created on 27 September 2020, 17:59
 */

#ifndef CONFIG_H
#define	CONFIG_H

#ifdef	__cplusplus
extern "C" {
#endif

// config options
#define DEBOUNCE_THRESHOLD 5
#define AUTO_SHUTDOWN_TIME_s 60 * 2 // 2 min

#define LED_COUNT 5 * 5 + 4 * 4 + 3 * 3 + 2 * 2 + 16 + 10 + 4
#define PWM_BITS 6 /* NEED TO CHANGE gamma_table in led_status.c if this changes! */
#define N_FILTERS 16


// flag values
#define HELD_FLAG 100
#define FILTER_BUTTON 25
#define PWR_BUTTON 26
#define PWM_LEVELS (1 << PWM_BITS)
#define MAX_PWM_LEVEL (PWM_LEVELS - 1)


#ifdef	__cplusplus
}
#endif

#endif	/* CONFIG_H */

