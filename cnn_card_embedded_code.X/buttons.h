/* 
 * File:   button_matrix.h
 * Author: kling
 *
 * Created on 08 November 2020, 20:32
 */

#ifndef BUTTON_MATRIX_H
#define	BUTTON_MATRIX_H

#include <stdint.h>

#ifdef	__cplusplus
extern "C" {
#endif
    
    // flag values
    #define HELD_FLAG 100
    #define FILTER_BUTTON 25
    #define PWR_BUTTON 26

    int8_t read_button_matrix();
    
    int8_t read_buttons();


#ifdef	__cplusplus
}
#endif

#endif	/* BUTTON_MATRIX_H */

