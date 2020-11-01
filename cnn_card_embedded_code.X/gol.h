/* 
 * File:   gol.h
 * Author: kling
 *
 * Created on 01 November 2020, 19:09
 */

#ifndef GOL_H
#define	GOL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef	__cplusplus
extern "C" {
#endif

    bool check_glider();
    
    void run_gol();

#ifdef	__cplusplus
}
#endif

#endif	/* GOL_H */

