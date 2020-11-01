#include "gol.h"
#include "led_control.h"


bool check_glider() {
    bool match = true;
    bool led_on;
    for (uint8_t row=0; row < 5; row++) {
        for (uint8_t col=0; col < 5; col++) {
            led_on = get_led_on(row, col);
            if ((row == 1 && col == 2) ||
                (row == 2 && col == 1) ||
                (row == 3 && col > 0 && col < 4)) {
                if (!led_on) {
                    match = false;
                }
            } else {
                if (led_on) {
                    match = false;
                }
            }
        }
    }
    return match;
}