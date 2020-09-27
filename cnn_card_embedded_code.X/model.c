#include "model.h"

#include "model_weights.h"
#include "tensor.h"
#include "config.h"
#include "led_status.h"

#include <math.h>

#include <avr/io.h>


void set_output_led_brightness (
    struct float_4tensor *output
) {
    // TODO: gamma correction
    set_led_brightness(8, 1, round(output->data[0] * MAX_PWM_LEVEL));
    for (uint8_t i=0; i < output->s0 - 1; i++) {
        set_led_brightness(7, i, round(output->data[i + 1] * MAX_PWM_LEVEL));
    }
}


float t_in_conv1_out_pool_out_data[3 * 3 * 8] = {0};
float conv02_out_data[4 * 4 * 4] = {0};
float t_out_data[10] = {0};

struct float_4tensor t_in = {
    .data=(float*) t_in_conv1_out_pool_out_data,
    .s0=5, // rows
    .s1=5, // cols
    .s2=1,
    .s3=1
};


struct float_4tensor conv0_out = {
    .data=(float*) conv02_out_data,
    .s0=4, // rows
    .s1=4, // cols
    .s2=4, // channels
    .s3=1
};


struct float_4tensor conv1_out = {
    .data=(float*) t_in_conv1_out_pool_out_data,
    .s0=3, // rows
    .s1=3, // cols
    .s2=8, // channels
    .s3=1
};

struct float_4tensor conv2_out = {
    /* reuse conv0_out data (both 64 floats)*/
    .data=(float*) conv02_out_data,
    .s0=2, // rows
    .s1=2, // cols
    .s2=16, // channels
    .s3=1
};


struct float_4tensor pool_out = {
    .data=(float*) t_in_conv1_out_pool_out_data,
    .s0=16, // channels
    .s1=1,
    .s2=1,
    .s3=1
};


struct float_4tensor t_out = {
    .data=(float*) t_out_data,
    .s0=10, // channels
    .s1=1,
    .s2=1,
    .s3=1
};


void run_model_and_set_led_brightness() {
    for (uint8_t row=0; row < 5; row++) {
        for (uint8_t col=0; col < 5; col++) {
            t4_set_value(&t_in, row, col, 0, 0,
                         !!get_led_brightness(row, col));
        }
    }
    
    
    VPORTB.DIR |= PIN6_bm;
    PORTB.OUT &= ~PIN6_bm;
    t4_convolve_2x2(&t_in, &conv0_kernel, &conv0_out);
    t4_add_conv_bias(&conv0_out, &conv0_bias);
    t4_relu(&conv0_out);
 
    
    t4_convolve_2x2(&conv0_out, &conv1_kernel, &conv1_out);
    t4_add_conv_bias(&conv1_out, &conv1_bias);
    t4_relu(&conv1_out);
    
    
    t4_convolve_2x2(&conv1_out, &conv2_kernel, &conv2_out);
    t4_add_conv_bias(&conv2_out, &conv2_bias);
    t4_relu(&conv2_out);
   
    
    
    t4_max_pool(&conv2_out, &pool_out);
    

    
    t4_matrix_mult(&dense_kernel, &pool_out, &t_out);
    t4_add(&t_out, &dense_bias);
    t4_softmax(&t_out);
    
    
    PORTB.OUT |= PIN6_bm;
    
    set_output_led_brightness(&t_out);
}
