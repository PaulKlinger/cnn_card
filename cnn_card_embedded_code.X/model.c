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
    set_led_brightness(8, 1, value_to_pwm_level(output->data[0], 1.0));
    for (uint8_t i=0; i < output->s0 - 1; i++) {
        set_led_brightness(7, i, value_to_pwm_level(output->data[i + 1], 1.0));
    }
}


void set_conv0_led_brightness (
    struct float_4tensor *conv0_out,
    uint8_t filter_idx
) {
    if (filter_idx >= 4) {
        for (uint8_t i=0; i < 15; i++) {
            set_led_brightness(i % 5, 5 + i / 5, 0);
        }
        set_led_brightness(5, 0, 0);
        return;
    }
    for (uint8_t i=0; i < 15; i++) {
        set_led_brightness(i % 5, 5 + i / 5,
            value_to_pwm_level(
                t4_get_value(conv0_out, i % 4, i / 4, filter_idx, 0),
                conv0_activation_99per));
    }
    set_led_brightness(5, 0,
            value_to_pwm_level(
                t4_get_value(conv0_out, 3, 3, filter_idx, 0),
                conv0_activation_99per));
}


void set_conv1_led_brightness (
    struct float_4tensor *conv1_out,
    uint8_t filter_idx
) {
    if (filter_idx >= 8) {
        for (uint8_t i=0; i < 7; i++) {
            set_led_brightness(5, i + 1, 0);
        }
        set_led_brightness(0, 8, 0);
        set_led_brightness(1, 8, 0);
        return;
    }
    for (uint8_t i=0; i < 7; i++) {
        set_led_brightness(5, i + 1,
            value_to_pwm_level(
                t4_get_value(conv1_out, i % 3, i / 3, filter_idx, 0),
                conv1_activation_99per));
    }
    set_led_brightness(0, 8,
            value_to_pwm_level(
                t4_get_value(conv1_out, 1, 2, filter_idx, 0),
                conv1_activation_99per));
    set_led_brightness(1, 8,
            value_to_pwm_level(
                t4_get_value(conv1_out, 2, 2, filter_idx, 0),
                conv1_activation_99per));
}


void set_conv2_led_brightness (
    struct float_4tensor *conv2_out,
    uint8_t filter_idx
) {
    for (uint8_t i=0; i < 4; i++) {
        set_led_brightness(2 + i, 8,
            value_to_pwm_level(
                t4_get_value(conv2_out, i % 2, i / 2, filter_idx, 0),
                conv2_activation_99per));
    }
}


void set_pool_led_brightness (
    struct float_4tensor *pool_out
) {
    for (uint8_t i=0; i < 6; i++) {
        set_led_brightness(8, i + 3,
            value_to_pwm_level(
                t4_get_value(pool_out, i, 0, 0, 0),
                conv2_activation_99per));
    }
    for (uint8_t i=0; i < 9; i++) {
        set_led_brightness(6, i,
            value_to_pwm_level(
                t4_get_value(pool_out, i + 6, 0, 0, 0),
                conv2_activation_99per));
    }
    set_led_brightness(8, 2,
        value_to_pwm_level(
            t4_get_value(pool_out, 15, 0, 0, 0),
            conv2_activation_99per));
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


void run_model_and_set_led_brightness(uint8_t filter_idx) {
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
    
    set_conv0_led_brightness(&conv0_out, filter_idx);
 
    
    t4_convolve_2x2(&conv0_out, &conv1_kernel, &conv1_out);
    t4_add_conv_bias(&conv1_out, &conv1_bias);
    t4_relu(&conv1_out);
    
    set_conv1_led_brightness(&conv1_out, filter_idx);
    
    
    t4_convolve_2x2(&conv1_out, &conv2_kernel, &conv2_out);
    t4_add_conv_bias(&conv2_out, &conv2_bias);
    t4_relu(&conv2_out);
   
    set_conv2_led_brightness(&conv2_out, filter_idx);
    
    t4_max_pool(&conv2_out, &pool_out);
    set_pool_led_brightness(&pool_out);
    
    t4_matrix_mult(&dense_kernel, &pool_out, &t_out);
    t4_add(&t_out, &dense_bias);
    t4_softmax(&t_out);
    
    
    PORTB.OUT |= PIN6_bm;
    
    set_output_led_brightness(&t_out);
}
