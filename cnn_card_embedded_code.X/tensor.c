#include "tensor.h"



float t4_get_value(
    const struct float_4tensor *t,
    uint8_t i0,
    uint8_t i1,
    uint8_t i2,
    uint8_t i3
) {
    return t->data[
          i0 * (t->s1 * t->s2 * t->s3)
        + i1 * (t->s2 * t->s3)
        + i2 * t->s3
        + i3
    ];
}


void t4_set_value(
    const struct float_4tensor *t,
    uint8_t i0,
    uint8_t i1,
    uint8_t i2,
    uint8_t i3,
    float val
) {
    t->data[
          i0 * (t->s1 * t->s2 * t->s3)
        + i1 * (t->s2 * t->s3)
        + i2 * t->s3
        + i3
    ] = val;
}


void t4_add_conv_bias(
    struct float_4tensor *t,
    const struct float_4tensor *bias
) {
    /* Add 4tensor bias of shape (channels, 1, 1, 1)
     * to tensor t of shape (rows, cols, channels, 1) in place,
     * broadcasting over the rows & cols dimension
     */
    for (uint8_t channel=0; channel < t->s2; channel++) {
        for (uint8_t row=0; row < t->s0; row++) {
            for (uint8_t col=0; col < t->s1; col++) {
                t4_set_value(t, row, col, channel, 0,
                    t4_get_value(t, row, col, channel, 0)
                    + t4_get_value(bias, channel, 0, 0, 0)
                );
            }
        }
    }
}


void t4_relu(struct float_4tensor *t) {
    for (uint16_t i=0; i < t->s0 * t->s1 * t->s2 * t->s3; i++) {
        if (t->data[i] < 0) {
            t->data[i] = 0.0;
        }
    }
}


void t4_convolve_2x2(
    const struct float_4tensor *input,
    const struct float_4tensor *kernel,
    struct float_4tensor *res
) {
    /* convolve input 4tensor x of shape (rows, cols, channels, 1)
     * with kernel 4tensor of shape (rows, cols, channels, filters)
     * writing results to 4tensor res of shape (rows-1, cols-1, filters, 1)
     */
    
    uint8_t in_rows = input->s0;
    uint8_t in_cols = input->s1;
    uint8_t in_channels = input->s2;
    
    uint8_t filters = kernel->s3;

    for (uint8_t row=0; row < in_rows - 1; row++) {
        for (uint8_t col=0; col < in_cols - 1; col++) {
            for (uint8_t filter=0; filter < filters; filter++) {
                float sum = 0;
                for (uint8_t in_channel=0; in_channel < in_channels; in_channel++) {
                    for (uint8_t filter_row=0; filter_row < 2; filter_row++) {
                        for (uint8_t filter_col=0; filter_col < 2; filter_col++) {
                            sum += t4_get_value(input,
                                    row + filter_row, col + filter_col,
                                    in_channel, 0)
                                   * t4_get_value(kernel,
                                        filter_row, filter_col,
                                        in_channel, filter);
                        }
                    }
                }
                t4_set_value(res, row, col, filter, 0, sum);
            }
        }
    }
}
