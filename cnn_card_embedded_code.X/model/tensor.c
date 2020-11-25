#include <float.h>
#include <math.h>

#include "tensor.h"


void t4_add_conv_bias(
    struct float_4tensor *t,
    const struct float_4tensor *bias
) {
    /* Add 4tensor bias of shape (channels, 1, 1, 1)
     * to 4tensor t of shape (rows, cols, channels, 1) in place,
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


void t4_add(
    struct float_4tensor *t1,
    const struct float_4tensor *t2
) {
    /* Add 4tensor t2 of shape (a, b, c, d)
     * to 4tensor t1 of shape (a, b, c, d) in place
     */
    for (uint16_t i=0; i < t1->s0 * t1->s1 * t1->s2 * t1->s3; i++) {
        t1->data[i] += t2->data[i];
    }
}


void t4_relu(struct float_4tensor *t) {
    /* apply relu function (elementwise maximum with 0) to 4tensor t */
    for (uint16_t i=0; i < t->s0 * t->s1 * t->s2 * t->s3; i++) {
        if (t->data[i] < 0) {
            t->data[i] = 0.0;
        }
    }
}


float t4_max(struct float_4tensor *t) {
    /* returns maximum entry in 4tensor t */
    float max = t->data[0];
    for (uint16_t i=1; i < t->s0 * t->s1 * t->s2 * t->s3; i++) {
        if (t->data[i] > max) {
            max = t->data[i];
        }
    }
    return max;
}


void t4_softmax(
    struct float_4tensor *t
) {
    /* applies softmax function to 4tensor of shape (channels, 1, 1, 1)
     * softmax(m) = exp(m) / sum(exp(m))
     * (the max entry of m is subtracted before exponentiating to prevent
     *  overflows)
     */
    float sum = 0;
    float max_value = t4_max(t);
    for (uint16_t i=0; i < t->s0 * t->s1 * t->s2 * t->s3; i++) {
        t->data[i] = exp(t->data[i] - max_value);
        sum += t->data[i];
    }
    for (uint16_t i=0; i < t->s0 * t->s1 * t->s2 * t->s3; i++) {
        t->data[i] = t->data[i] / sum;
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


void t4_max_pool(
    const struct float_4tensor *t,
    struct float_4tensor *res
) {
    /* apply max pooling to input 4tensor t of shape (rows, cols, channels, 1)
     * saving the results in 4tensor res of shape (channels, 1, 1, 1)
     */
    
    for (uint8_t channel=0; channel < t->s2; channel++) {
        float max = -FLT_MAX;
        for (uint8_t row=0; row < t->s0; row++) {
            for (uint8_t col=0; col < t->s1; col++) {
                float val = t4_get_value(t, row, col, channel, 0);
                if (val > max) {
                    max = val;
                }
            }
        }
        t4_set_value(res, channel, 0, 0, 0, max);
    }
}

void t4_matrix_mult(
    const struct float_4tensor *m1,
    const struct float_4tensor *m2,
    struct float_4tensor *res
) {
    /* Matrix multiply 4tensor m1 of shape (a, b, 1, 1)
     * with 4tensor m2 of shape (b, c, 1, 1)
     * storing the result in 4tensor res of shape (a, c, 1, 1)
     */
    float res_entry;
    for (uint8_t row=0; row < m1->s0; row++) {
        for (uint8_t col=0; col < m2->s1; col++) {
            res_entry = 0;
            for (uint8_t i=0; i<m1->s1; i++) {
                // m1[row, i] * m2[i, col]
                res_entry += t4_get_value(m1, row, i, 0, 0)
                           * t4_get_value(m2, i, col, 0, 0);
            }
            // res is shape (m1->s0, m2->s1)
            // res[row, col]
            t4_set_value(res, row, col, 0, 0, res_entry);
        };
    };
}