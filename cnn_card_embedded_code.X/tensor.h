#ifndef TENSOR_H
#define	TENSOR_H

#include <stdint.h>

#ifdef	__cplusplus
extern "C" {
#endif
    
    
struct float_4tensor {
    float *data;
    uint8_t s0, s1, s2, s3;
};


static inline float t4_get_value(
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


static inline void t4_set_value(
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


float t4_get_value(
    const struct float_4tensor *t,
    uint8_t i0,
    uint8_t i1,
    uint8_t i2,
    uint8_t i3
);


void t4_set_value(
    const struct float_4tensor *t,
    uint8_t i0,
    uint8_t i1,
    uint8_t i2,
    uint8_t i3,
    float val
);

void t4_matrix_mult(
    const struct float_4tensor *m1,
    const struct float_4tensor *m2,
    struct float_4tensor *res
);

void t4_add(
    struct float_4tensor *t1,
    const struct float_4tensor *t2
);

void t4_add_conv_bias(
    struct float_4tensor *t,
    const struct float_4tensor *bias
);

void t4_relu(
    struct float_4tensor *t
);

void t4_softmax(
    struct float_4tensor *t
);

void t4_max_pool(
    const struct float_4tensor *t,
    struct float_4tensor *res
);

void t4_convolve_2x2(
    const struct float_4tensor *x,
    const struct float_4tensor *kernel,
    struct float_4tensor *res
);

#ifdef	__cplusplus
}
#endif

#endif	/* TENSORS_H */

