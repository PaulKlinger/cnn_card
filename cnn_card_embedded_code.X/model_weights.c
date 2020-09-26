#include "tensor.h"


const float conv0_kernel_data[] = {
    1.5657663 ,  0.88373774, -1.2657819 , -0.68050164,  1.3737351 ,
    -1.0296128 , -1.3210745 ,  0.36918926,  0.76399225,  1.5246062 ,
    -0.77700794,  1.9321772 , -0.89375335, -0.785154  ,  0.58246917,
     0.5354092
};

const struct float_4tensor conv0_kernel = {
    .data=(float*) conv0_kernel_data,
    .s0=2, // rows
    .s1=2, // cols
    .s2=1, // input channels
    .s3=4  // filters
};

const float conv0_bias_data[] = {
    -7.0469618e-02,  2.3160256e-01,  6.1115247e-01, -1.8146499e-04
};

const struct float_4tensor conv0_bias = {
    .data=(float*) conv0_bias_data,
    .s0=4, // channels
    .s1=1,
    .s2=1,
    .s3=1 
};
