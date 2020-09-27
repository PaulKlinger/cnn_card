/* 
 * File:   model_weights.h
 * Author: kling
 *
 * Created on 26 September 2020, 23:02
 */

#ifndef MODEL_WEIGHTS_H
#define	MODEL_WEIGHTS_H

#ifdef	__cplusplus
extern "C" {
#endif

    struct float_4tensor conv0_kernel;
    struct float_4tensor conv0_bias;
    const float conv0_activation_99per;
    
    struct float_4tensor conv1_kernel;
    struct float_4tensor conv1_bias;
    const float conv1_activation_99per;
    
    struct float_4tensor conv2_kernel;
    struct float_4tensor conv2_bias;
    const float conv2_activation_99per;
    
    struct float_4tensor dense_kernel;
    struct float_4tensor dense_bias;
    
    
#ifdef	__cplusplus
}
#endif

#endif	/* MODEL_WEIGHTS_H */

