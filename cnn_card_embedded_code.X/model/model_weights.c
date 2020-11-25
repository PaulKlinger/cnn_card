#include "tensor.h"

/* 99th percentile of activations per layer
 * copied from cnn/train.ipynb
 */

const float conv0_activation_99per = 3.298433542251587;
const float conv1_activation_99per = 4.464225769042969;
const float conv2_activation_99per = 5.400259494781494;

/* weight tensor data (definitions below)
 * (copied from cnn/train.ipynb)
 */

const float conv0_kernel_data[] = {
    7.020391e-01, 1.0258303e+00, 8.72381e-01, -1.1648293e+00, 9.5234656e-01,
    7.509327e-01, -9.5353985e-01, -8.4196067e-01, 1.0978298e+00, 1.032137e+00,
    8.4567595e-01, 6.5854657e-01, -5.5697376e-01, 1.4566342e+00, -4.1969696e-01,
    9.727429e-01,
};

const float conv0_bias_data[] = {
    -1.446728e-01, -2.1616767e-01, 5.0059897e-01, 5.065947e-01,
};

const float conv1_kernel_data[] = {
    4.2334983e-01, 1.274771e-01, -1.3428767e-01, 7.536915e-01, -3.5425636e-01,
    9.6646294e-02, 1.6611163e-01, 5.1564974e-01, -9.327773e-02, -1.809538e-01,
    3.956333e-01, 5.099228e-01, 3.4310982e-01, 3.0296388e-01, -1.6110633e-01,
    4.999903e-01, 4.2740902e-01, -1.3602641e-01, 1.5409742e-01, 1.3710105e-01,
    2.734646e-01, -6.895231e-01, 5.1433768e-02, 1.4937659e-01, -9.385792e-02,
    2.3060943e-01, -7.2521734e-01, -2.5629053e-01, -1.8719584e-01, 2.724104e-01,
    1.24459326e-01, 5.3574786e-02, -1.273949e-01, 2.6588535e-01, -4.7672948e-01,
    -5.9506856e-02, 2.1918508e-01, 3.8890484e-01, -5.388753e-01, 2.488933e-01,
    -3.747219e-01, 3.6291236e-01, 1.9091092e-01, -2.7426612e-01, -6.6629425e-02,
    -2.3856506e-01, -2.4901028e-01, 4.895413e-01, 9.311364e-02, -3.7328753e-01,
    -9.452559e-02, -1.4837721e-01, -1.6631862e-02, 1.6128461e-01, 4.9406403e-01,
    3.5423458e-02, 1.995391e-01, -2.4690571e-01, -4.8778573e-01, 4.462772e-01,
    -8.997867e-03, 8.8146456e-02, 1.2446701e-01, -5.317949e-01, -4.7253225e-02,
    -4.8936647e-01, 5.0691897e-01, 5.823606e-03, 2.6115805e-01, 3.5123125e-02,
    -2.0525528e-01, -3.1182683e-01, 5.5622924e-02, -3.9421833e-01, 2.282095e-01,
    5.5946666e-01, 5.436027e-01, 1.6537191e-01, 4.4455406e-01, -5.3262246e-01,
    5.837681e-01, -4.872255e-02, -4.8092988e-01, 2.0632754e-01, 5.511814e-02,
    -4.1662964e-01, 5.929224e-01, -1.9860843e-01, 4.0426627e-01, -1.777021e-01,
    2.3336883e-01, 5.785766e-02, -5.2623445e-01, 6.560828e-01, 4.440026e-01,
    1.981022e-01, -4.560376e-02, -3.924611e-01, 9.7193964e-02, -2.5336322e-01,
    -1.1675019e-01, -2.3272533e-02, -1.0159597e-01, -1.5049314e-02, -7.477427e-01,
    8.57328e-01, 2.2635952e-01, -3.5968745e-01, 7.4628633e-01, 2.4808954e-01,
    1.4524686e-01, 1.2191307e-01, -5.5115283e-02, -2.2187577e-01, -9.597869e-02,
    -5.1998574e-01, 3.481289e-02, 6.645641e-02, 1.8831986e-01, -1.74282e-01,
    2.8714508e-01, 1.6601826e-01, -8.3671695e-01, 1.19240105e-01, 2.2873005e-01,
    5.6502543e-02, -4.4219702e-02, -8.787338e-03,
};

const float conv1_bias_data[] = {
    2.5175214e-01, 6.1886847e-02, -3.749722e-02, -5.363371e-01, -7.6928064e-02,
    -1.17897965e-01, 1.895641e-01, -3.4980022e-03,
};

const float conv2_kernel_data[] = {
    1.4048685e-01, 2.7043302e-02, 8.843822e-02, -6.1392654e-02, 4.991623e-02,
    -2.415549e-01, -1.6106975e-01, -2.085262e-01, -1.0016593e-01, -9.060042e-02,
    3.6989655e-02, -1.337826e-01, -2.854356e-01, -1.9639336e-01, 1.3251945e-01,
    1.6428024e-01, 2.5640306e-01, 4.7403723e-02, -5.9759684e-02, 3.3487402e-02,
    1.7324297e-02, -1.4170748e-01, -2.1530184e-01, 8.8608645e-02, 8.285963e-03,
    -1.03011794e-01, -3.6231212e-02, -1.3828726e-01, 4.1004708e-01, -2.6281044e-01,
    -4.8069574e-02, -6.440924e-02, 1.4153118e-01, 1.477348e-01, 7.219325e-02,
    1.3063684e-01, 4.1675913e-01, -1.8021777e-01, -3.8306683e-01, 1.7496142e-01,
    4.698904e-01, 4.1319802e-02, -3.5843853e-02, 1.0334995e-01, 1.287305e-01,
    4.924072e-02, 6.5630525e-02, 4.090808e-01, 2.3980467e-02, 2.8484783e-01,
    -3.2076308e-01, 3.6712736e-01, 1.8586512e-01, 2.4361597e-01, 1.6724026e-01,
    -3.882724e-02, -1.7070071e-01, 2.4775827e-01, 1.1987926e-02, -1.8261714e-01,
    2.2601038e-01, -1.2035954e-01, 2.0026231e-01, 2.368507e-02, -6.3172523e-03,
    -2.302375e-01, 1.2326322e-01, 6.4572744e-02, 1.6974951e-01, 1.14855334e-01,
    -1.1228653e-01, -1.7156668e-02, 1.6680838e-01, 2.4985549e-01, -2.066468e-01,
    -9.613202e-02, -2.236968e-02, 1.858208e-01, -9.564798e-02, 1.8587677e-01,
    -8.144202e-02, -1.09851584e-01, 1.866784e-01, -9.0954736e-02, 1.092e-02,
    -8.361029e-02, 4.685832e-02, -1.72731e-02, 1.5393977e-01, -2.8059882e-01,
    -2.5896296e-01, 5.2362405e-02, 4.9170446e-02, 2.03592e-01, -3.6607265e-02,
    2.5768407e-02, -2.913822e-02, 3.0886238e-02, -3.3048144e-01, 3.147917e-03,
    -1.3221885e-01, -2.8803852e-01, -5.5002388e-02, 6.5135404e-02, 7.400497e-02,
    -4.9009018e-02, 8.094114e-02, -2.0808113e-01, 2.697102e-01, -2.4880417e-01,
    2.1878457e-01, 1.3361329e-01, 1.2092272e-01, 6.0900018e-02, 3.398672e-01,
    1.0447728e-01, -2.2264676e-01, 2.2287498e-01, 1.3949366e-02, 1.3304922e-01,
    4.730048e-01, -6.94619e-02, 3.434295e-02, -1.4290468e-01, 4.3392718e-02,
    2.4899818e-01, 5.026047e-02, 1.1450102e-01, 1.8855499e-01, 1.9728707e-01,
    2.000137e-01, -9.361485e-02, -5.2287817e-02, -1.5401678e-01, -9.7099334e-02,
    1.2571743e-01, -1.5016322e-01, 2.0309342e-02, 1.4970875e-01, -9.072091e-02,
    -1.8907721e-01, 1.8553686e-01, 1.7887273e-01, -2.8631456e-02, 7.108218e-02,
    -6.44639e-02, -7.039104e-02, 1.4276125e-01, 1.3857755e-01, -2.1578676e-01,
    1.0589755e-01, 2.7642224e-02, 3.4178478e-01, 4.5925617e-02, -1.630917e-01,
    -1.6988932e-01, 2.5436568e-01, 2.8098577e-01, 8.503082e-02, -7.454835e-02,
    -1.940461e-01, 1.458269e-01, -3.6636826e-02, -3.0110094e-01, 1.20357305e-01,
    -6.0534477e-02, 3.515859e-01, -4.2296227e-02, 2.6574728e-01, 1.6646254e-01,
    2.3859112e-01, 1.8364346e-01, 2.6988873e-01, 2.8116152e-02, -7.45805e-02,
    -3.7204653e-02, 3.1153142e-01, 1.20743215e-01, -3.342519e-01, 5.27356e-01,
    1.5354563e-01, 1.4652988e-02, -1.11059815e-01, -2.933065e-01, -3.8684416e-01,
    1.7116994e-01, -2.9714268e-02, -7.887068e-02, 2.1266334e-01, -3.142241e-01,
    -1.023351e-01, -1.3312941e-02, 2.7505577e-01, -3.9597703e-03, 8.6178906e-02,
    4.7022797e-02, 1.3258843e-01, 2.6515937e-01, 9.7690836e-02, -2.7745439e-02,
    1.0771104e-01, 2.2404729e-01, -2.482704e-02, 1.210718e-02, -8.245886e-02,
    2.963514e-02, 2.7059668e-01, 4.7694054e-02, 2.0226565e-01, -4.8511554e-02,
    -9.632656e-02, 1.8619078e-01, 3.6012407e-02, -2.7423957e-01, -1.2023518e-01,
    4.165318e-02, -1.472569e-01, -5.3904645e-02, 7.9235844e-02, -9.572521e-02,
    9.449326e-03, -1.3755782e-01, 3.43756e-01, 1.507718e-01, -1.544927e-01,
    -1.8855672e-01, -8.490892e-02, -3.6133606e-02, -3.558966e-01, -1.445872e-01,
    -4.338772e-02, 8.2580216e-02, 2.2690058e-01, 3.3825003e-02, 1.1826525e-01,
    -2.0095874e-01, 1.1793385e-01, -2.2835515e-01, -7.80423e-02, -2.0675954e-01,
    1.346732e-01, -8.695924e-02, -7.040624e-02, 4.4794098e-01, -2.9012932e-02,
    1.8441099e-01, -2.4653696e-02, 1.9813734e-01, 2.808278e-01, 6.917885e-02,
    1.4953296e-01, -6.308014e-02, 3.7942104e-02, -5.1585145e-02, 2.7839792e-01,
    1.6332535e-01, 2.4627781e-01, -3.9535433e-02, 7.425761e-02, -2.475268e-01,
    -4.106776e-01, -1.476219e-01, -1.3006483e-01, -2.603322e-01, 5.8476664e-02,
    -2.0612647e-01, -9.551717e-02, 1.2264582e-01, -1.562386e-01, -1.5251563e-01,
    -1.2136982e-01, -2.3709345e-01, -4.4749114e-03, 1.1818687e-01, 1.4585716e-01,
    3.42233e-02, -8.24335e-02, -5.180522e-02, -1.6915856e-01, 1.1801564e-01,
    -1.541091e-01, 2.4293767e-01, 4.1984458e-02, 6.575193e-03, 1.0677213e-01,
    -2.2933015e-01, -3.35277e-01, 1.8127686e-01, 2.1784277e-01, -6.843825e-02,
    1.0449642e-01, 1.9020376e-01, -8.700962e-03, -2.4573117e-01, -1.7355978e-01,
    -1.6942142e-01, -2.2607218e-01, -1.8903835e-01, 2.166755e-02, -7.708672e-02,
    4.719686e-01, 5.5947606e-02, -1.18800424e-01, 1.3670686e-01, 5.353344e-02,
    -7.2554365e-02, 2.1130517e-02, 4.1525032e-02, 1.0114497e-01, 2.0205496e-01,
    6.712289e-02, 3.6876816e-01, -6.8838135e-03, -1.689209e-01, 1.0085282e-02,
    2.4535538e-01, 8.500873e-02, -5.469586e-02, 2.1283352e-01, -2.0270567e-01,
    1.6458817e-01, 1.6731916e-01, 1.05563514e-01, 5.7071533e-02, 2.524255e-01,
    4.2465773e-01, 1.8097247e-01, 1.8702264e-01, 3.9405206e-01, -6.536167e-02,
    -1.998147e-01, 1.12241894e-01, 2.1883036e-01, 2.5815713e-01, 8.7377876e-02,
    4.7486555e-02, 2.733977e-01, 2.2242136e-01, 3.134386e-02, 1.4938416e-01,
    -1.036097e-01, -3.2242015e-01, 1.8168818e-02, -3.078569e-02, -3.2093516e-01,
    -5.281441e-03, -3.1025836e-01, 1.0037778e-01, -7.561027e-02, 2.3863614e-02,
    4.404039e-02, 2.5714293e-01, 2.8463268e-01, -4.929887e-02, -3.3882275e-02,
    -2.0596872e-01, 3.2061508e-01, 8.2342826e-02, 1.382553e-01, 2.4935912e-02,
    4.393673e-02, -1.0573537e-01, 6.0861085e-02, 1.664828e-01, -4.848999e-03,
    -1.764841e-01, 5.394449e-02, 1.3691029e-01, -9.435618e-02, -2.6550457e-02,
    8.9976475e-02, -1.12522654e-01, -1.9684523e-01, -1.4760219e-01, 7.94454e-02,
    -2.3365218e-02, -1.8472928e-01, 1.6885692e-01, 1.7930375e-01, -2.2778969e-01,
    6.94819e-02, -7.88285e-02, -1.2185754e-02, -3.1105095e-01, -4.6370335e-02,
    -1.5905312e-01, 1.6488142e-01, 1.8354046e-01, -3.3927894e-01, -1.9512536e-01,
    -1.9221261e-01, 9.508465e-02, 9.479639e-02, 6.5521024e-02, 3.2844904e-01,
    -4.362041e-02, 1.9572937e-01, 1.886458e-01, 2.3642395e-01, -7.924889e-02,
    2.3462203e-01, 2.6582748e-02, 1.4241572e-01, 9.241922e-02, -6.866822e-02,
    -6.4639516e-02, -1.6988105e-01, 1.4344013e-01, 1.5815869e-02, 8.618344e-02,
    -4.647521e-01, 4.0291164e-02, 4.6619177e-01, 3.3032888e-01, -7.365269e-02,
    -8.913618e-02, -3.346408e-02, 5.3350776e-01, -1.5560964e-01, 2.6011929e-02,
    1.2163478e-02, 8.5060455e-02, 2.1598269e-01, -2.2800462e-03, -9.4378494e-02,
    2.4876315e-03, 1.6737387e-01, -8.059993e-03, 4.0982926e-01, 2.5280628e-01,
    4.8689105e-02, 2.4133788e-02, 3.2549214e-02, -1.03405975e-01, -5.5195626e-02,
    2.7918482e-02, -1.836737e-02, 2.0782018e-01, -1.1985488e-01, 2.186196e-02,
    2.3548122e-01, -1.9536218e-01, 2.8793547e-01, 1.4588597e-01, -1.3754556e-01,
    -8.9637384e-02, -3.4673512e-02, -1.01761535e-01, 1.9198713e-01, -1.1344833e-01,
    3.978898e-02, 6.1346833e-02, 2.3535754e-01, 4.881431e-01, 7.163611e-02,
    1.3072595e-01, 2.548653e-01, 1.2219752e-02, 2.2593254e-01, 1.2289079e-01,
    5.752292e-02, 4.1584194e-02, 2.7233872e-01, 1.1712766e-01, 2.285963e-01,
    2.1114625e-01, 1.7963453e-01, 1.8284848e-01, -2.724864e-01, -2.5731978e-01,
    -6.8139225e-02, -8.722281e-02, 4.6623483e-02, -1.3759002e-01, 3.0346447e-01,
    1.7288308e-01, -4.1010167e-02, -8.37031e-03, 1.0090028e-01, -3.9306313e-02,
    2.9607785e-01, -1.2301375e-02, -1.1124951e-01, -4.2756412e-02, 8.9819886e-02,
    1.6277504e-01, 2.3177351e-01, -1.7777433e-01, -1.7342056e-01, -1.5660514e-01,
    1.5336999e-02, 1.4650084e-02, 4.001608e-02, -2.1707983e-01, -1.4396352e-01,
    -2.4845469e-01, 5.3915627e-02, 9.616663e-02, 1.1657415e-02, 1.9310006e-01,
    1.5380435e-01, 8.577431e-02, -2.9727211e-03, -5.119954e-02, -2.6815698e-01,
    -1.285168e-01, -5.673727e-02, -9.04181e-02, -2.449092e-02, -7.1802124e-02,
    5.8120873e-02, -3.016094e-01,
};

const float conv2_bias_data[] = {
    -1.5632814e-01, -1.7647623e-01, 4.007947e-02, -3.2597475e-02, -1.8165756e-02,
    -9.216547e-03, 5.2559364e-04, -4.3133788e-02, 2.6531614e-02, -5.0415523e-02,
    1.07844226e-01, -8.6377285e-02, -2.9700091e-02, -4.838905e-02, 7.68175e-02,
    1.2490478e-01,
};

const float dense_kernel_data[] = {
    -2.1050208e+00, -1.2888659e+00, 2.6487717e-01, -4.0920442e-01, 8.8571846e-01,
    1.9862264e+00, 3.0167315e+00, -9.673082e-01, 5.06366e-01, -6.7465484e-01,
    1.8899652e+00, -1.6867628e+00, -1.0057436e+00, -1.5392945e+00, 1.6556948e+00,
    -2.0517004e+00, 2.139746e+00, -6.411151e-01, -5.609446e-01, -1.7557813e+00,
    5.835209e-01, -1.8541792e-01, -3.3546991e+00, -1.4940518e+00, 3.62447e-02,
    -9.5899963e-01, 3.97191e+00, -1.2675092e+00, -1.7027527e+00, 4.065831e+00,
    4.7643197e-01, -2.2531974e+00, -4.182212e-02, -2.1469197e+00, 2.8382618e+00,
    -1.1024497e+00, -1.7316482e+00, 1.2634846e+00, -1.5705428e+00, 7.547195e-01,
    1.3475882e+00, -5.6772456e-02, -6.892607e-02, 4.1814194e+00, -1.1831974e+00,
    -1.2980108e+00, 3.2816485e-01, -6.5174836e-01, 8.8550895e-01, 1.3565218e+00,
    2.2802205e+00, -9.043323e-02, 1.8852184e-02, -3.2379587e+00, 7.4874395e-01,
    1.2436306e+00, 1.2796718e+00, -2.670708e+00, -1.6433595e+00, -1.2636104e+00,
    5.7065684e-01, 3.564464e-02, -1.7639555e+00, -2.1643884e-02, -3.375192e-01,
    -3.0789168e+00, 5.503039e-01, -1.2037907e+00, 1.371464e+00, -1.0841372e+00,
    8.2016826e-01, -1.894203e+00, -2.8630047e+00, 3.3403141e+00, 2.0709543e+00,
    4.3197637e+00, 9.0591764e-01, 2.6966357e+00, 1.6653547e+00, -7.2042006e-01,
    7.304425e-01, 3.1491158e+00, -1.4743009e+00, 2.5554583e+00, -5.424053e-01,
    1.4552903e-01, -2.1793842e+00, -4.263962e-01, -2.0422065e+00, -1.6013503e+00,
    -1.7773019e+00, -2.8125973e+00, -1.2950343e+00, -1.7211294e+00, 8.336129e-01,
    1.9441879e+00, 1.1279765e+00, 3.0552474e-01, -1.5343947e+00, -3.271247e-01,
    2.6317863e+00, 8.99958e-01, -2.93929e+00, 2.4754703e+00, -1.5371373e+00,
    8.610173e-01, -3.546785e-01, -1.7081878e-01, 5.8463436e-02, -5.688945e+00,
    -2.990285e-01, -1.9158258e+00, -6.0830116e-01, -1.354684e+00, 5.8668755e-02,
    -1.6474639e+00, -2.6802528e+00, 4.4401643e-01, -1.7899154e+00, 1.4639356e+00,
    3.1540236e+00, 2.8279272e-01, 2.5572747e-01, -4.3001914e+00, -1.7215252e+00,
    3.0387948e+00, -3.7874767e-01, 2.5296707e+00, -9.106793e-01, 8.0545443e-01,
    -7.494044e-01, 7.086275e-02, 1.5919234e+00, 8.6274695e-01, 5.429404e-01,
    -6.3095856e-01, 8.462711e-01, 1.0806475e-02, -2.2370462e+00, 4.7786728e-01,
    1.2399755e+00, -2.9211602e+00, -1.7445532e+00, 4.9983785e-02, -7.516006e-01,
    -1.1201372e+00, -1.7433944e+00, 1.6023718e+00, -2.6503468e+00, -1.2129475e+00,
    2.0043445e+00, -1.3965753e+00, 9.559851e-02, 5.4492116e-01, 1.4797763e-02,
    -7.8738564e-01, 2.326161e+00, 1.1061105e+00, -7.3313093e-01, 1.975456e+00,
};

const float dense_bias_data[] = {
    1.8121316e+00, -5.4363614e-01, -1.3337061e+00, 2.1079712e-01, 8.1284297e-01,
    3.3748004e-01, -6.7212075e-01, 3.100645e-01, 3.179907e-01, -9.563908e-01,
};


/* Weight tensor definitions */

const struct float_4tensor conv0_kernel = {
    .data=(float*) conv0_kernel_data,
    .s0=2, // rows
    .s1=2, // cols
    .s2=1, // input channels
    .s3=4  // filters
};


const struct float_4tensor conv0_bias = {
    .data=(float*) conv0_bias_data,
    .s0=4, // channels
    .s1=1,
    .s2=1,
    .s3=1 
};

const struct float_4tensor conv1_kernel = {
    .data=(float*) conv1_kernel_data,
    .s0=2, // rows
    .s1=2, // cols
    .s2=4, // input channels
    .s3=8  // filters
};

const struct float_4tensor conv1_bias = {
    .data=(float*) conv1_bias_data,
    .s0=8, // channels
    .s1=1,
    .s2=1,
    .s3=1 
};

const struct float_4tensor conv2_kernel = {
    .data=(float*) conv2_kernel_data,
    .s0=2, // rows
    .s1=2, // cols
    .s2=8, // input channels
    .s3=16  // filters
};

const struct float_4tensor conv2_bias = {
    .data=(float*) conv2_bias_data,
    .s0=16, // channels
    .s1=1,
    .s2=1,
    .s3=1 
};

const struct float_4tensor dense_kernel = {
    .data=(float*) dense_kernel_data,
    .s0=10, // units
    .s1=16, // input channels
    .s2=1,
    .s3=1
};

const struct float_4tensor dense_bias = {
    .data=(float*) dense_bias_data,
    .s0=10, // units
    .s1=1,
    .s2=1,
    .s3=1 
};