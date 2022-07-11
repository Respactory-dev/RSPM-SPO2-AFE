/**************************************************************
WinFilter version 0.8
http://www.winfilter.20m.com
akundert@hotmail.com

Filter type: Band Pass
Filter model: Butterworth
Filter order: 2
Sampling Frequency: 200 Hz
Fc1 and Fc2 Frequencies: 1.000000 Hz and 5.000000 Hz
Coefficents Quantization: float

Z domain Zeros
z = -1.000000 + j 0.000000
z = -1.000000 + j 0.000000
z = 1.000000 + j 0.000000
z = 1.000000 + j 0.000000

Z domain Poles
z = 0.981577 + j -0.030023
z = 0.981577 + j 0.030023
z = 0.925050 + j -0.111231
z = 0.925050 + j 0.111231
***************************************************************/
#define NCoef 4
float bp_iir(float NewSample) {
    float ACoef[NCoef+1] = {
        0.00425133877684963360,
        0.00000000000000000000,
        -0.00850267755369926710,
        0.00000000000000000000,
        0.00425133877684963360
    };

    float BCoef[NCoef+1] = {
        1.00000000000000000000,
        -3.81325425602666270000,
        5.46451668975586900000,
        -3.48842176300547370000,
        0.83718165125641664000
    };

    static float bp_y[NCoef+1]; //output samples
    static float bp_x[NCoef+1]; //input samples
    int n;

    //shift the old samples
    for(n=NCoef; n>0; n--) {
       bp_x[n] = bp_x[n-1];
       bp_y[n] = bp_y[n-1];
    }

    //Calculate the new output
    bp_x[0] = NewSample;
    bp_y[0] = ACoef[0] * bp_x[0];
    for(n=1; n<=NCoef; n++)
        bp_y[0] += ACoef[n] * bp_x[n] - BCoef[n] * bp_y[n];
    
    return bp_y[0];
}
