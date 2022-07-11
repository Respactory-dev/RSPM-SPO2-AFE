/**************************************************************
WinFilter version 0.8
http://www.winfilter.20m.com
akundert@hotmail.com

Filter type: Low Pass
Filter model: Chebyshev
Filter order: 2
Sampling Frequency: 200 Hz
Cut Frequency: 3.000000 Hz
Pass band Ripple: 1.000000 dB
Coefficents Quantization: float

Z domain Zeros
z = -1.000000 + j 0.000000
z = -1.000000 + j 0.000000

Z domain Poles
z = 0.946243 + j -0.080084
z = 0.946243 + j 0.080084
***************************************************************/
#define NCoef 2
float ired_iir(float NewSample) {
    float ACoef[NCoef+1] = {
        0.00232582227512601090,
        0.00465164455025202180,
        0.00232582227512601090
    };

    float BCoef[NCoef+1] = {
        1.00000000000000000000,
        -1.89248598789308890000,
        0.90178928833898697000
    };

    static float ired_y[NCoef+1]; //output samples
    static float ired_x[NCoef+1]; //input samples
    int n;

    //shift the old samples
    for(n=NCoef; n>0; n--) {
       ired_x[n] = ired_x[n-1];
       ired_y[n] = ired_y[n-1];
    }

    //Calculate the new output
    ired_x[0] = NewSample;
    ired_y[0] = ACoef[0] * ired_x[0];
    for(n=1; n<=NCoef; n++)
        ired_y[0] += ACoef[n] * ired_x[n] - BCoef[n] * ired_y[n];
    
    return ired_y[0];
}

float red_iir(float NewSample) {
    float ACoef[NCoef+1] = {
        0.00232582227512601090,
        0.00465164455025202180,
        0.00232582227512601090
    };

    float BCoef[NCoef+1] = {
        1.00000000000000000000,
        -1.89248598789308890000,
        0.90178928833898697000
    };

    static float red_y[NCoef+1]; //output samples
    static float red_x[NCoef+1]; //input samples
    int n;

    //shift the old samples
    for(n=NCoef; n>0; n--) {
       red_x[n] = red_x[n-1];
       red_y[n] = red_y[n-1];
    }

    //Calculate the new output
    red_x[0] = NewSample;
    red_y[0] = ACoef[0] * red_x[0];
    for(n=1; n<=NCoef; n++)
        red_y[0] += ACoef[n] * red_x[n] - BCoef[n] * red_y[n];
    
    return red_y[0];
}


