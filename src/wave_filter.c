

/*
Filter 프로그램에서 함수를 만들고,
계수는 http://engineerjs.com/?sidebar=docs/iir.html 에서 생성함

butter worth
filter order = 1
bandpass 
cutting = 0.5Hz, 5Hz
sampling 200Hz
*/

#define NCoef 2
float wave_iir(float NewSample) {
    float ACoef[NCoef+1] = {
        0.06612,
        0.00000000000000000000,
        -0.06612
    };

    float BCoef[NCoef+1] = {
        1.00000000000000000000,
        -1.865,
        0.8678
    };

    static float wave_y[NCoef+1]; //output samples
    static float wave_x[NCoef+1]; //input samples
    int n;

    //shift the old samples
    for(n=NCoef; n>0; n--) {
       wave_x[n] = wave_x[n-1];
       wave_y[n] = wave_y[n-1];
    }

    //Calculate the new output
    wave_x[0] = NewSample;
    wave_y[0] = ACoef[0] * wave_x[0];
    for(n=1; n<=NCoef; n++)
        wave_y[0] += ACoef[n] * wave_x[n] - BCoef[n] * wave_y[n];
    
    return wave_y[0];
}