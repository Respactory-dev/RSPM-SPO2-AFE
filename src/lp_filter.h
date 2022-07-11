#ifndef LP_FILTER_H
#define LP_FILTER_H

#include "Arduino.h" 

extern "C"
{
  float ired_iir(float NewSample);
  float red_iir(float NewSample);
}




#endif
