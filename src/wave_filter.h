#ifndef WAVE_FILTER_H
#define WAVE_FILTER_H

#include "Arduino.h" 

extern "C"
{
  float wave_iir(float NewSample);
}




#endif
