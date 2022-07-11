#ifndef BP_FILTER_H
#define BP_FILTER_H

#include "Arduino.h" 

extern "C"
{
  float bp_iir(float NewSample);
}




#endif
