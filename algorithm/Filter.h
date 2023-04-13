#ifndef _FILTER_H
#define _FILTER_H

#include <stdint.h>

typedef struct
{
 float Input_Butter[3];
 float Output_Butter[3];
}Butter_BufferData;

typedef struct
{
  float a[3];
  float b[3];
}Butter_Parameter;

float GildeAverageValueFilter(float NewValue, float *Data, uint8_t size);
float Butterworth_Filter(float curr_inputer, Butter_BufferData *Buffer, Butter_Parameter *Parameter);
void Set_Cutoff_Frequency(float sample_frequent, float cutoff_frequent, Butter_Parameter *LPF);

#endif
