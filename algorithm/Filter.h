#ifndef _FILTER_H
#define _FILTER_H
#define MVF_LENGTH 10
#define PI 3.1415926f

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

typedef struct
{
	int butter[MVF_LENGTH];
	float sum;
}Moving_Filter;
typedef Moving_Filter* MvfPtr;

float GildeAverageValueFilter(float NewValue, float *Data, uint8_t size);
float Butterworth_Filter(float curr_inputer, Butter_BufferData *Buffer, Butter_Parameter *Parameter);
float Speed_low_filter(MvfPtr record, int Observation_new);
void Set_Cutoff_Frequency(float sample_frequent, float cutoff_frequent, Butter_Parameter *LPF);
void MVF_init(MvfPtr record);

#endif
