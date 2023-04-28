#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H


extern float Q;
extern float R;
extern float cov;
extern float _Yaw;

void kf_init(void);
void kalman_filter(int angle_observation
									,float *_Yaw
									,float *Q
									,float R
									,float *cov
									,float Delta_T);

#endif


