#include "kalman_filter.h"
#include "ahrs.h"
#include "Filter.h"

float Q;
float R;
float cov;
float _Yaw;

void kf_init()
{
	R = 0.45;
	Q = 0.005 * R;
	cov = 1;
	_Yaw = Yaw;
}
/****
* @brief  kalman filter for Angle Yaw
* @param  R the standard error of observation
					R = r_wheel*2pi/(500*r_round) = 0.45(deg)
	@param  Q = R * Delta_T
* @author Haokai Su
*/
void kalman_filter(int angle_observation
									,float *_Yaw
									,float *Q
									,float R
									,float *cov
									,float Delta_T)
{
	float Pre_cov = 0;
	float Pre_Yaw = 0;
	Pre_Yaw = *_Yaw + this_gyro.z * Delta_T;
	Pre_cov = *cov + *Q;
	float kal_gain = Pre_cov / (Pre_cov + R);
	*_Yaw = Pre_Yaw + kal_gain*(angle_observation - Pre_Yaw);
	*cov =(1-kal_gain)*Pre_cov;
}


