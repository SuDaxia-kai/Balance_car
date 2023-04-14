#include "ahrs.h"
#include "math.h"
#include "time_cnt.h"
#include "imu.h"
#include "Filter.h"
#include "stdio.h"

typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;
} Vector4q;

#define PI 3.1415926f
#define DEG2RAD (PI / 180.0f)
#define RAD2DEG (180.0f / PI)
//时延控制
#define TimeSync_Cnt 9 + 1
//保存历史个数
#define Quad_Num  20
//动态步长贝塔
#define Beta_Base 0.0075f
#define Yaw_Fusion_Beta 0.025f

//四元数计算出的角度
float Yaw, Pitch, Roll;
float Sin_Pitch, Sin_Roll, Sin_Yaw;
float Cos_Pitch, Cos_Roll, Cos_Yaw;

//转换矩阵
static volatile float rMat[3][3];

static Butter_Parameter Butter_5HZ_Parameter;
static float Kp = 0, Ki = 0;
//四元数历史值
static Vector4q quad_history[Quad_Num];
//本次四元数值
static Vector4q this_quad;

/**********************************************************************************************************
*函 数 名: invSqrt
*功能说明: 求1/sqrt(x)
*形    参: x
*返 回 值: 1/sqrt(x)
**********************************************************************************************************/
float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;

	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));

	return y;
}

/**********************************************************************************************************
*函 数 名: ahrs_init
*功能说明: 姿态解算初始化四元数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ahrs_init(void)
{
	uint16_t i = 0;
	this_quad.q0 = 1;
	this_quad.q1 = 0;
	this_quad.q2 = 0;
	this_quad.q3 = 0;

	for (i = Quad_Num - 1; i > 0; i--) {
		quad_history[i] = this_quad;
	}
	quad_history[0] = this_quad;
	printf(" I have finish the init of ahrs \r\n" );
}

/**********************************************************************************************************
*函 数 名: constrain
*功能说明: 限定value的最大值和最小值
*形    参: value
*返 回 值: 修正后的value
**********************************************************************************************************/
float constrain(float value, const float min_val, const float max_val)
{
    if (value >= max_val)
        value = max_val;
    if (value <= min_val)
        value = min_val;
    return value;
}

/**********************************************************************************************************
*函 数 名: ComputeRotationMatrix
*功能说明: 更新坐标系变换矩阵
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ComputeRotationMatrix(void)
{
    Sin_Pitch = sin(Pitch * DEG2RAD);
    Cos_Pitch = cos(Pitch * DEG2RAD);
    Sin_Roll = sin(Roll * DEG2RAD);
    Cos_Roll = cos(Roll * DEG2RAD);
    Sin_Yaw = sin(Yaw * DEG2RAD);
    Cos_Yaw = cos(Yaw * DEG2RAD);
    rMat[0][0] = Cos_Yaw * Cos_Roll;
    rMat[0][1] = Sin_Pitch * Sin_Roll * Cos_Yaw - Cos_Pitch * Sin_Yaw;
    rMat[0][2] = Sin_Pitch * Sin_Yaw + Cos_Pitch * Sin_Roll * Cos_Yaw;
    rMat[1][0] = Sin_Yaw * Cos_Roll;
    rMat[1][1] = Sin_Pitch * Sin_Roll * Sin_Yaw + Cos_Pitch * Cos_Yaw;
    rMat[1][2] = Cos_Pitch * Sin_Roll * Sin_Yaw - Sin_Pitch * Cos_Yaw;
    rMat[2][0] = -Sin_Roll;
    rMat[2][1] = Sin_Pitch * Cos_Roll;
    rMat[2][2] = Cos_Pitch * Cos_Roll;
//		printf(" finish the R2M \r\n");
}
/**********************************************************************************************************
*函 数 名: ahrs_update
*功能说明: 姿态解算
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ahrs_update()
{
	static uint16_t sync_cnt = 0;
	//上次次陀螺仪角速度
	static Vector3f_t gyro_history[Quad_Num];
	//这次陀螺仪角速度
	static Vector3f_t this_gyro;
	//加速度历史值
	static Vector3f_t accel_history[Quad_Num];
	//本次加速度
	static Vector3f_t this_accel;
	//误差积分
	static float exInt = 0, eyInt = 0, ezInt = 0;
	static float vx, vy, vz;
	//角速度模长滤波内部数据
	Butter_BufferData Butter_Buffer_Gyro_Length;
	uint16_t i;
	//用于计算时间差的结构体
	static Testime Time_Delta;
	//时间差
	float dt;
	//角速度模长
	float Gyro_Length;
	float Gyro_Length_Filter;
	//归一化加速度
	Vector3f_t recip_accel;
	//模长
	float recipNorm;
	//梯度下降步长
	float BETADEF;
	//误差
	float ex = 0, ey = 0, ez = 0;
	//梯度下降算子求出来的姿态    
	float s0, s1, s2, s3;
	// 四元数微分方程求得的姿态
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
	static Vector3f_t gyro_tmp;
	float delta;
	
	//更新计算时间差
	Get_Time_Period(&Time_Delta);
	dt = Time_Delta.Time_Delta / 1000000.0;
	if (dt == 0)
		return;
	
	sync_cnt++;
	//4*5=20ms滑动一次
	if (sync_cnt >= 4) {
		//将四元数历史值保存起来,20*20=400ms
		for (i = Quad_Num - 1; i > 0; i--) {
			quad_history[i] = quad_history[i - 1];
		}
		quad_history[0] = this_quad;
		
		//将加速度历史值保存起来,20*20=400ms
		accel_history[0] = this_accel;
		for (i = Quad_Num - 1; i > 0; i--) {
			accel_history[i] = accel_history[i - 1];
		}
		sync_cnt = 0;
	}


	this_accel.x = acceCorrectFilter.x;
	this_accel.y = acceCorrectFilter.y;
	this_accel.z = acceCorrectFilter.z;

	//保存陀螺仪值
	for (i = Quad_Num; i > 0; i--) {
		gyro_history[i] = gyro_history[i - 1];
	}
	gyro_history[0] = this_gyro;
	this_gyro.x = gyroDataFilter.x * GYRO_CALIBRATION_COFF;
	this_gyro.y = gyroDataFilter.y * GYRO_CALIBRATION_COFF;
	this_gyro.z = gyroDataFilter.z * GYRO_CALIBRATION_COFF;
//	printf("{4quad:%f,%f,%f}\r\n",this_gyro.x,this_gyro.y,this_gyro.z);
	
	//角速度模长
	Gyro_Length = sqrt(this_gyro.x * this_gyro.x + this_gyro.y * this_gyro.y + this_gyro.z * this_gyro.z);
	Gyro_Length_Filter = Gyro_Length;

	//Butterworth_Filter(Gyro_Length, &Butter_Buffer_Gyro_Length, &Butter_5HZ_Parameter);

	//取加速度，并归一化
	recip_accel = accel_history[9];
	recipNorm = invSqrt(recip_accel.x * recip_accel.x + recip_accel.y * recip_accel.y + recip_accel.z * recip_accel.z);
	recip_accel.x *= recipNorm;
	recip_accel.y *= recipNorm;
	recip_accel.z *= recipNorm;

	/* 避免重复运算 */
	_2q0 = 2.0f * quad_history[TimeSync_Cnt].q0;
	_2q1 = 2.0f * quad_history[TimeSync_Cnt].q1;
	_2q2 = 2.0f * quad_history[TimeSync_Cnt].q2;
	_2q3 = 2.0f * quad_history[TimeSync_Cnt].q3;
	_4q0 = 4.0f * quad_history[TimeSync_Cnt].q0;
	_4q1 = 4.0f * quad_history[TimeSync_Cnt].q1;
	_4q2 = 4.0f * quad_history[TimeSync_Cnt].q2;
	_8q1 = 8.0f * quad_history[TimeSync_Cnt].q1;
	_8q2 = 8.0f * quad_history[TimeSync_Cnt].q2;
	q0q0 = quad_history[TimeSync_Cnt].q0 * quad_history[TimeSync_Cnt].q0;
	q1q1 = quad_history[TimeSync_Cnt].q1 * quad_history[TimeSync_Cnt].q1;
	q2q2 = quad_history[TimeSync_Cnt].q2 * quad_history[TimeSync_Cnt].q2;
	q3q3 = quad_history[TimeSync_Cnt].q3 * quad_history[TimeSync_Cnt].q3;

	//梯度下降算法,计算误差函数的梯度
	s0 = _4q0 * q2q2 + _2q2 * recip_accel.x + _4q0 * q1q1 - _2q1 * recip_accel.y;
	s1 = _4q1 * q3q3 - _2q3 * recip_accel.x + 4.0f * q0q0 * quad_history[TimeSync_Cnt].q1 - _2q0 * recip_accel.y - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * recip_accel.z;
	s2 = 4.0f * q0q0 * quad_history[TimeSync_Cnt].q2 + _2q0 * recip_accel.x + _4q2 * q3q3 - _2q3 * recip_accel.y - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * recip_accel.z;
	s3 = 4.0f * q1q1 * quad_history[TimeSync_Cnt].q3 - _2q1 * recip_accel.x + 4.0f * q2q2 * quad_history[TimeSync_Cnt].q2 - _2q2 * recip_accel.y;
	
    /* 梯度归一化 */
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

	//计算动态步长
    BETADEF = Beta_Base + 0.05f * dt * constrain(Gyro_Length_Filter, 0, 400);
	//BETADEF -= 0.01 * (constrain(navigation_acce_length, 0, 1000) / 1000); 
    BETADEF = constrain(BETADEF, 0.015, 0.12);

    vx = 2 * (quad_history[TimeSync_Cnt].q1 * quad_history[TimeSync_Cnt].q3 - quad_history[TimeSync_Cnt].q0 * quad_history[TimeSync_Cnt].q2);
    vy = 2 * (quad_history[TimeSync_Cnt].q2 * quad_history[TimeSync_Cnt].q3 + quad_history[TimeSync_Cnt].q0 * quad_history[TimeSync_Cnt].q1);
    vz = 1 - 2 * (quad_history[TimeSync_Cnt].q1 * quad_history[TimeSync_Cnt].q1 + quad_history[TimeSync_Cnt].q2 * quad_history[TimeSync_Cnt].q2);

    ex = (recip_accel.y * vz - recip_accel.z * vy);
    ey = (recip_accel.z * vx - recip_accel.x * vz);
    ez = (recip_accel.x * vy - recip_accel.y * vx);
    exInt += ex * Ki * dt;
    eyInt += ey * Ki * dt;
    ezInt += ez * Ki * dt;

	/* 转换为弧度制，用于姿态更新*/
	gyro_tmp.x = this_gyro.x * PI / 180 + exInt + Kp * ex;
	gyro_tmp.y = this_gyro.y * PI / 180 + eyInt + Kp * ey;
	gyro_tmp.z = this_gyro.z * PI / 180 + ezInt + Kp * ez;

	/* 四元数微分方程计算本次待矫正四元数 */
	qDot1 = 0.5f * (-this_quad.q1 * gyro_tmp.x - this_quad.q2 * gyro_tmp.y - this_quad.q3 * gyro_tmp.z);
	qDot2 = 0.5f * (this_quad.q0 * gyro_tmp.x + this_quad.q2 * gyro_tmp.z - this_quad.q3 * gyro_tmp.y);
	qDot3 = 0.5f * (this_quad.q0 * gyro_tmp.y - this_quad.q1 * gyro_tmp.z + this_quad.q3 * gyro_tmp.x);
	qDot4 = 0.5f * (this_quad.q0 * gyro_tmp.z + this_quad.q1 * gyro_tmp.y - this_quad.q2 * gyro_tmp.x);

	qDot1 -= BETADEF * s0;
	qDot2 -= BETADEF * s1;
	qDot3 -= BETADEF * s2;
	qDot4 -= BETADEF * s3;
	

	
	//补偿由四元数微分方程引入的姿态误差
	//将四元数姿态导数积分,得到当前四元数姿态
	//二阶毕卡求解微分方程
	delta = (dt * gyro_tmp.x) * (dt * gyro_tmp.x) + (dt * gyro_tmp.y) * (dt * gyro_tmp.y) + (dt* gyro_tmp.z) * (dt * gyro_tmp.z);
	this_quad.q0 = (1.0f - delta / 8.0f) * this_quad.q0 + qDot1 * dt;
	this_quad.q1 = (1.0f - delta / 8.0f) * this_quad.q1 + qDot2 * dt;
	this_quad.q2 = (1.0f - delta / 8.0f) * this_quad.q2 + qDot3 * dt;
	this_quad.q3 = (1.0f - delta / 8.0f) * this_quad.q3 + qDot4 * dt;
	//单位化四元数
	recipNorm = invSqrt(this_quad.q0 * this_quad.q0 + this_quad.q1 * this_quad.q1 + this_quad.q2 * this_quad.q2 + this_quad.q3 * this_quad.q3);
	this_quad.q0 *= recipNorm;
	this_quad.q1 *= recipNorm;
	this_quad.q2 *= recipNorm;
	this_quad.q3 *= recipNorm;
	
	//四元数到欧拉角转换,转换顺序为Z-Y-X,参见<Representing Attitude: Euler Angles, Unit Quaternions, and Rotation Vectors>.pdf一文,P24 */
	Pitch = atan2(2.0f * this_quad.q2 * this_quad.q3 + 2.0f * this_quad.q0 * this_quad.q1, -2.0f * this_quad.q1 * this_quad.q1 - 2.0f * this_quad.q2 * this_quad.q2 + 1.0f) * RAD2DEG;
	Roll = asin(2.0f * this_quad.q0 * this_quad.q2 - 2.0f * this_quad.q1 * this_quad.q3) * RAD2DEG;
	Yaw = atan2(2.0f * this_quad.q1 * this_quad.q2 + 2.0f * this_quad.q0 * this_quad.q3, -2.0f * this_quad.q3 * this_quad.q3 - 2.0f * this_quad.q2 * this_quad.q2 + 1.0f) * RAD2DEG;
	printf("{Gyro:%f,%f,%f}\r\n",Yaw,Pitch,Roll);
	ComputeRotationMatrix();
}

