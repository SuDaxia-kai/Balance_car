#include "Filter.h"
#include "math.h"

#define PI 3.1415926f

/**********************************************************************************************************
*�� �� ��: GildeAverageValueFilter
*����˵��: ȥ�����Сֵƽ���˲�
*��    ��: �˲����� ��ʷ�������� ��ʷ�������д�С
*�� �� ֵ: �˲����
**********************************************************************************************************/
float GildeAverageValueFilter(float NewValue, float *Data, uint8_t size)
{
	float max, min;
	float sum;
	unsigned char i;
	Data[0] = NewValue;
	max = Data[0];
	min = Data[0];
	sum = Data[0];
	for (i = size - 1; i != 0; i--)
	{
		if (Data[i] > max)
		max = Data[i];
		else if (Data[i] < min)
		min = Data[i];
		sum += Data[i];
		Data[i] = Data[i - 1];
	}
	i = size - 2;
	sum = sum - max - min;
	sum = sum / i;
	return (sum);
}

/**********************************************************************************************************
*�� �� ��: Butterworth_Filter
*����˵��: ������˹�˲�
*��    ��: �˲����� �˲����ڲ����� �˲�������
*�� �� ֵ: �˲����
**********************************************************************************************************/
float Butterworth_Filter(float curr_inputer, Butter_BufferData *Buffer, Butter_Parameter *Parameter)
{
	//��ȡ����x(n)
	Buffer->Input_Butter[2] = curr_inputer;
	//Butterworth�˲�
	Buffer->Output_Butter[2] =
	Parameter->b[0] * Buffer->Input_Butter[2]
		+ Parameter->b[1] * Buffer->Input_Butter[1]
		+ Parameter->b[2] * Buffer->Input_Butter[0]
		- Parameter->a[1] * Buffer->Output_Butter[1]
		- Parameter->a[2] * Buffer->Output_Butter[0];
	/* x(n) ���б��� */
	Buffer->Input_Butter[0] = Buffer->Input_Butter[1];
	Buffer->Input_Butter[1] = Buffer->Input_Butter[2];
	/* y(n) ���б��� */
	Buffer->Output_Butter[0] = Buffer->Output_Butter[1];
	Buffer->Output_Butter[1] = Buffer->Output_Butter[2];
	return (Buffer->Output_Butter[2]);
}

/**********************************************************************************************************
*�� �� ��: Set_Cutoff_Frequency
*����˵��: ��ͨ������˹��������
*��    ��: ����Ƶ�� ��ֹƵ�� �˲�������
*�� �� ֵ: �˲����
**********************************************************************************************************/
void Set_Cutoff_Frequency(float sample_frequent, float cutoff_frequent, Butter_Parameter *LPF)
{
	float fr = sample_frequent / cutoff_frequent;
	float ohm = tanf(PI / fr);
	float c = 1.0f + 2.0f * cosf(PI / 4.0f) * ohm + ohm * ohm;
	if (cutoff_frequent <= 0.0f)
		return;
	LPF->b[0] = ohm * ohm / c;
	LPF->b[1] = 2.0f * LPF->b[0];
	LPF->b[2] = LPF->b[0];
	LPF->a[0] = 1.0f;
	LPF->a[1] = 2.0f * (ohm * ohm - 1.0f) / c;
	LPF->a[2] = (1.0f - 2.0f * cosf(PI / 4.0f) * ohm + ohm * ohm) / c;
}
