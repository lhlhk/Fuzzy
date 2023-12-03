#include"stdio.h"
#include"math.h"
#include"Fuzzy.h"

/*�����U����ֵ������*/
float UFF_P[7] = { 0,2,4,6,8,10,12 };
float UFF_I[7] = { 0.2,0.4,0.8,1.5,2.0,2.5 };
float UFF_D[7] = { 0,1,2,3,4,5,6 };

//p�����
unsigned int rule_p[7][7] = {
	//���仯�� -3,-2,-1, 0, 1, 2, 3     // ���     
				{6, 5, 4, 3, 2, 0, 0,},   //   -3   
				{5, 4, 3, 2, 1, 0, 1,},   //   -2 
				{4, 3, 2, 1, 0, 1, 2,},   //   -1 
				{3, 2, 1, 0, 1, 2, 3,},   //    0 
				{2, 1, 0, 1, 2, 3, 4,},   //    1 
				{1, 0, 1, 2, 3, 4, 5,},   //    2 
				{0, 0, 2, 3, 4, 5, 6} };   //    3 

//i�����
unsigned int rule_i[7][7] = {
	//���仯�� -3,-2,-1, 0, 1, 2, 3     // ���     
				{6, 6, 4, 3, 1, 0, 0,},   //   -3   
				{6, 5, 4, 2, 1, 0, 0,},   //   -2 
				{5, 4, 2, 1, 0, 1, 2,},   //   -1 
				{4, 3, 1, 0, 1, 3, 4,},   //    0 
				{2, 1, 0, 1, 2, 4, 5,},   //    1 
				{0, 0, 1, 2, 4, 5, 6,},   //    2 
				{0, 0, 1, 3, 4, 6, 6} };   //    3 

//d�����
unsigned int rule_d[7][7] = {
	//���仯�� -3,-2,-1, 0, 1, 2, 3    // ���     
				{2, 2, 6, 5, 6, 4, 2,},   //   -3   
				{1, 2, 5, 4, 3, 1, 0,},   //   -2 
				{0, 1, 3, 3, 1, 1, 0,},   //   -1 
				{0, 1, 1, 1, 1, 1, 0,},   //    0 
				{0, 0, 0, 0, 0, 0, 0,},   //    1 
				{5, 1, 1, 1, 1, 1, 1,},   //    2 
				{6, 4, 4, 3, 3, 1, 1} };   //    3 

//�仯��Χ�� 
int PFF[4] = { 0,100,300,600 };
int DFF[4] = { 0,100,300,600 };

void Parameter_init()
{
	myPID.Target = 50;
	myPID.Measure = 0;
	myPID.Kp = 6;    //����         
	myPID.Ki = 2;    //����          
	myPID.Kd = 4;    //΢��  
	myPID.Erro_last = 0;   //Error[K-1]
	myPID.Erro_sum = 0;
}

void Parameter_Calc(PID* param, int measure)
{
	measure += param->pid_output;
	param->Measure = measure;

	//λ��ʽPID 
	param->Erro = param->Target - param->Measure; //ƫ��
	param->Erro_sum += param->Erro; //����
	param->Erro_diff = param->Erro - param->Erro_last; //΢��

	lishudu(param->Erro, param->Erro_diff);
	param->Kp = Fuzzy_Kp();    //ģ��������P
	param->Ki = Fuzzy_Ki();    //ģ��������I
	param->Kd = Fuzzy_Kd();    //ģ��������D	

	param->pid_output = param->Kp * param->Erro + param->Ki * param->Erro_sum + param->Kd * param->Erro_diff;

	param->Erro_last = param->Erro;          //Error[K-1] 	
}

int   PF[2], DF[2];   //ƫ��,ƫ��΢��������
int   Pn, Dn;   //�Ǳ�
const int FMAX = 1000;    //����ֵ������ֵ
//ģ���� 
void lishudu(int e, int ec)
{
	/*����E Ec��ָ������ֵ�����Ч������*/
	//Ѱ��e�������� 
	if (e > -PFF[3] && e < PFF[3])//E�ı仯�ڷ�ֵ�� 
	{
		if (e <= -PFF[2])
		{
			Pn = -2;
			PF[0] = FMAX * ((float)(-PFF[2] - e) / (PFF[3] - PFF[2]));
		}
		else if (e <= -PFF[1])
		{
			Pn = -1;
			PF[0] = FMAX * ((float)(-PFF[1] - e) / (PFF[2] - PFF[1]));
		}
		else if (e <= PFF[0])
		{
			Pn = 0;
			PF[0] = FMAX * ((float)(-PFF[0] - e) / (PFF[1] - PFF[0]));
		}
		else if (e <= PFF[1])
		{
			Pn = 1;
			PF[0] = FMAX * ((float)(PFF[1] - e) / (PFF[1] - PFF[0]));
		}
		else if (e <= PFF[2])
		{
			Pn = 2;
			PF[0] = FMAX * ((float)(PFF[2] - e) / (PFF[2] - PFF[1]));
		}
		else if (e <= PFF[3])
		{
			Pn = 3;
			PF[0] = FMAX * ((float)(PFF[3] - e) / (PFF[3] - PFF[2]));
		}
	}
	else if (e <= -PFF[3])  //�޷� 
	{
		Pn = -2;
		PF[0] = FMAX;
	}
	else if (e >= PFF[3])
	{
		Pn = 3;
		PF[0] = 0;
	}
	PF[1] = FMAX - PF[0];

	if (ec > -DFF[3] && ec < DFF[3])
	{
		if (ec <= -DFF[2])
		{
			Dn = -2;
			DF[0] = FMAX * ((float)(-DFF[2] - ec) / (DFF[3] - DFF[2]));
		}
		else if (ec <= -DFF[1])
		{
			Dn = -1;
			DF[0] = FMAX * ((float)(-DFF[1] - ec) / (DFF[2] - DFF[1]));
		}
		else if (ec <= DFF[0])
		{
			Dn = 0;
			DF[0] = FMAX * ((float)(-DFF[0] - ec) / (DFF[1] - DFF[0]));
		}
		else if (ec <= DFF[1])
		{
			Dn = 1;
			DF[0] = FMAX * ((float)(DFF[1] - ec) / (DFF[1] - DFF[0]));
		}
		else if (ec <= DFF[2])
		{
			Dn = 2;
			DF[0] = FMAX * ((float)(DFF[2] - ec) / (DFF[2] - DFF[1]));
		}
		else if (ec <= DFF[3])
		{
			Dn = 3;
			DF[0] = FMAX * ((float)(DFF[3] - ec) / (DFF[3] - DFF[2]));
		}
	}
	else if (ec <= -DFF[3])
	{
		Dn = -2;
		DF[0] = FMAX;
	}
	else if (ec >= DFF[3])
	{
		Dn = 3;
		DF[0] = 0;
	}
	DF[1] = FMAX - DF[0];
}

//Kpģ������
float Fuzzy_Kp()
{
	float out = 0;  //���ֵ�ľ�ȷ�� 
	int Un[4];//��Ӧ����Pֵ 
	float Un_out[4];
	unsigned int UF[4]; //������
	int temp1, temp2;

	/*ʹ����Χ�Ż���Ĺ����rule[7][7]*/
	/*һ�㶼���ĸ�������Ч*/
	Un[0] = rule_p[Pn + 2][Dn + 2];
	Un[1] = rule_p[Pn + 3][Dn + 2];
	Un[2] = rule_p[Pn + 2][Dn + 3];
	Un[3] = rule_p[Pn + 3][Dn + 3];

	if (PF[0] <= DF[0])
		UF[0] = PF[0];
	else
		UF[0] = DF[0];

	if (PF[1] <= DF[0])
		UF[1] = PF[1];
	else
		UF[1] = DF[0];

	if (PF[0] <= DF[1])
		UF[2] = PF[0];
	else
		UF[2] = DF[1];

	if (PF[1] <= DF[1])
		UF[3] = PF[1];
	else
		UF[3] = DF[1];


	/*ͬ���������������ֵ���*/
	if (Un[0] == Un[1])
	{
		if (UF[0] > UF[1])
			UF[1] = 0;
		else
			UF[0] = 0;
	}

	if (Un[0] == Un[2])
	{
		if (UF[0] > UF[2])
			UF[2] = 0;
		else
			UF[0] = 0;
	}

	if (Un[0] == Un[3])
	{
		if (UF[0] > UF[3])
			UF[3] = 0;
		else
			UF[0] = 0;
	}

	if (Un[1] == Un[2])
	{
		if (UF[1] > UF[2])
			UF[2] = 0;
		else
			UF[1] = 0;
	}

	if (Un[1] == Un[3])
	{
		if (UF[1] > UF[3])
			UF[3] = 0;
		else
			UF[1] = 0;
	}

	if (Un[2] == Un[3])
	{
		if (UF[2] > UF[3])
			UF[3] = 0;
		else
			UF[2] = 0;
	}

	/*���ķ���ģ��*/
	/*Un[]ԭֵΪ�������������ţ�ת��Ϊ��������ֵ*/
	if (Un[0] >= 0)
		Un_out[0] = UFF_P[Un[0]];
	else
		Un_out[0] = -UFF_P[-Un[0]];

	if (Un[1] >= 0)
		Un_out[1] = UFF_P[Un[1]];
	else
		Un_out[1] = -UFF_P[-Un[1]];

	if (Un[2] >= 0)
		Un_out[2] = UFF_P[Un[2]];
	else
		Un_out[2] = -UFF_P[-Un[2]];

	if (Un[3] >= 0)
		Un_out[3] = UFF_P[Un[3]];
	else
		Un_out[3] = -UFF_P[-Un[3]];


	temp1 = UF[0] * Un_out[0] + UF[1] * Un_out[1] + UF[2] * Un_out[2] + UF[3] * Un_out[3];
	temp2 = UF[0] + UF[1] + UF[2] + UF[3];
	out = (float)temp1 / temp2;

	return out;
}

//Kiģ������
float  Fuzzy_Ki()
{
	float out;  //���ֵ�ľ�ȷ�� 
	int Un[4];
	float Un_out[4];
	unsigned int  UF[4]; //������  
	int   temp1, temp2;

	/*ʹ����Χ�Ż���Ĺ����rule[7][7]*/
	/*һ�㶼���ĸ�������Ч*/
	Un[0] = rule_i[Pn + 2][Dn + 2];
	Un[1] = rule_i[Pn + 3][Dn + 2];
	Un[2] = rule_i[Pn + 2][Dn + 3];
	Un[3] = rule_i[Pn + 3][Dn + 3];

	if (PF[0] <= DF[0])
		UF[0] = PF[0];
	else
		UF[0] = DF[0];
	if (PF[1] <= DF[0])
		UF[1] = PF[1];
	else
		UF[1] = DF[0];
	if (PF[0] <= DF[1])
		UF[2] = PF[0];
	else
		UF[2] = DF[1];
	if (PF[1] <= DF[1])
		UF[3] = PF[1];
	else
		UF[3] = DF[1];

	/*ͬ���������������ֵ���*/

	if (Un[0] == Un[1])
	{
		if (UF[0] > UF[1])
			UF[1] = 0;
		else
			UF[0] = 0;
	}
	if (Un[0] == Un[2])
	{
		if (UF[0] > UF[2])
			UF[2] = 0;
		else
			UF[0] = 0;
	}
	if (Un[0] == Un[3])
	{
		if (UF[0] > UF[3])
			UF[3] = 0;
		else
			UF[0] = 0;
	}
	if (Un[1] == Un[2])
	{
		if (UF[1] > UF[2])

			UF[2] = 0;
		else
			UF[1] = 0;
	}
	if (Un[1] == Un[3])
	{
		if (UF[1] > UF[3])
			UF[3] = 0;
		else
			UF[1] = 0;
	}
	if (Un[2] == Un[3])
	{
		if (UF[2] > UF[3])
			UF[3] = 0;
		else
			UF[2] = 0;
	}

	/*���ķ���ģ��*/
	/*Un[]ԭֵΪ�������������ţ�ת��Ϊ��������ֵ*/
	if (Un[0] >= 0)
		Un_out[0] = UFF_I[Un[0]];
	else
		Un_out[0] = -UFF_I[-Un[0]];
	if (Un[1] >= 0)
		Un_out[1] = UFF_I[Un[1]];
	else
		Un_out[1] = -UFF_I[-Un[1]];
	if (Un[2] >= 0)
		Un_out[2] = UFF_I[Un[2]];
	else
		Un_out[2] = -UFF_I[-Un[2]];
	if (Un[3] >= 0)
		Un_out[3] = UFF_I[Un[3]];
	else
		Un_out[3] = -UFF_I[-Un[3]];

	temp1 = UF[0] * Un_out[0] + UF[1] * Un_out[1] + UF[2] * Un_out[2] + UF[3] * Un_out[3];
	temp2 = UF[0] + UF[1] + UF[2] + UF[3];
	out = (float)temp1 / temp2;

	return out;
}

//Kdģ������
float Fuzzy_Kd()
{
	float out;  //���ֵ�ľ�ȷ�� 
	int Un[4]; //��������ȱ��
	float Un_out[4];
	unsigned int   UF[4]; //������  
	int   temp1, temp2;

	/*ʹ����Χ�Ż���Ĺ����rule[7][7]*/
	/*һ�㶼���ĸ�������Ч*/
	Un[0] = rule_d[Pn + 2][Dn + 2];
	Un[1] = rule_d[Pn + 3][Dn + 2];
	Un[2] = rule_d[Pn + 2][Dn + 3];
	Un[3] = rule_d[Pn + 3][Dn + 3];

	if (PF[0] <= DF[0])
		UF[0] = PF[0];
	else
		UF[0] = DF[0];
	if (PF[1] <= DF[0])
		UF[1] = PF[1];
	else
		UF[1] = DF[0];
	if (PF[0] <= DF[1])
		UF[2] = PF[0];
	else
		UF[2] = DF[1];
	if (PF[1] <= DF[1])
		UF[3] = PF[1];
	else
		UF[3] = DF[1];

	/*ͬ���������������ֵ���*/

	if (Un[0] == Un[1])
	{
		if (UF[0] > UF[1])
			UF[1] = 0;
		else
			UF[0] = 0;
	}
	if (Un[0] == Un[2])
	{
		if (UF[0] > UF[2])
			UF[2] = 0;
		else
			UF[0] = 0;
	}
	if (Un[0] == Un[3])
	{
		if (UF[0] > UF[3])
			UF[3] = 0;
		else
			UF[0] = 0;
	}
	if (Un[1] == Un[2])
	{
		if (UF[1] > UF[2])

			UF[2] = 0;
		else
			UF[1] = 0;
	}
	if (Un[1] == Un[3])
	{
		if (UF[1] > UF[3])
			UF[3] = 0;
		else
			UF[1] = 0;
	}
	if (Un[2] == Un[3])
	{
		if (UF[2] > UF[3])
			UF[3] = 0;
		else
			UF[2] = 0;
	}

	/*���ķ���ģ��*/
	/*Un[]ԭֵΪ�������������ţ�ת��Ϊ��������ֵ*/
	if (Un[0] >= 0)
		Un_out[0] = UFF_D[Un[0]];
	else
		Un_out[0] = -UFF_D[-Un[0]];
	if (Un[1] >= 0)
		Un_out[1] = UFF_D[Un[1]];
	else
		Un_out[1] = -UFF_D[-Un[1]];
	if (Un[2] >= 0)
		Un_out[2] = UFF_D[Un[2]];
	else
		Un_out[2] = -UFF_D[-Un[2]];
	if (Un[3] >= 0)
		Un_out[3] = UFF_D[Un[3]];
	else
		Un_out[3] = -UFF_D[-Un[3]];

	temp1 = UF[0] * Un_out[0] + UF[1] * Un_out[1] + UF[2] * Un_out[2] + UF[3] * Un_out[3];
	temp2 = UF[0] + UF[1] + UF[2] + UF[3];
	out = (float)temp1 / temp2;

	return out;
}

