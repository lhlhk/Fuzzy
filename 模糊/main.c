#define _CRT_SECURE_NO_WARNINGS 1
#include"stdio.h"
#include"math.h"
#include"Fuzzy.h"

int main(void)
{
	/*FILE* input, * Kp, * Ki, * Kd;*/

	//printf("\n����һ��ģ��PID�������̣�������õ����ģ��PID���ƹ���\n");

	Parameter_init();//����������ʼ��
	//printf("\n��ʼ�����趨\n");
	//printf("Target��%d\n", myPID.Target);
	//printf("Meature��%d\n", myPID.Measure);
	//printf("Kp��%f\n", myPID.Kp);
	//printf("Ki��%f\n", myPID.Ki);
	//printf("Kd��%f\n", myPID.Kd);

	//input = fopen("input.txt", "r");
	//Kp = fopen("Kp.txt", "w");
	//Ki = fopen("Ki.txt", "w");
	//Kd = fopen("Kd.txt", "w");

	printf("\n��ʼ����...\n");
	for(int i = 0;i < 100;i++)
	{
		//fscanf(input, "%d", &myPID.Measure);
		Parameter_Calc(&myPID, myPID.Measure);
		printf("%d    %d\n", myPID.pid_output,myPID.Measure);
		//fprintf(Kp, "%f		 ", myPID.Kp);
		//fprintf(Ki, "%f		 ", myPID.Ki);
		//fprintf(Kd, "%f		 ", myPID.Kd);
	}
	//fclose(Kp);
	//fclose(Ki);
	//fclose(Kd);
	printf("\n���������(��������ļ��в鿴���)\n");
	return 0;
}

