#define _CRT_SECURE_NO_WARNINGS 1
#include"stdio.h"
#include"math.h"
#include"Fuzzy.h"

int main(void)
{
	/*FILE* input, * Kp, * Ki, * Kd;*/

	//printf("\n这是一个模糊PID控制例程，方便更好地理解模糊PID控制过程\n");

	Parameter_init();//整定参数初始化
	//printf("\n初始参数设定\n");
	//printf("Target：%d\n", myPID.Target);
	//printf("Meature：%d\n", myPID.Measure);
	//printf("Kp：%f\n", myPID.Kp);
	//printf("Ki：%f\n", myPID.Ki);
	//printf("Kd：%f\n", myPID.Kd);

	//input = fopen("input.txt", "r");
	//Kp = fopen("Kp.txt", "w");
	//Ki = fopen("Ki.txt", "w");
	//Kd = fopen("Kd.txt", "w");

	printf("\n开始运算...\n");
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
	printf("\n运算结束！(请在输出文件中查看结果)\n");
	return 0;
}

