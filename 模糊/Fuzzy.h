#pragma once
#ifndef PID_H
#define PID_H

typedef struct
{
	int Target;
	int Measure;
	float Kp;
	float Ki;
	float Kd;
	int Erro_sum;//»ý·Ö 
	int Erro_diff;//Î¢·Ö 
	int Erro;    	   //Error[K]
	int Erro_last;     //Error[K-1]
	int pid_output;
} PID;
PID myPID;

void Parameter_init(void);
void Parameter_Calc(PID* pp, int measure);
void lishudu(int e, int ec);
float Fuzzy_Kp();
float Fuzzy_Ki();
float Fuzzy_Kd();

#endif



