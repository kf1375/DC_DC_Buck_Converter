#include "BuckConverter_PID.h"

////////////////////////////////////////////////////////////////////////////////
void PID_Init(PID_TypeDef *pid, double p, double i, double d)
{
	pid->P_COE = p;
	pid->I_COE = i;
	pid->D_COE = d;
	pid->Integral = 0;
	pid->Derivative = 0;
	pid->PrevError = 0;
}
////////////////////////////////////////////////////////////////////////////////
void PID_Reset(PID_TypeDef *pid)
{
	pid->Integral = 0;
}
////////////////////////////////////////////////////////////////////////////////
double PID_Compute(PID_TypeDef *pid, double current_pos, double desierd_pos)
{
	double error = 0;
	double ret = 0;
	error = desierd_pos-current_pos  ;
	pid->Integral += error;
	pid->Derivative = error - pid->PrevError;
	pid->PrevError = error;
	ret = (pid->P_COE * error) + (pid->I_COE * pid->Integral) - (pid->D_COE * pid->Derivative);
	return ret;
}
////////////////////////////////////////////////////////////////////////////////
double P2ID_Compute(PID_TypeDef *pid, double current_pos, double desierd_pos)
{
	double error = 0;
	double ret = 0;
	error = current_pos - desierd_pos;
	pid->Integral += error;
	pid->Derivative = error - pid->PrevError;
	pid->PrevError = error;
	if(error < 0)
		error = -1 * error * error;
	else
		error = error * error;
	ret = (pid->P_COE * error) + (pid->I_COE * pid->Integral) - (pid->D_COE * pid->Derivative);
	return ret;
}
////////////////////////////////////////////////////////////////////////////////
