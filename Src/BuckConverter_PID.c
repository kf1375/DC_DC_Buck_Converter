#include "BuckConverter_PID.h"

////////////////////////////////////////////////////////////////////////////////
void StepMotor_Power(MotorStatusTypeDef *motor, MotorPower power)
{
	if(motor->Power != power)
	{
		if(power == Power_ON)
		{
			HAL_TIM_PWM_Start_IT(motor->timer, motor->channel);
		}
		else
		{
			HAL_TIM_PWM_Stop_IT(motor->timer, motor->channel);
		}
		motor->Power = power;
	}
}
////////////////////////////////////////////////////////////////////////////////
void StepMotor_Direction(MotorStatusTypeDef *motor, MotorDir dir)
{
	if(motor->Direction != dir)
	{
		if(dir == ClockWise)
		{
			HAL_GPIO_WritePin(motor->port, motor->pin, GPIO_PIN_RESET);
		}
		else
		{
			HAL_GPIO_WritePin(motor->port, motor->pin, GPIO_PIN_SET);
		}
		motor->Direction = dir;
	}
}
////////////////////////////////////////////////////////////////////////////////
void StepMotor_Speed(MotorStatusTypeDef *motor, uint32_t speed)
{
	uint32_t temp;
	if(motor->Speed != speed)
	{
		if(speed < motor->MinSpeed)
			temp = motor->MinSpeed;
		else if(speed > motor->MaxSpeed)
			temp = motor->MaxSpeed;
		else
			temp = speed;

		__HAL_TIM_SET_PRESCALER(motor->timer, (uint16_t)(160000 / temp));
		motor->Speed = speed;
	}
}
////////////////////////////////////////////////////////////////////////////////
void StepMotor_Move_CW(MotorStatusTypeDef *motor, uint32_t speed)
{
	StepMotor_Direction(motor , ClockWise);
	StepMotor_Speed(motor, speed);
	StepMotor_Power(motor , Power_ON);
}
////////////////////////////////////////////////////////////////////////////////
void StepMotor_Move_CCW(MotorStatusTypeDef *motor, uint32_t speed)
{
	StepMotor_Direction(motor, CounterClockWise);
	StepMotor_Speed(motor, speed);
	StepMotor_Power(motor, Power_ON);
}
////////////////////////////////////////////////////////////////////////////////
HAL_StatusTypeDef StepMotor_Goto_Position(MotorStatusTypeDef *motor, double pos, uint32_t speed)
{
	int32_t temp = 0;
	motor->SteppingMode = SET;
	temp = (int32_t)(pos * motor->GearFactor);
	if(temp < 0)
	{
		temp *= -1;
		StepMotor_Direction(motor, CounterClockWise);
	}
	else
	{
		StepMotor_Direction(motor, ClockWise);
	}
	motor->StepLimit = (uint32_t)temp;
	StepMotor_Speed(motor, speed);
	motor->Flag = RESET;
	StepMotor_Power(motor, Power_ON);
	while(motor->Flag != SET);
	motor->SteppingMode = RESET;
	return HAL_OK;
}
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
	// return constrain(ret,10,90);
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
