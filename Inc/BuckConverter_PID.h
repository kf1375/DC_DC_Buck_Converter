#ifndef __BuckConverter_PID_H__
#define __BuckConverter_PID_H__

#include "stm32f1xx_hal.h"
#include "math.h"

#define STEPMOTOR_DEFAULT_FREQUENCY  200
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
typedef enum 
{
	ClockWise = 0,
	CounterClockWise = 1
}MotorDir;

typedef enum 
{
	Power_OFF = 0,
	Power_ON = 1
}MotorPower;

typedef struct
{
	MotorPower Power;
	MotorDir Direction;
	uint32_t Speed;
	uint32_t MaxSpeed;
	uint32_t MinSpeed;
	uint32_t StepCounter;
	uint32_t StepLimit;
	volatile FlagStatus Flag;
	volatile FlagStatus SteppingMode;
	TIM_HandleTypeDef *timer;
	uint32_t channel;
	GPIO_TypeDef *port;
	uint16_t pin;
	double GearFactor;
}MotorStatusTypeDef;

typedef struct
{
	double P_COE;
	double I_COE;
	double D_COE;
	double Integral;
	double Derivative;
	double PrevError;
}PID_TypeDef;

void StepMotor_Init(MotorStatusTypeDef *motor, TIM_HandleTypeDef *tim, uint32_t channel,
GPIO_TypeDef *port, uint16_t pin, double gear_factor, uint32_t max_speed, uint32_t min_speed);
void StepMotor_Power(MotorStatusTypeDef *motor, MotorPower power);
void StepMotor_Direction(MotorStatusTypeDef *motor, MotorDir dir);
void StepMotor_Speed(MotorStatusTypeDef *motor, uint32_t speed);
void StepMotor_Move_CW(MotorStatusTypeDef *motor, uint32_t speed);
void StepMotor_Move_CCW(MotorStatusTypeDef *motor, uint32_t speed);
HAL_StatusTypeDef StepMotor_Goto_Position(MotorStatusTypeDef *motor, double pos, uint32_t speed);
void PID_Init(PID_TypeDef *pid, double p, double i, double d);
void PID_Reset(PID_TypeDef *pid);
double PID_Compute(PID_TypeDef *pid, double current_pos, double desierd_pos);
double P2ID_Compute(PID_TypeDef *pid, double current_pos, double desierd_pos);
#endif /*__BuckConverter_PID_H__*/
