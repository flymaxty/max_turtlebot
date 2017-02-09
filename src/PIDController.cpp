#include "PIDController.h"

PIDController::PIDController()
{}
void PIDController::PIDControllerInitialize(float KpValue = 1, float TiValue = 0, float TdValue = 0, float samplingTimeValue = 1)
{
	Kp = KpValue;
	Ti = TiValue;
	Td = TdValue;
	samplingTime = samplingTimeValue;
	d0 = Kp*(1.0 + samplingTime / Ti + Td / samplingTime);
	d1 = -Kp*(1.0 + 2.0*Td / samplingTime);
	d2 = Kp*Td / samplingTime;
	error0 = 0;
	error1 = 0;
	error2 = 0;
	output = 0;
}
float PIDController::PIDControlOutput(float errorNew)
{
	error0 = error1;
	error1 = error2;
	error2 = errorNew;
	output = output + d0*error2 + d1*error1 + d2*error0;
	return output;
}