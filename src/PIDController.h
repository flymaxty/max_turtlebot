#ifndef  PIDController_h
#define PIDController_h
class PIDController
{
	private:
		float Kp ;
		float Ti ;
		float Td;
		float samplingTime;//√Î
		float d0;
		float d1;
		float d2;
		float error0;
		float error1;
		float error2;
		float output;
	public:
		PIDController();
		void PIDControllerInitialize(float KpValue, float TiValue, float TdValue, float samplingTime);
		float PIDControlOutput(float errorNew);
};
#endif