typedef struct {
	float kp;
	float ki;
	float kd;
	float lastActual;
	float errorSum;

	float lastD;
	float maxOutput;
	float minOutput;

	float maxIntegral;

	bool run;

} pidController;

void new_pid(pidController& pid, float kp, float ki, float kd, float max, float min) {
	pid.kp = kp;
	pid.ki = ki;
	pid.kd = kd;
	pid.run = false;
	pid.errorSum = 0;
	pid.maxOutput = max;
	pid.minOutput = min;
}

float get_output(pidController pid, float actual, float target) {
	float pOutput = 0;
	float iOutput = 0;
	float dOutput = 0;

	float error = target - actual;

	// simple P term
	pOutput = pid.kp * error;

	// we need to know if we have run before to gather
	if(!pid.run) {
		pid.lastActual = pOutput;
		pid.run = true;
	}

	dOutput = -pid.kd * (actual - pid.lastActual);
	pid.lastActual = actual;
	pid.lastD = dOutput;

	iOutput = pid.ki * (pid.errorSum);
	if(iOutput > pid.maxIntegral) {
		iOutput = pid.maxIntegral;
	} else if(iOutput < -pid.maxIntegral) {
		iOutput = -pid.maxIntegral;
	}

	float output = pOutput + iOutput + dOutput;

	if(output > pid.maxOutput) {
		output = pid.maxOutput;
	} else if(output < pid.minOutput) {
		output = pid.minOutput;
	}

	pid.errorSum += error;

	return output;

}
