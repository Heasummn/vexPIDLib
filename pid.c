typedef struct {
	float kp;
	float ki;
	float kd;
	float lastActual;
	float errorSum;

	bool run;

} pidController;

void new_pid(pidController& pid, float kp, float ki, float kd) {
	pid.kp = kp;
	pid.ki = ki;
	pid.kd = kd;
	pid.run = false;
	pid.errorSum = 0;
}


// WIP
void get_output(pidController pid, float actual, float target) {
	float pOutput = 0;
	float iOutput = 0;
	float dOutput = 0;

	float error = actual - target;

	// simple P term
	pOutput = pid.kp * error;

	// we need to know if we have run before to gather
	if(!pid.run) {
		pid.lastActual = pOutput;
		pid.run = true;
	}

	dOutput = -pid.kd * (actual - pid.lastActual);
	pid.lastActual = actual;

	iOutput = pid.ki * (pid.errorSum);

}
