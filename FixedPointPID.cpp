// Fixed-Point PID Algorithm
// Ported from: https://gist.github.com/bradley219/5373998
// Author: Li "oldrev" Wei <oldrev@gmail.com>

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#define FIXED32_Q (16)
#define FIXED32_FMASK (((Fixed32)1 << FIXED32_Q) - 1)

typedef int32_t Fixed32;
typedef int64_t Fixed64;

Fixed32 Fixed32_FromInt(int32_t n) {
	return (Fixed32)((Fixed32)n << FIXED32_Q);
}

int32_t Fixed32_Frac(Fixed32 a){
	return a & FIXED32_FMASK;
}

// Optional
Fixed32 Fixed32_FromFloat(float f) {
	return (Fixed32)((f) * (((Fixed64)1 << FIXED32_Q) + ((f) >= 0 ? 0.5 : -0.5)));
}

// Optional
Fixed32 Fixed32_ToFloat(float T) {
	return (float)((T)*((float)(1)/(float)(1 << FIXED32_Q)));
}

Fixed32 Fixed32_Mul(Fixed32 a, Fixed32 b) {
	return (Fixed32)(((Fixed64)a * (Fixed64)b) >> FIXED32_Q);	
}

Fixed32 Fixed32_Div(Fixed32 a, Fixed32 b) {
	return (Fixed32)(((Fixed64)a << FIXED32_Q) / (Fixed64)b);
}

typedef struct {
	Fixed32 Dt;
	Fixed32 Max;
	Fixed32 Min;
	Fixed32 Kp;
	Fixed32 Kd;
	Fixed32 Ki;
	Fixed32 PrevError;
	Fixed32 Integral;
} FixedPid;


void FixedPid_Init(FixedPid* self) {
	memset(self, 0, sizeof(FixedPid)); 
}

Fixed32 FixedPid_Calculate(FixedPid* self, Fixed32 setpoint, Fixed32 pv) {
    // Calculate error
    Fixed32 error = setpoint - pv; //setpoint - pv;

    // Proportional term
    Fixed32 Pout = Fixed32_Mul(self->Kp, error); // self.Kp * error;

    // Integral term
    self->Integral += Fixed32_Mul(error, self->Dt);  // self.Integral += error * self.Dt;
    Fixed32 Iout = Fixed32_Mul(self->Ki, self->Integral);

    // Derivative term
    Fixed32 derivative = Fixed32_Div(error - self->PrevError, self->Dt);
    Fixed32 Dout = Fixed32_Mul(self->Kd, derivative);

    // Calculate total output
    Fixed32 output = Pout + Iout + Dout;
	
    // Restrict to max/min
    if(output > self->Max) {
        output = self->Max;
	}
    else if(output < self->Min){
        output = self->Min;
	}

    // Save error to previous error
    self->PrevError = error;
    return output;
}


int main() {
	FixedPid pid;
	FixedPid_Init(&pid);

	pid.Dt = Fixed32_FromFloat(0.1);
	pid.Max = Fixed32_FromFloat(100);
	pid.Min = Fixed32_FromFloat(-100);
	pid.Kp = Fixed32_FromFloat(0.1);
	pid.Kd = Fixed32_FromFloat(0.01);
	pid.Ki = Fixed32_FromFloat(0.5);
	float float_val = 20;
	Fixed32 val = Fixed32_FromFloat(float_val);
	printf("max=%d    min=%d    dt=%d\n", pid.Max, pid.Min, pid.Dt);
	for(int i = 0; i < 100; i++) {
		Fixed32 inc = FixedPid_Calculate(&pid, 0, val);
		float float_inc = Fixed32_ToFloat(inc);
		float_val = Fixed32_ToFloat(val);
        	printf("%d - val:% 7.8f inc:% 7.8f\n", i, float_val, float_inc);
		val += inc;
	}
	return 0;
}