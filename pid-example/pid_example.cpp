//Copyright 2019 Bradley J. Snyder <snyder.bradleyj@gmail.com>


#include "pid.h"
#include <stdio.h>

int main() {

    PID pid = PID(0.05,0.9, 0.005, 0); //dt max min kp kd ki

    double val = 20;
    for (int i = 0; i < 100; i++) {
        double inc = pid.calculate(0, val);
        printf("val:% 7.3f inc:% 7.3f\n", val, inc);
        val += inc;
    }

    return 0;
}
