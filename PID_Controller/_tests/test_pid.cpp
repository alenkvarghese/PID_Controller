#include<iostream>
#include"../include/PidController.hpp"

int main()
{
    PIDConfig cfg;

    PidController pid(cfg);
    double setpoint = 10.0;
    double output = 0.0;

    return 0;
}