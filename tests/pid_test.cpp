#include<iostream>

#include"../ClassicalPID.hpp"

constexpr double dt = 1e-2;

double plant(double state, double controlSignal)
{
    // first order system
    return (1 - dt)*state + dt*controlSignal;
}

int main(void)
{
    PIDConfig cfg;

    std::cout<<"start";

    cfg.timeStep = dt;
    cfg.u_min = -1.1;
    cfg.u_max = 1.1;
    cfg.spWeight = 1.0;
    cfg.allowWindupProtection = false;
    cfg.allowFilter = true;
    cfg.filterConst = 1.0;

    PidController ctrl(2.8, 2.0, 1e-2, cfg);

    double x = 0.0; double u = 0.0;

    for (int i = 0; i<2000; i++)
    {
        x = plant(x, u);
        u = ctrl.computeControlSignal(1.0, x);
        std::cout << x << "\t" << u << "\t iter " << i << '\n' << std::flush;
    }
    return 0;
}
