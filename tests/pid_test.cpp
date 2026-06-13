#include<iostream>
#include<cmath>
#include<deque>

#include"../ClassicalPID.hpp"

constexpr double dt = 1e-2;


class System
{
private:
    bool allow_noise = false;
    double delay;
    std::deque<double> delayed_state;
    long unsigned int max_dequeSize = 0;

public:
    System(
        bool allow_noise,
        int delay)
        :
        allow_noise(allow_noise),
        delay(delay)
        {
            for(int i = 0; i < delay; ++i) delayed_state.push_back(0.0);
            max_dequeSize = delay + 1;
        }

    double run(double time, double state, double controlSignal)
    {
        if(delayed_state.size() > max_dequeSize){delayed_state.pop_front();}
        delayed_state.push_back(state);

        double value = (1 - dt)*delayed_state[0] + dt*controlSignal;

        value += (allow_noise)*0.01*sin(1000*time);

        return value;
    }
};

int main(void)
{
    // Declaring Controller
    PIDConfig cfg;
 // Configuring Controller
    cfg.kp = 1.0;   cfg.ki = 1.0;   cfg.kd = 1e-3;

    cfg.time_step = dt;
    
    cfg.u_min = -1.1;   cfg.u_max = 1.1;
    
    cfg.sp_weight = 1.0;
    cfg.allow_windup_protection = true;
    cfg.allow_filter = true;
    cfg.filter_const = 1.0;
    cfg.f_enable_monitoring = true;


    // Initializing Controller
    PidController ctrl(cfg);

    // Declaring System
    System sys(true, 0);

    // Initial Conditions
    double x = 0.0; double u = 0.0;

    // Beginnnging Controller
    ctrl.begin(x, 0.0);

    // Main Control Loop
    for (int i = 0; i<1000; i++)
    {
        x = sys.run(i*dt, x, u);
        u = ctrl.computeControlSignal(1.0, x);
        std::cout << x << "\t" << u << "\t iter " << i << '\n' << std::flush;
    }

    ctrl.end();
    return 0;
}
