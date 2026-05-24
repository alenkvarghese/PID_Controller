#ifndef PID_CONTROLLER_HPP

#define PID_CONTROLLER_HPP

#include<iostream>

#include"ControllerConfig.hpp"
#include"Polynomial.hpp"

struct PIDConfig: public Config
{
    double timeStep;
    double u_max;
    double u_min;
    double spWeight_ProportionalTerm;

    bool allowIntegral;
    bool allowWindupProtection;
    int windupMethod;
    double maxIntegralValue;

    bool allowDerivative;
    bool allowDerivativeFilter;
    double filterConst;

    bool verbose;
};

class PidController
{
public:
    PIDConfig m_cfg;        // Config Sturct
    double controlSignal;   // Output of controller

    // Contructor & Deconstructor
    PidController(const PIDConfig& config);
    

    double ComputeControlSignal(
        const double& setpoint, 
        const double& state,
        const double& time,
        const double& k, 
        const double& ti,
        const double& td,
        const double& tt);
private:
    
    double pTerm, iTerm, dTerm;
    double prev_state, prev_controlSignal, prev_error;

    void ResetParams(void);
    void UpdateParams(void);
    void displayInternalParams(void);
    void chechSanity(void);
    
    double __Differentiator(
        const double& state, 
        const double& k, 
        const double& td);
    

    double __Integrator(
        const double error, 
        const double k, 
        const double ti, 
        const double tt);
    
};

#endif