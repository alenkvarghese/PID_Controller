#include<iostream>
#include<cmath>
#include<algorithm>

#include"../include/PidController.hpp"


PidController::PidController(const PIDConfig& cfg)
    :m_cfg(cfg)
    {
        ResetParams();
    }

void PidController::ResetParams(void)
{
    pTerm = 0; iTerm=0.0; dTerm = 0.0;
    prev_controlSignal = 0.0; prev_error = 0.0; prev_state = 0.0;
}

double PidController::ComputeControlSignal(
    const double& __setpoint, 
    const double& __state,
    const double& __time,
    const double& __k, 
    const double& __ti,
    const double& __td,
    const double& __tt)
        {   
            double __error = __setpoint - __state;
            double error_pTerm = (m_cfg.spWeight_ProportionalTerm * __setpoint - __state);

            pTerm = __k * error_pTerm;
            
            if(m_cfg.allowDerivative)
            {
                dTerm = __Differentiator(__state, __k, __td);
            }
            else dTerm = 0.0;
            

            if(m_cfg.allowIntegral)
            {
                iTerm = __Integrator(__error, __k, __ti, __tt);
            }
            else iTerm = 0.0;

            // u_min <= control signal <= u_max 
            controlSignal = std::clamp(pTerm + iTerm + dTerm, m_cfg.u_min, m_cfg.u_max);

            return controlSignal;
        }

double PidController::__Differentiator(
    const double& state, 
    const double& k, 
    const double& td)    
    {
        const double& h = m_cfg.timeStep;    // Time Step

        // Weight Computation for Derivative Term (Tustins Method)
        double a1 = -1.0, a2 = k*td/h; // Normal weights, no filter
        
        if (m_cfg.allowDerivativeFilter)
        {   
            // First-Order Filter
            const double& N = m_cfg.filterConst; // filterConst
            a1 = (2*td - N*h)/(2*td*N + N*h);
            a2 = -(2*k*td)/(2*td + N*h);
        }

        return a1 * dTerm + a2*(state - prev_state);
    }

double PidController::__Integrator(
    const double error, 
    const double k, 
    const double ti, 
    const double tt)
    {
        const double& h = m_cfg.timeStep;    // Time Step
        if (m_cfg.allowWindupProtection)
        {
            switch (m_cfg.windupMethod)
            {
            case 1:
            {
                // Back Calculation
                double u_predicted = pTerm + dTerm + iTerm;
                double e_s = std::clamp(u_predicted, m_cfg.u_min, m_cfg.u_max) - u_predicted;
                return iTerm + (k*h)/(2*ti)*(error + prev_error) + (1/tt)*(e_s);
            }



            default:
                // Clamping
                return std::max(iTerm + (k*h)/(2*ti)*(error + prev_error), m_cfg.maxIntegralValue);
                
            }
        }
        return iTerm + (k*h)/(2*ti)*(error + prev_error);
    }

