#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <iostream>
#include <cmath>
#include <cassert>
#include <algorithm>
#include <vector>

constexpr double g_pi = 3.1416;

// PerformanceMonitor analyses the real time stability and performance of the PID controller including 
// ** The quality of gains and saturation times, presence of oscillations and instability in the system.
// ** The reachability of setpoints and feasibility of control bounds as well as estimation of parameters like
// dead times and process gain etc.
class PerformanceMonitor
{
private:

    /* ==== Common variables ==== */

    unsigned int m_count_call = 0; // Count of Controller calls

    double m_time_step = 0.0;  // Time Step of Controller (from config struct)
    double time = 0.0;  // Time (Call Count * time Step)
    double prev_error = 0.0;

    double k, td, ti, tt; // Controller gain

    unsigned int m_count_upperSat = 0; // Count of instants of saturation at u_max
    unsigned int m_count_lowerSat = 0; // Count of instants of saturation at u_min

    
    /* ==== Variables for Oscillation and Load Detection ==== */

    double m_omega_ult; // Ultimate Frequency
    double m_gamma = 0.45; // Memory weightage for load history
    double m_load_mem = 0.0; // Memory term for load history

    double m_iae = 0.0; // The value of Integral Absolute Error (IAE) in the gap between subsequent zero crossings of error signal.
    double m_iae_limit = 0.0; // Limit on Integral absolute error for detecting load disturbance.

    double m_oscillation_amplitude = 1.0; // Amplitude of oscillation of controller
    double m_oscillation_limit = 10.0; // Limit of number of oscillations detected

    bool f_oscillation_detected = false; // Flag for detecting presence of oscillation
    bool f_oscillation_sustained = false; // Flag to see if oscillation is sustaied

    // time of Zero Crossing of error signal
    unsigned int  m_prev_zero_crossing[2] = {0,0}, m_count_zero_crossing = 0.0;

    // Peak vales of error in previous and current oscillation cycle
    double m_prev_error_max = 0.0, m_error_max = 0.0;

    // Min Rate of decrease of errorMax of oscillations to ascertain decaying. 
    // If rate is below m_eps, its taken to be presence of sustained oscillation.
    double m_eps = 0.3f;

    unsigned short int m_load = 0;
    
    // Function to monitor oscillations and load disturbances in the system. 
    //
    // It detects zero crossings in the error signal and monitors the change in amplitude of 
    // error signal peak. It flags oscillations as sustained if subsequent peaks of error signal 
    // do not show sufficent deacy (approx 30%).
    // 
    // Depending on the ultimate frequency, if number of oscillations exceeds a limit, it warns
    // the user about presence of sustained oscillations and suggests retuning.
    //
    // Moreover, in the gap between suqsequnt zero crossings, the Integral Absolute Error (IAE) 
    // is measured and  if it exceeds iae_limit, its taken to be presence of load disturbance in the 
    // system. Choice of iae_limit is a tradeoff between accuracy of detection and false positives, 
    // and is best set based on the expected load disturbances in the system. 
    //
    // During the event of a setpoint change(>30% of current setpoint), the values are reset 
    // to avoid false positives.
    // 
    // Certain constants like omega_u (ultimate frequency) are estimated using parameters of corrosponding 
    // PidController Object. So highly imporper tuning can generate erratic results. Hence such parameters 
    // are flagged and error is thrown. 
    void monitor_Oscillation(const double error)
    {
        // Load Detection
        if (error*prev_error < 0.0)
        {
            m_load = (m_iae > m_iae_limit) ? 1 : 0;

            m_iae = std::fabs(error)*m_time_step;
            
            if(m_prev_error_max > 0.0)
            {
                f_oscillation_sustained = (std::fabs((m_error_max - m_prev_error_max)/m_prev_error_max) > m_eps);
            }
            else{}

        }
        else m_error_max = std::max(prev_error, error);

        // Oscillation detection
        m_load_mem = m_gamma*m_load_mem + m_load;
        if (m_load_mem > 10.0){}
    }

    /* ==== Variables for Saturation ==== */

    double m_u_max = 1.0e6;        
    double m_u_min = -1.0e6; 

    // Function monitoring saturation of controller output. 
    // If it exceeds 5% in upper of loweer bound, its warns user.
    // Only starts monitoring after 20-30 of operation to 
    // avoid false positives during setpoint changes and initial transients.
    void monitor_saturation(const double controlSignal)
    {
        if (controlSignal >= m_u_max){++m_count_upperSat;}
        if (controlSignal <= m_u_min){++m_count_lowerSat;}

        if (m_count_call > 1/m_time_step)
        {
            if (m_count_lowerSat/m_count_call >= 5)
            {std::cout<<"Saturation of controller exceeds 5\% at lower bound";}

            if (m_count_upperSat/m_count_call >= 5)
            {std::cout<<"Saturation of controller exceeds 5\% at upper bound";}
        }
    }
    
    void monitor_gainQuality(void){}


    void estimate_Kp(void){}

public:

    PerformanceMonitor(
        const double time_step,
        const double kp,
        const double ki,
        const double kd,
        const double u_max,
        const double u_min,
        const double filter_coeff)
    {
        m_time_step = time_step;

        setGains(kp, ki, kd);
        setActuatorLimits(u_min, u_max);

    }

    void setGains(
        const double kp, 
        const double ki, 
        const double kd)
        {
            assert(kp > 0.0);
            k = kp;

            assert(ki > 0.0);
            ti = k/ki;

            m_gamma = 1.0 - (50*ti)/m_time_step;
            m_omega_ult = (2.0*g_pi)/ti;
            m_iae_limit = (2.0*m_oscillation_amplitude)/m_omega_ult;

            tt = ti;
            if (kd > 0.0)
            {
                td = kd/k;
                tt = std::sqrt(ti*td);
            }
        }

    void setActuatorLimits(
        const double u_min, 
        const double u_max) //:m_u_min(u_min), m_u_max(u_max)
        {}


    void Monitor(
        const double& error,
        const double& control_signal)
    {
        ++m_count_call;
        
        monitor_saturation(control_signal);
        monitor_Oscillation(error);

    }
};


// Configuration Struct for PID Controller
//
// @param timeStep : The delta_t in discretization for simulation
// @param u_max : Actuator Limit(Maximum). Can be set through function PidConroller::setActuatorLimits(u_min, u_max).
// @param u_min : Actuator Limit(Minimum). Can be set through function PidConroller::setActuatorLimits(u_min, u_max).
// @param spWeight : Setpoint weight for Proportional Term. Defaults to 1.
// @param allowWindupProtection : To allow of disallow Integral Windup Protection using Back-Calculation. Defaults to false
// @param allowFilter : To allow or disallow Filetr for derivative term
// @param filterConst : Constant for derivative filter. Typically between 7 and 20.
// @param f_enabelLogging : To allow or disallow Logging of data, allowing logging, plotting and convert_to_csv. Defaults to false. See: Logger
// @param f_enableMonitoring : To allow or disallow Closed Loop Performance Monitoring. Defaults to false. See: PerformanceMonitor.
struct PIDConfig
{
    double kp;
    double kd;
    double ki;

    double time_step = 1e-3;

    double u_max = 1.0e12;
    double u_min = -1.0e12;

    double sp_weight = 1.0;
    double filter_const = 10.0;

    bool allow_windup_protection = true;
    bool allow_filter = true;
    bool f_enable_logging = false;
    bool f_enable_monitoring = false;
};

// Class for a classical PID controller.
class PidController
{
private:

    /*Internal variable for values of Current call;
     stores values from previous call at the beginning of call*/
    double m_pTerm = 0.0, m_iTerm = 0.0, m_dTerm = 0.0;

    /*Variable storing inputs and outputs from previous iteration  */
    double m_prev_state = 0.0, m_prev_control_signal = 0.0, m_prev_error = 0.0;

    // Internal Flag
    bool f_sp_weight = false, f_deriv = false, f_intgr = false;

    void validateConfig()
    {
        assert(s_cfg.time_step > 0.0);
        assert(s_cfg.u_max - s_cfg.u_min > 0.0);
        assert(s_cfg.sp_weight>0.0 && s_cfg.sp_weight < 1.0);
        f_sp_weight = true;

    };

public:

    double k = 0.0;
    double ti = 0.0;
    double td = 0.0;
    double tt = 0.0;

    PIDConfig s_cfg;

    double m_control_signal = 0.0;

    PidController(
        const PIDConfig& config)
        :s_cfg(config)
    { 
        validateConfig();
        setGains(config.kp, config.ki, config.kd);
    }
    
    // sets gains of the controller and updates internal variables accordingly.
    //
    // Internally these are converted to k, ti, td and tt(windup protection time constant = sqrt(ti*td) if td !=0 else tt = ti).
    //
    // @param kp : Proportional Gain
    // @param ki : Integral Gain
    // @param kd : Derivative Gain
    void setGains(
        const double& kp,
        const double& ki,
        const double& kd)
    {
        assert(kp >0.0);  k = kp;

        f_deriv = false;
        if (kd > 0.0)
        {
            f_deriv = true;  
            td = kd/k;
        }

        f_intgr = false;
        if(ki > 0.0)
        {
            f_intgr = true;  
            ti = k/ki;

            tt = ti;
            if (f_deriv){tt = std::sqrt(ti*td);}
        }
    }

    void begin(void)
    {
        // Reset Terms
        m_pTerm = 0.0;
        m_iTerm = 0.0;
        m_dTerm = 0.0;

        m_prev_state = 0.0;
        m_prev_control_signal = 0.0;
        m_prev_error = 0.0;

    }

    // Function to compute control signal for given setpoint and state. 
    //
    //Setpoint and state are required explicitly required to accomodate setpoint changes and setpoint weighting.
    //
    // The function also updates internal variables for next call and logs data if enabled in config struct.
    //
    // @param setpoint : Desired value of the variable being controlled
    // @param state : Current value of the variable being controlled
    // @return control signal computed by the controller
    double computeControlSignal(
        const double& setpoint,
        const double& state)
    {
        const double N = s_cfg.filter_const;

        const double h = s_cfg.time_step;    /*Time Step*/

        const double error = setpoint - state;

        double error_pTerm = error;
        if (f_sp_weight){error_pTerm = s_cfg.sp_weight*setpoint - state;}

        /*Proportional Term*/
        m_pTerm = k*error_pTerm;

        /*Derivative Term*/
        if (f_deriv)
        {
            /*Weights for Tustins Method*/
            double a1 = 0.0, a2 = k*td/h;
            if(s_cfg.allow_filter)
            {
                /*First order Filter*/
                
                a1 = (2*td - N*h)/(2*td + N*h);
                a2 = -(2*k*td*N)/(2*td + N*h);
            }
            m_dTerm = a1*m_dTerm + a2*(state - m_prev_state);
        }
        
        /*Integral Term*/
        if (f_intgr)
        {   
            m_iTerm = m_iTerm + (k*h*0.5/ti)*(error + m_prev_error);
            if (s_cfg.allow_windup_protection)
            {
                const double u_pred = m_pTerm + m_iTerm + m_dTerm;

                /*Saturation of predicted output*/
                const double error_sat = std::clamp(u_pred, s_cfg.u_min, s_cfg.u_max) - u_pred;
                m_iTerm += (h/tt)*error_sat;
                }
        }

        m_control_signal = m_pTerm + m_iTerm + m_dTerm;
        if (s_cfg.f_enable_monitoring){}

        m_control_signal = std::clamp(m_control_signal, s_cfg.u_min, s_cfg.u_max);

        
        m_prev_state = state;
        m_prev_error = error;
        m_prev_control_signal = m_control_signal;

        return m_control_signal;
    }    
};

#endif