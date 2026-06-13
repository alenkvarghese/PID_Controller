// The main PID Controller and performance Monitor.
// A basic (Integer Order) PID Controller is implemented here. Has provisions for noise filtering, Setpoint weighting, integral windup protection etc.
// The Controller also encompasses the usual PID Controller with some modifications. Refer [1] for more details on each feature. The controller is configured
// using a Struct PID_Config. As for the theory of the classical PID Controller, the proportional, integral and derivative term encapsulate the
//  present, past and future respectively. The basic control law is given by:
//          $u(t) = K(e[n] + td * de[n]/dt + (1/ts)*Integral(e[n]) )$
// In this implementation the derivative term has a filter and integral term has a back-calculation based Windup Protection. Filetrs help to eliminate noise,
// while windup protection is for dealing with the phenomenon on integral windup, due to actuator saturation. This causes a groggy and laggy controller. 
// Back-calculation can get unstable if sqrt(ki/kd) << 1. It can cause negative control signals for positive errors. Generally Kd << Ki or Kp. Derivative 
// term is sensitive to noise and integral back-calculation is sensitive to value of Kd. Either of this ought to be prevented for good performance.
//
// The performance monitor is another aspect of this. It can be Configured is Config Struct. 
//
// More specifics on working is present at each part seperately.
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <iostream>
#include <cassert>
#include <algorithm>
#include <vector>
#include<optional>
#include<deque>
#include<list>
#include <stdexcept>

constexpr double g_pi = 3.1416;

class Logger
{
private:
    std::list<double> error_signal;
    std::list<double> reference_signal;
    std::list<double> control_signal;

    std::list<double> proportional_term;
    std::list<double> integral_term;
    std::list<double> derivative_term;

public:

    Logger(const int& iter)
    :
    error_signal(iter),
    reference_signal(iter),
    control_signal(iter),
    proportional_term(iter),
    integral_term(iter),
    derivative_term(iter)
    {}

    void log_data(
        double reference,
        double error,
        double control,
        double p_term,
        double i_term,
        double d_term)
        {
            reference_signal.push_back(reference);
            error_signal.push_back(error);
            control_signal.push_back(control);

            proportional_term.push_back(p_term);
            integral_term.push_back(i_term);
            derivative_term.push_back(d_term);
        }
};



// PerformanceMonitor analyses the real time stability and performance of the PID controller including 
// ** The quality of gains and saturation times, presence of oscillations and instability in the system.
// ** The reachability of setpoints and feasibility of control bounds as well as estimation of parameters like
// dead times and process gain etc.
class PerformanceMonitor
{
private:

    // Controller Parameter
    std::deque<double> error_window;
    double u_maxValue = 1.0e6;        
    double u_minValue = -1.0e6;
    unsigned int count_saturation = 0; // Count of instants of saturation at u_max

    double k, td, ti, tt; 

    // Simulation Parameters
    unsigned int count_call = 0; // Count of Controller calls
    double time_step;  
    double initial_state;
    double initial_time = 0.0;

    double error= 0.0;    // Current Error Value
    double steadyStateError = 0.0; 
    double prev_error = 0.0;

    double prev_setpoint = 0.0;

    std::vector<double> rise_time_arr; // time for state to rise from 10% of setpoint ro 90% of setpoiny

    double rise_time_y10 = 0.0; // Time when state reaches 10% of setpoint. Must be stored until rise time is calculated.
    
    double peak_value = 0.0;
    double peak_value_time = 0.0;

    std::vector<double> overshootValue_arr;
    std::vector<double> overshootTime_arr;
    std::vector<double> settlingTime_arr;
    
    bool y10_recorded = false;
    bool y90_recorded = false;
    bool peakValue_recorded = false;
    bool steadyState_reached = false;

    
    // System Parameters

    double omega_ult; // Ultimate Frequency
    double gamma = 0.45; // Memory weightage for load history

    double load_memory = 0.0; // Memory term for load history

    double iae = 0.0; // The value of Integral Absolute Error (IAE) in the gap between subsequent zero crossings of error signal.
    double iae_limit = 0.0; // Limit on Integral absolute error for detecting load disturbance.

    double oscillation_amplitude = 1.0; // Amplitude of oscillation of controller
    double oscillation_limit = 10.0; // Limit of number of oscillations detected

    // time of Zero Crossing of error signal
    double prev_zero_crossing = 0.0;

    // Peak vales of error in previous and current oscillation cycle
    double prev_error_max = 0.0, m_error_max = 0.0;

    // Band Threshold for distinguishing noise and actual zerocorssings (in percent)
    double noise_threshold = 0.05;

    bool load = 0;

public:

    PerformanceMonitor(
        const double& time_step,
        const double& initial_state,
        const double& initial_time,
        const double& kp,
        const double& ki,
        const double& kd,
        const double& u_max,
        const double& u_min,
        const double& filter_coeff)
    :
    time_step(time_step),
    initial_state(initial_state),
    initial_time(initial_time)
    {
        setGains(kp, ki, kd);
        setActuatorLimits(u_min, u_max);
    }

    void setGains(
        const double& kp, 
        const double& ki, 
        const double& kd)
        {
            assert(kp > 0.0);
            k = kp;

            assert(ki > 0.0);
            ti = k/ki;

            gamma = 1.0 - (50*ti)/time_step;
            omega_ult = (2.0*g_pi)/ti;
            iae_limit = (2.0*oscillation_amplitude)/omega_ult;

            tt = ti;
            if (kd > 0.0)
            {
                td = kd/k;
                tt = std::sqrt(ti*td);
            }
        }

    void setActuatorLimits(
        const double& u_min, 
        const double& u_max)
        {
            u_maxValue = u_max;
            u_minValue = u_min;
        }


    void Monitor(
        const double& setpoint,
        const double& state,
        const double& control_signal)
    {
        error = setpoint - state;
        ++count_call;

        /*Basic Params: rise time, prak time etc..*/
        double delta_setpoint = std::fabs(setpoint - prev_setpoint);

        // Change in setpoint must be atlest 5% to be counted as a change in setpoint.
        // Once setpoint change is encountered, all values from previous setpoint are discarded.
        if (delta_setpoint/std::fabs(prev_setpoint) > 0.05 and prev_setpoint > 0.0)
        {
            if(!y10_recorded or !y90_recorded)
            {
                rise_time_arr.push_back(0);
                overshootValue_arr.push_back(0);
                overshootTime_arr.push_back(0);
                settlingTime_arr.push_back(0);
            }
            y10_recorded = false;
            y90_recorded = false;
            peakValue_recorded = false;
            steadyState_reached = false;

            error_window.clear();
        }

        if(state >= 0.1*setpoint and !y10_recorded)
        {
            y10_recorded = true;
            rise_time_y10 = initial_time + count_call*time_step;
        }


        if(state >= 0.9*setpoint and !y90_recorded)
        {
            y90_recorded = true;
            rise_time_arr.push_back(initial_time + count_call*time_step - rise_time_y10);
        }

        if(!peakValue_recorded)
        {
            peak_value = std::max(peak_value, state);
            peak_value_time = count_call*time_step;
        }

        error_window.push_back(std::fabs(error));
        if(error_window.size() > 5)
        {
            error_window.pop_front();

            if(!steadyState_reached)
            {
                double sum_err = 0.0;
                for(int i = 0; i < 5; i++) sum_err += error_window.at(i);

                // Error must be with 5% or reference vales for 
                if (sum_err/(5*setpoint) <= 0.05)
                {
                    steadyState_reached = true;
                    peakValue_recorded = true;

                    overshootValue_arr.push_back(peak_value/setpoint);
                    overshootTime_arr.push_back(peak_value_time);
                    settlingTime_arr.push_back((count_call - 5)*time_step);
                }
            }
        }

        /* Detection of Actuator Saturation */
        // Function monitoring saturation of controller output. 
        // If it exceeds 5% in upper of loweer bound, its warns user.
        // Only starts monitoring after 20-30 of operation to 
        // avoid false positives during setpoint changes and initial transients.
        count_saturation += (control_signal <= u_minValue) or (control_signal >= u_maxValue);

        if ((count_call > 1.0/time_step) and (count_saturation > 0.1*count_call))
        {   
            std::cerr<<"\n Warning! Saturation Detected \n "<<std::endl; 
            count_saturation = 0;
        }


        /*Detection of Zero Crossings and Load */
        // It detects zero crossings in the error signal and monitors the change in amplitude of 
        // error signal peak. It flags oscillations as sustained if subsequent peaks of error signal 
        // do not show sufficent deacy.
        // 
        // Depending on the ultimate frequency, if number of oscillations exceeds a limit, it warns
        // about presence of sustained oscillations and suggests retuning.
        //
        // Moreover, in the gap between subsequent zero crossings, the Integral Absolute Error (IAE) 
        // is measured and  if it exceeds iae_limit, its taken to be presence of load disturbance in the 
        // system. Choice of iae_limit is a tradeoff between accuracy of detection and false positives, 
        // and is best set based on the expected load disturbances in the system. 
        //
        // During the event of a setpoint change, certain values are reset to avoid false positives.
        // 
        // Certain constants like omega_u (ultimate frequency) are estimated using parameters of corrosponding 
        // PidController. So highly imporper tuning can generate erratic results. Hence such parameters 
        // are flagged and error is thrown.
        if (error*prev_error < 0.0 and std::fabs(error) > noise_threshold)
        {
            load = (iae > iae_limit);
            if(load){std::cerr<<"Warning! Load Detected";}
            
            // resetting IAE for new supposed load cycle
            iae = std::fabs(error)*time_step;

            // resetting error max for new supposed load cycle
            m_error_max = std::fabs(error);

        }  
        else
        {
            iae += std::fabs(error)*time_step;
            load = 0;

            m_error_max = std::max(std::fabs(error), std::fabs(prev_error));
        }
        
        /* Oscillation detection*/
        // Oscillations are said to be present if,for some supervision time, T_sup, the numbers of detectiosn exceed limit*T_ult/2, for every half period.
        // Generally, this implies that load had been detected (IAE > IAE_Limit) for ~10 times in given supervision time. This would prectically require 
        // all values in T_sup to be stored in. Instead a weighted memory term is used for this. Refer [2] 
        load_memory = gamma*load_memory + load;
        if(load_memory > 5.0)
        {
            if (load_memory > 15.0)
            {
                throw std::runtime_error("Too many oscillations detected. Session Terminated!");
            }

            load_memory = 0.0;
            std::cerr<<"Warning! Oscillations Detected";
        }
    }

    void ShowResults(void)
    {
        std::cout<<"======================================================="<<std::endl;
        std::cout<<"\t \t Performance Analysis"<< std::endl;
        std::cout<<"======================================================="<<std::endl;

        for(unsigned long int i = 0; i < rise_time_arr.size(); ++i)
        {
            std::cout<<"Setpoint change  #"<<i+1<<std::endl;

            std::cout<<"Rise Time       "<< rise_time_arr.at(i)<<" sec"<<std::endl;

            std::cout<<"Overshoot       "<<overshootValue_arr.at(i)<<" % \t"<<
                "at "<<overshootTime_arr.at(i)<< " sec"<<std::endl;

            std::cout<<"Settling Time:  "<<settlingTime_arr.at(i)<< " sec"<<std::endl;
        }
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

    int iterations = 500;

    bool allow_windup_protection = true;
    bool allow_filter = true;
    bool f_enable_monitoring = false;
};

// Class for a classical PID controller.
class PidController
{
private:

    /*Internal variable for values of Current call;
     stores values from previous call at the beginning of call*/
    double proportional_term = 0.0, integral_term = 0.0, derivative_term = 0.0;

    /*Variable storing inputs and outputs from previous iteration  */
    double prev_state = 0.0, prev_control_signal = 0.0, prev_error = 0.0;

    // Internal Flag
    bool f_sp_weight = false, f_deriv = false, f_intgr = false;

    std::optional<Logger> PID_Log;
    std::optional<PerformanceMonitor> PID_Monitor;

    void validateConfig()
    {
        assert(s_cfg.time_step > 0.0);
        assert(s_cfg.u_max - s_cfg.u_min > 0.0);
        assert(s_cfg.sp_weight>0.0 && s_cfg.sp_weight <= 1.0);
        f_sp_weight = true;

    };

public:

    double k = 0.0;
    double ti = 0.0;
    double td = 0.0;
    double tt = 0.0;

    PIDConfig s_cfg;

    double control_signal = 0.0;

    PidController(
        const PIDConfig& config)
        :s_cfg(config)
    { 
        validateConfig();
        setGains(config.kp, config.ki, config.kd);
    }
    
    // sets gains of the controller and updates internal variables accordingly.
    //
    // Internally these are converted to k, ti, td and tt.
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

    void begin(
        double initial_state,
        double initial_time)
    {
        // Reset Terms
        proportional_term = 0.0;
        integral_term = 0.0;
        derivative_term = 0.0;

        prev_state = 0.0;
        prev_control_signal = 0.0;
        prev_error = 0.0;


        PID_Log.emplace(s_cfg.iterations);
        PID_Monitor.emplace(
            s_cfg.time_step,
            initial_state,
            initial_time,
            s_cfg.kp,
            s_cfg.ki,
            s_cfg.kd,
            s_cfg.u_max,
            s_cfg.u_min,
            s_cfg.filter_const
        );
    }

    void end(void)
    {
        PID_Monitor.value().ShowResults();
    }

    // Function to compute control signal for given setpoint and state. 
    //
    //Setpoint and state are required explicitly required to accomodate setpoint changes and setpoint weighting.
    //
    // The function also updates internal variables for next call and logs data. It also performs performance analysis using 
    // PerformancMonitor if enabled.
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
        proportional_term = k*error_pTerm;

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
            derivative_term = a1*derivative_term + a2*(state - prev_state);
        }
        
        /*Integral Term*/
        if (f_intgr)
        {   
            integral_term = integral_term + (k*h*0.5/ti)*(error + prev_error);
            if (s_cfg.allow_windup_protection)
            {
                const double u_pred = proportional_term + integral_term + derivative_term;

                /*Saturation of predicted output*/
                const double error_sat = std::clamp(u_pred, s_cfg.u_min, s_cfg.u_max) - u_pred;
                integral_term += (h/tt)*error_sat;
                }
        }
        // Unsaturated control signal
        control_signal = proportional_term + integral_term + derivative_term;

        // Logging data
        PID_Log.value().log_data(setpoint, error, control_signal, proportional_term, integral_term, derivative_term);

        // Monitoring of controller performance
        if (s_cfg.f_enable_monitoring)
        {
            PID_Monitor.value().Monitor(
                setpoint,
                state,
                control_signal
            );
        }

        // Actual, Saturated control signal
        control_signal = std::clamp(control_signal, s_cfg.u_min, s_cfg.u_max);

        
        prev_state = state;
        prev_error = error;
        prev_control_signal = control_signal;

        return control_signal;
    }    
};

#endif