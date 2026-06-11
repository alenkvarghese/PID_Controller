#ifndef PIDTUNER_H

#define PIDTUNER_H

#include<iostream>
#include<cmath>
#include<algorithm>
#include<vector>
#include<array>
#include<random>
#include<cassert>

#include"ClassicalPID.hpp"


std::mt19937 rng(std::random_device{}());
std::uniform_real_distribution<double> dist(0.0, 1.0);

class PIDTuner
{
private:
    // Configured controller to be tuned
    PidController ctrl;

    // Bounds on Parameters 
    std::array<double, 4> m_upper_bound, m_lower_bound;

    const unsigned short int m_iter_limit, m_agents;

    std::vector<std::array<double,4>> values;

public:
    PIDTuner(
        const PIDConfig& controller_config, 
        const unsigned short int& number_of_agents,
        const unsigned short int& iterations,
        const std::array<double, 4>& upper_bound, 
        const std::array<double, 4>& lower_bound)
    :
    ctrl(controller_config),
    m_agents(number_of_agents),
    m_iter_limit(iterations),
    m_upper_bound(upper_bound), 
    m_lower_bound(lower_bound),
    values(number_of_agents)
    {
        assert(upper_bound[0] > lower_bound[0]);
        assert(upper_bound[1] > lower_bound[1]);
        assert(upper_bound[2] > lower_bound[2]);
        assert(upper_bound[3] > lower_bound[3]);

        for (int i = 0; i < m_agents; i++)
        {
            values[i][0] = m_lower_bound[0] + dist(rng)*(m_upper_bound[0] - m_lower_bound[0]);
            values[i][1] = m_lower_bound[1] + dist(rng)*(m_upper_bound[1] - m_lower_bound[1]);
            values[i][2] = m_lower_bound[2] + dist(rng)*(m_upper_bound[2] - m_lower_bound[2]);
            values[i][3] = m_lower_bound[3] + dist(rng)*(m_upper_bound[3] - m_lower_bound[3]);
        }
    }
};



#endif