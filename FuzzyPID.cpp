


#include<iostream>
#include<cmath>
#include<algorithm>
#include<cassert>
#include<vector>

#include"ClassicalPID.hpp"

// Tolerance for floating point rounding errors
constexpr double g_eTol = 1e-6;

// The different types on signals for input and output layer
enum SignalClass {
    NL, // Negative Large
    NS, // Negative Small
    Z,  // Zero
    PS, // Positive Small
    PL  // Positive Large

};

// Matrix That stores the input to output map
using RuleMatrix = std::vector<std::vector<SignalClass>>;


class MembFun_Triangular
{
private:
    double m_left, m_center, m_right;

    void validateConfig()
    {
        assert(m_right > m_left);

        assert((m_center - m_left >= g_eTol) &&
               (m_right - m_center >= g_eTol));
    }

public:
    MembFun_Triangular(double left,
                       double center,
                       double right)
        : m_left(left), m_center(center),m_right(right)

    {validateConfig();}

    double membership(double x)
    {
        if (x < m_left || x > m_right) {return 0.0;}

        else if (std::fabs(x - m_center) < g_eTol){return 1.0;}

        else if (x < m_center)
        {
            return (x - m_left) /
                   (m_center - m_left);
        }

        else
        {
            return (m_right - x) /
                   (m_right - m_center);
        }
    }
};

class MembFun_Gaussian{};
class MembFun_Trapezoidal{};

template<class MembFun>

class FuzzyLayer
{
    FuzzyLayer(MembFun f)
};