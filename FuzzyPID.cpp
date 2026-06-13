


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
    double left, center, right;

    void validateConfig()
    {
        assert(right > left);

        assert((center > left) && (right > center));
    }

public:
    MembFun_Triangular(double left,
                       double center,
                       double right)
        : left(left), center(center),right(right)

    {validateConfig();}

    double membership(double x)
    {
        if (x < left || x > right) return 0.0;

        else if (x == center) return 1.0;

        else if (x < center) return (x - left) / (center - left);

        else  return (right - x) / right - center;
    }
};

