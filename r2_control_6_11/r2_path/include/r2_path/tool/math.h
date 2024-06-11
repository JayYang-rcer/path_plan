#ifndef __MATH_H
#define __MATH_H

#include "iostream"
using namespace std;



namespace math_ns
{
    class math
    {
    private:
        /* data */
    public:
        unsigned long long factorial(int n);
        float abs_limit(float x, float ABS_MAX);
        double distance_of_two_point(double x1, double y1, double x2, double y2);
    };
} // namespace math_ns


#endif