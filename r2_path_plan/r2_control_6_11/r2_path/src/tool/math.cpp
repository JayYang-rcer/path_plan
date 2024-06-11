#include "r2_path/tool/math.h"
#include <cmath>

namespace math_ns
{
    unsigned long long math::factorial(int n)
	{
		if (n <= 1) return 1;
		return n * factorial(n - 1);
	}

    
    float math::abs_limit(float x, float ABS_MAX)
    {
        if(x>ABS_MAX)
        {
            return ABS_MAX;
        }
        else if(x<-ABS_MAX)
        {
            return -ABS_MAX;
        }
        else
        {
            return x;
        }
    }

    double math::distance_of_two_point(double x1, double y1, double x2, double y2)
    {
        return sqrt(pow(x1-x2,2)+pow(y1-y2,2));
    }
} // namespace math_ns