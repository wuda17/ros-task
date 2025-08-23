#pragma once

#include <cmath>
#include <algorithm>

namespace limo
{
    struct UnicycleKinematicState
    {
        double x;
        double y;
        double theta;
    };

    struct UnicycleControlInput
    {
        double v;
        double omega;
    };

    class UnicycleController
    {
    public:
        UnicycleController(double k_l2, double k_alpha, double k_beta,
                           double v_max, double w_max,
                           double eps_l2 = 0.01, double eps_theta = 0.01);

        UnicycleControlInput error_to_control_input(const UnicycleKinematicState &error) const;

    private:
        // Gains
        double k_l2_;
        double k_alpha_;
        double k_beta_;

        // Limits
        double v_max_;
        double w_max_; // in radians

        // Goal tolerance
        double eps_l2_;
        double eps_theta_;

        static double wrap_to_pi(double angle);
    };
} // namespace limo