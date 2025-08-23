#include "unicycle_model.hpp"

namespace limo
{
    UnicycleController::UnicycleController(double k_l2, double k_alpha, double k_beta,
                                           double v_max, double w_max,
                                           double eps_l2, double eps_theta)
        : k_l2_(k_l2), k_alpha_(k_alpha), k_beta_(k_beta),
          v_max_(v_max), w_max_(w_max),
          eps_l2_(eps_l2), eps_theta_(eps_theta)
    {
    }

    UnicycleControlInput UnicycleController::error_to_control_input(const UnicycleKinematicState &error) const
    {
        UnicycleControlInput u{0.0, 0.0};

        // Distance and angle to goal position
        double l2 = std::sqrt(error.x * error.x + error.y * error.y);
        double alpha = std::atan2(error.y, error.x);

        // Current-goal angular error
        double beta = wrap_to_pi(error.theta - alpha);

        u.v = std::clamp(k_l2_ * l2, 0.0, v_max_);
        u.omega = std::clamp(k_alpha_ * alpha + k_beta_ * beta, -w_max_, w_max_);

        if (l2 < eps_l2_ && std::abs(error.theta) < eps_theta_)
        {
            u.v = 0.0;
            u.omega = 0.0;
        }

        return u;
    }

    double UnicycleController::wrap_to_pi(double angle)
    {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;
        while (angle < -M_PI)
            angle += 2.0 * M_PI;
        return angle;
    }
} // namespace limo