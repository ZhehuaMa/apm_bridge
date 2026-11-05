// Second translation unit for quadratic_thrust_model_test: including the header from
// two TUs of one binary must link (all header definitions are inline, ODR-safe).
#include "quadratic_thrust_model.h"

double quadratic_thrust_model_second_tu(const quadratic_thrust_model::MotorParams &params,
                                         double force)
{
    return quadratic_thrust_model::forceToThrust(params, force, 1.0) +
           quadratic_thrust_model::thrustToForce(params, 0.0, 1.0) * 0.0;
}
