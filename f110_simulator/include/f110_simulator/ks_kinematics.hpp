#pragma once

#include "f110_simulator/car_state.hpp"
#include "f110_simulator/car_params.hpp"

namespace racecar_simulator {

class KSKinematics {

public:

    static CarState update(
            const CarState start,
            double accel,
            double steer_angle_vel,
            CarParams p,
            double dt);
};

}
