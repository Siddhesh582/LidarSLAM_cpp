#pragma once

#include <Eigen/Dense>
#include <numbers>
#include "MotionModel.h"

class Unicycle : public MotionModel {
public:
    Eigen::Vector2d action_space_low;
    Eigen::Vector2d action_space_high;

    // constructor
    Unicycle(
        float v_min = 0.0f,
        float v_max = 1.0f,
        float w_min = -2.0f * std::numbers::pi,
        float w_max =  2.0f * std::numbers::pi
    );

    // override step function
    Eigen::Vector3d step(
        const Eigen::Vector3d& current_state,
        const Eigen::Vector2d& action,
        float dt = 0.1f
    ) override;
};