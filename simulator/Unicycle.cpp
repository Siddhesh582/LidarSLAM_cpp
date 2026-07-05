#include "Unicycle.h"
#include <algorithm>   // for std::clamp
#include <cmath>       // for cos, sin

// constructor implementation
Unicycle::Unicycle(
    float v_min,
    float v_max,
    float w_min,
    float w_max
)
: action_space_low(v_min, w_min),
  action_space_high(v_max, w_max)
{}

// step function implementation
Eigen::Vector3d Unicycle::step(
    const Eigen::Vector3d& current_state,
    const Eigen::Vector2d& action,
    float dt
){
    Eigen::Vector2d clipped_action;
    clipped_action[0] = std::clamp(action[0], action_space_low[0], action_space_high[0]);
    clipped_action[1] = std::clamp(action[1], action_space_low[1], action_space_high[1]);

    double x = current_state[0];
    double y = current_state[1];
    double theta = current_state[2];

    double v = clipped_action[0];
    double w = clipped_action[1];

    Eigen::Vector3d next_state;
    next_state[0] = x + v * std::cos(theta) * dt;
    next_state[1] = y + v * std::sin(theta) * dt;
    next_state[2] = theta + w * dt;

    return next_state;
}