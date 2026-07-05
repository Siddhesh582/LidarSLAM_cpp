#pragma once

#include <Eigen/Dense>

// Abstract class for 2d mobile robot motion model
class MotionModel {
    public:
        virtual ~MotionModel() = default;

        //step function
        virtual Eigen::Vector3d step(
            const Eigen::Vector3d& current_state,  //[x, y, theta]
            const Eigen::Vector2d& action,     //[vx, vw]
            float dt=0.1    
        ) = 0;
};