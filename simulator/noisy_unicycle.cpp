// Discrete time unicycle kinematic model w/ process noise

#include <Eigen/Dense>
#include <numbers>
#include <random>
#include "Unicycle.h"

// add process noise during step() function
class NoisyUnicycle : public Unicycle {
    public:

        // constructor -- automatically called when an object is created of this class 
        NoisyUnicycle(
            const Eigen::Vector3d& process_noise_limits = Eigen::Vector3d(0.02, 0.02, 0.1),  //[x,y,theta] noise limits
            float v_min = 0.0f,
            float v_max = 1.0f,
            float w_min = -2.0f * std::numbers::pi,
            float w_max =  2.0f * std::numbers::pi
        ) : Unicycle(v_min, v_max, w_min, w_max),         // base class constructor
            process_noise_limits_(process_noise_limits),   //member variable
            gen_(std::random_device{}())
        {}   

        // step function with process noise 
        Eigen::Vector3d step(
            const Eigen::Vector3d& current_state, 
            const Eigen::Vector2d& action, 
            float dt=0.1f
        ) override {
            // base class step
            Eigen::Vector3d next_state = Unicycle::step(current_state, action, dt);

            //add component-wise noise to next state
            Eigen::Vector3d process_noise;

            for (int i =0; i < 3; i++){
                std::uniform_real_distribution<double> dist(
                    -process_noise_limits_[i],
                    process_noise_limits_[i]
                );
                process_noise[i] = dist(gen_);  //add random number at each index to make noise vector
            }

            // add process noise (vector) to the next state
            Eigen::Vector3d perturbed_next_state = next_state + process_noise;

            return perturbed_next_state;
        }

    private:
        Eigen::Vector3d process_noise_limits_;  // member variable to hold noise limits
        std::mt19937 gen_;  // random number generator

};
