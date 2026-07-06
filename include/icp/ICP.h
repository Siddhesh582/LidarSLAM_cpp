#pragma once

#include <Eigen/Dense>
#include <vector>
#include <utility>

class ICP{
    public:
        //constructor
        ICP(int d_max = 15,             //maximum distance for correspondence
            int num_iterations = 100,   //maximum ICP iterations
            double epsilon = 1e-4       //min RMSE to stop
        ): d_max(d_max),
        num_iterations(num_iterations),
        epsilon(epsilon)
        {}

        std::pair<Eigen::Vector2d, Eigen::Matrix2d> run(
            const std::vector<Eigen::Vector2d>& source_ptcloud,     //source point cloud --> list of 2D points
            const std::vector<Eigen::Vector2d>& target_ptcloud,    //target point cloud --> list of 2D points
            Eigen::Vector2d t0,     //translation
            Eigen::Matrix2d R0,     //rotation
            bool show = false,
            bool pause = false,
            bool verbose = false
        );

    private:
        int d_max;
        int num_iterations;
        double epsilon;

        // estimate correspondences b/w source and target point clouds
        std::vector<std::pair<int, int>> estimate_correspondences(
            const std::vector<Eigen::Vector2d>& source_ptcloud,     // X
            const std::vector<Eigen::Vector2d>& target_ptcloud,     // Y
            const Eigen::Vector2d& t, 
            const Eigen::Matrix2d& R
        );

        // compute optimal rigid registration (translation and rotation) given correspondences
        std::pair<Eigen::Vector2d, Eigen::Matrix2d> compute_optimal_rigid_registration (
            const std::vector<Eigen::Vector2d>& source_ptcloud,     //X
            const std::vector<Eigen::Vector2d>& target_ptcloud,     //Y
            const std::vector<std::pair<int, int>>& C               //corresponding indices list(int, int)
        );

        // compute RMSE given correspondences and transformation
        double RMSE(
            const std::vector<Eigen::Vector2d>& source_ptcloud,
            const std::vector<Eigen::Vector2d>& target_ptcloud,
            const std::vector<std::pair<int, int>>& C,
            const Eigen::Vector2d& t,
            const Eigen::Matrix2d& R
        );

};