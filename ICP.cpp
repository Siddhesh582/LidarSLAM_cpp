#include <iostream>
#include <Eigen/Dense>
#include <optional>
#include <vector>

class ICP{
    public:

        //constructor
        ICP(int d_max = 15,             //maximum distance for correspondence
            int num_iterations = 100,   //maximum ICP iterations
            double epsilon = 1e-4       //min RMSE to stop
        );     

        std::pair<Eigen::Vector2d, Eigen::Matrix2d> run(
            const std::vector<Eigen::Vector2d>& source_ptcloud,     //source point cloud --> list of 2D points
            const std::vector<Eigen::Vector2d>& target_ptcloud,    //target point cloud --> list of 2D points
            Eigen::Vector2d t0,     //translation
            Eigen::Matrix2d R0,     //rotation
            bool show = false,
            bool pause = false,
            bool verbose = false
        )
        {
            std::optional<double> prev_rmse;

            for (int i=0; i < num_iterations; i++){

                //STEP 1: get the correspondences from point clouds --> list(int, int) 
                std::vector<std::pair<int, int>> C = estimate_correspondences(source_ptcloud, 
                        target_ptcloud, 
                        t0, 
                        R0);

                //STEP 2: get the next transformation t, R
                std::pair<Eigen::Vector2d, Eigen::Matrix2d> next_T = compute_optimal_rigid_registration(source_ptcloud, target_ptcloud, C);

                //STEP 3: compute RMSE 
                double rmse = RMSE(source_ptcloud, 
                    target_ptcloud, C, 
                    next_T.first, 
                    next_T.second);

                if(verbose){
                    std::cout << "Iteration:" << i << ": RMSE =" << rmse << std::endl;
                }

                //update current estimate of transformation
                t0 = next_T.first;
                R0 = next_T.second;

                //check if rmse improved
                if (prev_rmse.has_value() && 
                    (*prev_rmse - rmse) < epsilon)
                {
                    if (verbose)
                    {
                        std::cout << "Iteration " << i << ": RMSE = " << rmse << std::endl;
                    }
                    break;
                }

                //store current rmse for next iteration 
                prev_rmse = rmse;

                // Max iterations reached
                if(i == num_iterations - 1 && verbose){
                    std::cout << "Max iterations reached: " << num_iterations << std::endl;
                    std::cout << "Converged at iteration " << i << " with RMSE " << rmse << std::endl;
                }
            }

            return {t0, R0};
        }

    private:
        int d_max = 15;              //maximum distance for correspondence
        int num_iterations = 100;     //maximum ICP iterations
        double epsilon = 1e-4;         //min RMSE to stop

        // estimate correspondences b/w source and target point clouds
        std::vector<std::pair<int, int>> estimate_correspondences(
            const std::vector<Eigen::Vector2d>& source_ptcloud,     // X
            const std::vector<Eigen::Vector2d>& target_ptcloud,     // Y
            Eigen::Vector2d t, 
            Eigen::Matrix2d R
        )
        {   
            //list of correspondences (index from X, index from Y)
            std::vector<std::pair<int, int>> C; 

            for (int n=0; n<source_ptcloud.size(); n++)
            {
                Eigen::Vector2d Xt = R * source_ptcloud[n]+ t;

                double best_d = std::numeric_limits<double>::max();
                int best_j = -1;

                for (int j=0; j<target_ptcloud.size(); j++)
                {
                    double d = (Xt - target_ptcloud[j]).norm();  //list of all differences
                    
                    //find the minimum distance and corresponding index
                    if (d < best_d)
                    {
                        best_d = d;     // least distance
                        best_j = j;     //index from Y (target point cloud)
                    }
                }

                // if best distance < threshold
                if (best_d < d_max)
                {
                    C.push_back(std::make_pair(n, best_j));  //(index from X, index from Y)
                }

            }
              
        }

        // compute optimal rigid registration (translation and rotation) given correspondences
}; 