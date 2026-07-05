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
            const Eigen::Vector2d& t, 
            const Eigen::Matrix2d& R
        )
        {   
            //list of correspondences (index from X, index from Y)
            std::vector<std::pair<int, int>> C; 

            for (size_t n=0; n<source_ptcloud.size(); n++)
            {
                Eigen::Vector2d Xt = R * source_ptcloud[n]+ t;

                double best_d = std::numeric_limits<double>::max();
                int best_j = -1;

                for (size_t j=0; j<target_ptcloud.size(); j++)
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

            if (C.empty())
            {
                throw std::runtime_error("No correspondences found.");
            }

            return C;
              
        }

        // compute optimal rigid registration (translation and rotation) given correspondences
        std::pair<Eigen::Vector2d, Eigen::Matrix2d> compute_optimal_rigid_registration (
            const std::vector<Eigen::Vector2d>& source_ptcloud,     //X
            const std::vector<Eigen::Vector2d>& target_ptcloud,     //Y
            const std::vector<std::pair<int, int>>& C               //corresponding indices list(int, int)
        )
        {
            std::vector<Eigen::Vector2d> X_from_C;
            std::vector<Eigen::Vector2d> Y_from_C;

            //store the corresponding points using indices from C
            for (const auto& [i, j] : C)
            {
                X_from_C.push_back(source_ptcloud[i]);
                Y_from_C.push_back(target_ptcloud[j]);
            }

            //compute centroids
            Eigen::Vector2d X_centroid = Eigen::Vector2d::Zero();
            Eigen::Vector2d Y_centroid = Eigen::Vector2d::Zero();

            for (const auto& point: X_from_C)
            {
                X_centroid += point;
            }

            for (const auto& point: Y_from_C)
            {
                Y_centroid += point;
            }

            X_centroid /= X_from_C.size();
            Y_centroid /= Y_from_C.size();

            // vector of points centered at the centroid
            std::vector<Eigen::Vector2d> X_prime;
            std::vector<Eigen::Vector2d> Y_prime;
            
            for (const auto& point : X_from_C)
            {
                X_prime.push_back(point - X_centroid);
            }

            for (const auto& point : Y_from_C)
            {
                Y_prime.push_back(point - Y_centroid);
            }

            // covariance/cross correlation matrix --> 2x2 matrix (how X, Y are correlated after centering)
            Eigen::Matrix2d W = Eigen::Matrix2d::Zero();

            for (size_t k=0; k<C.size(); k++)
            {
                W += Y_prime[k] * X_prime[k].transpose();   //X_prime, Y_prime are already aligned
            }

            //decompose W using SVD --> U, sigma, V
            Eigen::JacobiSVD<Eigen::Matrix2d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);

            Eigen::Matrix2d U = svd.matrixU();
            Eigen::Matrix2d V = svd.matrixV();              //Eigen gives V (Numpy gives Vt)

            //compute rotation matrix
            Eigen::Matrix2d R = V * U.transpose();

            if (R.determinant() < 0)
            {
                V.col(1) *= -1;
                R = V * U.transpose();
            }

            //compute translation vector --> rotate X and subtract from Y to get the difference
            Eigen::Vector2d t = Y_centroid - R * X_centroid;

            return {t, R};
        }

        // compute RMSE given correspondences and transformation
        double RMSE(
            const std::vector<Eigen::Vector2d>& source_ptcloud,
            const std::vector<Eigen::Vector2d>& target_ptcloud,
            const std::vector<std::pair<int, int>>& C,
            const Eigen::Vector2d& t,
            const Eigen::Matrix2d& R
        )
        {
            double total_error = 0.0;

            for (const auto& [i, j]: C)
            {
                Eigen::Vector2d Xt = R * source_ptcloud[i] + t;
                double error = (Xt - target_ptcloud[j]).squaredNorm();
                total_error += error;
            }

            double rmse = std::sqrt(total_error / C.size());

            return rmse;
        }



};