//
// Created by melody on 2022/9/15.
//

#include "Solve.h"
#include <ceres/ceres.h>

Solver::Solver() {}

Solver::~Solver() {}

void Solver::Calib(std::vector<std::pair<Pose2D,Pose2D>> &sync_data, double *data) {
    ceres::Problem problem;
    // ceres::LossFunction loss_function = ceres::CauchyLoss(1.0);
    for (size_t i = 0; i < sync_data.size(); i++) {
        printf("odom : x = %f,y = %f, phi = %f scan_odom : x = %f, y = %f, phi = %f\n",
               sync_data[i].first.x,sync_data[i].first.y,sync_data[i].first.Phi(),
               sync_data[i].second.x,sync_data[i].second.y,sync_data[i].second.Phi());

        problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<CostFunctor, 3,3>(
                        new CostFunctor(sync_data[i].second.x,sync_data[i].second.y,sync_data[i].second.Phi(),
                                        sync_data[i].first.x,sync_data[i].first.y,sync_data[i].first.Phi())),nullptr,
                data
        );
    }
    ceres::Solver::Options options;
    options.max_num_iterations           = 50;
    options.linear_solver_type           = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    printf("Initial l_x: %f l_y: %f l_theta: %f,Final l_x: %lf l_y: %lf l_theta: %lf", 0.0, 0.0, 0.0, data[0], data[1],data[2]);
}