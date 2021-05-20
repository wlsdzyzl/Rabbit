#include "Optimization/FactorGraph.h"

namespace rabbit
{
namespace optimization
{
    void Optimizer::Optimize(std::vector<Frame> &frame_list, const std::vector<FrameCorrespondence> &frame_corrs)
    {
        ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
        ceres::Problem::Options problem_options;
        ceres::Problem problem(problem_options);
        for(size_t i = 0; i != frame_list.size(); ++i)
        {
            problem.AddParameterBlock(frame_list[i].pose.data(), SE3::num_parameters, 
                new factor::LocalParameterization() );
        }

        for(size_t i = 0; i != frame_corrs.size(); ++i)
        {
            if(frame_corres[i].is_valid)
            {
                int new_id = frame_corres[i].new_id;
                int old_id = frame_corres[i].old_id;
                // relative_pose: from target to source
                ceres::CostFunction *cost_function = 
                    factor::PoseEdgeFactor::Create(frame_corres.relative_pose);
                problem.AddResidualBlock(cost_function, loss_function, frame_list[new_id].pose.data(), frame_list[old_id].pose.data());
            }
        }
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::ITERATIVE_SCHUR;
        options.max_num_iterations = max_num_iterations;
        options.minimizer_progress_to_stdout = false;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
    }
    void Optimizer::Optimize(std::vector<Frame> &frame_list, const std::vector<FrameCorrespondence> &frame_corrs, const std::vector<IMUConstraint> &imu_constraints)
    {
    }
}
}