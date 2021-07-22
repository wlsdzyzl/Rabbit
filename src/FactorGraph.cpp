#include "Optimization/FactorGraph.h"

namespace rabbit
{
namespace optimization
{
    void Optimizer::Optimize(std::vector<Frame> &frame_list, const std::vector<FrameCorrespondence> &frame_corres, bool fix_begin, bool fix_end)
    {
        ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
        ceres::Problem::Options problem_options;
        ceres::Problem problem(problem_options);
        for(size_t i = 0; i != frame_list.size(); ++i)
        {
            problem.AddParameterBlock(frame_list[i].pose.data(), SE3::num_parameters, 
                new factor::LocalParameterizationSE3() );

            if(fix_begin && i == 0) problem.SetParameterBlockConstant(frame_list[i].pose.data());
            if(fix_end && i == frame_list.size() - 1) problem.SetParameterBlockConstant(frame_list[i].pose.data());
        }

        for(size_t i = 0; i != frame_corres.size(); ++i)
        {
            if(frame_corres[i].is_valid)
            {
                int new_id = frame_corres[i].new_id;
                int old_id = frame_corres[i].old_id;
                // relative_pose: from target to source
                ceres::CostFunction *cost_function = 
                    factor::PoseEdgeFactor::Create(frame_corres[i].relative_pose, frame_corres[i].weight);
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
    void Optimizer::Optimize(std::vector<SE3> &pose_list, const std::vector<FrameCorrespondence> &frame_corres, bool fix_begin, bool fix_end)
    {
        ceres::LossFunction *loss_function = nullptr;
        ceres::Problem::Options problem_options;
        ceres::Problem problem(problem_options);
        for(size_t i = 0; i != pose_list.size(); ++i)
        {
            problem.AddParameterBlock(pose_list[i].data(), SE3::num_parameters, 
                new factor::LocalParameterizationSE3() );

            if(fix_begin && i == 0) problem.SetParameterBlockConstant(pose_list[i].data());
            if(fix_end && i == pose_list.size() - 1) problem.SetParameterBlockConstant(pose_list[i].data());
        }

        for(size_t i = 0; i != frame_corres.size(); ++i)
        {
            if(frame_corres[i].is_valid)
            {
                int new_id = frame_corres[i].new_id;
                int old_id = frame_corres[i].old_id;
                // relative_pose: from target to source
                ceres::CostFunction *cost_function = 
                    factor::PoseEdgeFactor::Create(frame_corres[i].relative_pose, frame_corres[i].weight);
                problem.AddResidualBlock(cost_function, loss_function, pose_list[new_id].data(), pose_list[old_id].data());
            }
        }
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::ITERATIVE_SCHUR;
        options.max_num_iterations = max_num_iterations;
        options.minimizer_progress_to_stdout = false;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
    }
    void Optimizer::Optimize(std::vector<Frame> &frame_list, const std::vector<FrameCorrespondence> &frame_corres, const std::vector<IMUConstraint> &imu_constraints)
    {
    }
}
}