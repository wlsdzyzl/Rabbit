#ifndef FACTOR_GRAPH_H
#define FACTOR_GRAPH_H
// factor graph means we could have imu pre-integration
#include "Factors/Factors.h"
#include "Frame.h"
#include "Optimization/Constraints.h"
namespace rabbit
{
namespace optimization
{
    class Optimizer
    {
        public:
        // only use lidar matching results
        void Optimize(std::vector<Frame> &frame_list, const std::vector<FrameCorrespondence> &frame_corrs,
            bool fix_begin = true, bool fix_end = false);
        void Optimize(std::vector<SE3> &pose_list, const std::vector<FrameCorrespondence> &frame_corres, 
            bool fix_begin = true, bool fix_end = false);
        // with imu and lidar results
        void Optimize(std::vector<Frame> &frame_list, const std::vector<FrameCorrespondence> &frame_corrs, const std::vector<IMUConstraint> &imu_constraints);
        
        int max_num_iterations = 4;
    };
}
}
#endif