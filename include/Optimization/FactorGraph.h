#ifndef FACTOR_GRAPH_H
#define FACTOR_GRAPH_H
// factor graph means we could have imu pre-integration
#include "Factors/Factors.h"
namespace rabbit
{
namespace optimization
{
    class Optimizer
    {
        public:
        // only use lidar matching results
        void  Optimize(std::vector<Frame> &frame_list, const std::vector<FrameCorrespondence> &frame_corrs);
        // with imu and lidar results
        void  Optimize(std::vector<Frame> &frame_list, const std::vector<FrameCorrespondence> &frame_corrs, const std::vector<IMUConstraint> &imu_constraints);
        int max_num_iterations = 4;
    };
}
}
#endif