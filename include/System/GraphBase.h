#ifndef RABBIT_GRAPH_BASE_H
#define RABBIT_GRAPH_BASE_H
#include "Frame.h"
#include "Odometry.h"
#include  "LCDetection.h"
#include "Optimization/FactorGraph.h"

namespace rabbit
{
namespace system
{
    using namespace util;
    class GraphBase
    {
        public:
        GraphBase():lcdetector(keyframe_list){}
        //check the pose difference to decide if we should add a new keyframe
        bool NewKeyframe();
        void AddNewFrame(const Frame &f);
        void Optimize();
        SE3List relative_pose_list;
        std::vector<Frame> keyframe_list;
        LidarOdometry odometry;
        double angle_threshold = 10;
        double distance_threshold =1.0; 
        double lcd_leaf_size = 0.5;
        optimization::Optimizer optimizer;
        LCDetector lcdetector;
        bool with_imu = false;
    };
}
}
#endif