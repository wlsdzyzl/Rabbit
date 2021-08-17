#ifndef RABBIT_LCD_H
#define RABBIT_LCD_H
/*
In lidar slam, an naive lcd based on the Euclidean distance is effective. 
More advanced methods such as scan context is also useful, but if the lidar point is sparse,
the result is not reliable. Becuase detecting a false loop closure is much worse than missing a true loop closure.
therefore a lot of the-state-of-the-art systems just use a naive lcd (A-LOAM, LEGO-LOAM, LIO-SAM).
In rabbit, we also implement a naive loop closure detection.
*/

#include "Frame.h"
#include "ScanContext/Scancontext.h"
#include "Utils/KDTree.h"
#include "Odometry.h"
namespace rabbit
{
namespace system
{
    class GraphBase;
    class LCDetector
    {
        public:
        LCDetector(GraphBase &_sys, 
            double _filter_size = 0.5):system(_sys), filter_size(_filter_size)
        {
            down_size_filter.setLeafSize(filter_size, filter_size, filter_size);
        }
        void SetLeafSize(double _filter_size)
        {
            filter_size = _filter_size;
            down_size_filter.setLeafSize(filter_size, filter_size, filter_size);
        }
        // based on euclidean distance
        void NaiveDetection(std::vector<int> &candidates);
        // based on Scan Context
        void SCDetection(std::vector<int> &candidates);
        GraphBase &system;
        SCManager sc_manager;
        double filter_size = 0.5; 
        pcl::VoxelGrid<PointType>  down_size_filter;
        size_t max_candidate_size = 5;
        int interval = 5;
        double radius = 20.0;
        // int max_results = 20;
    };
}
}
#endif