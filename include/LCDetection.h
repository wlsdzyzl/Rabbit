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
    class LCDetector
    {
        public:
        LCDetector(std::vector<Frame> &_frame_list, double _filter_size = 0.5):frame_list(_frame_list), filter_size(_filter_size)
        {
            down_size_filter.setLeafSize(filter_size, filter_size, filter_size);
        }
        void SetLeafSize(double _filter_size)
        {
            filter_size = _filter_size;
            down_size_filter.setLeafSize(filter_size, filter_size, filter_size);
        }
        // based on euclidean distance
        void NaiveDetection(std::vector<int> &candidates)
        {
            Frame &f = frame_list.back();

            // use kdtree to find the cloest frames.
            Vec3List positions;
            KDTree<> kdtree;
            for(int i = 0; i< (int)(frame_list.size()) - neighbor_size ; ++ i)
            {
                positions.push_back(frame_list[i].pose.translation());
            }
            // kdtree doesn't supports updates
            kdtree.BuildTree(positions);
            Vec3 curr_pos = f.pose.translation();
            std::vector<float > dists;
            candidates.clear();
            kdtree.KnnRadiusSearch(curr_pos, candidates, dists, max_candidate_size,  radius);
        }
        // based on Scan Context
        void SCDetection(std::vector<int> &candidates)
        {
            PointCloud downsized_pcd;
            Frame &f = frame_list.back();

            if(frame_list.size() !=sc_manager.database_size() )
            down_size_filter.setInputCloud(f.pcd);
            down_size_filter.filter(downsized_pcd);

            sc_manager.makeAndSaveScancontextAndKeys(downsized_pcd);
            auto results = sc_manager.detectLoopClosureIDs();
            candidates.resize(results.size());
            for(size_t i = 0; i != candidates.size(); ++i)
            candidates[i] = std::get<0>(results[i]);

            Vec3 curr_pos = f.pose.translation();
            size_t ptr = 0;
            for(size_t i = 0 ; i != candidates.size(); ++i)
            {
                // filter out the neighbor frames in time and far frames in euclidean space
                if((curr_pos - frame_list[candidates[i]].pose.translation()).norm() <= radius &&
                    (frame_list.size() -candidates[i] )> neighbor_size   )
                    candidates[ptr++] = candidates[i];
            }
            candidates.resize(ptr);
            // set to max_candidates
            if(ptr > max_candidate_size)
            candidates.resize(max_candidate_size);

        }
        std::vector<Frame> &frame_list;
        SCManager sc_manager;
        double filter_size = 0.5; 
        pcl::VoxelGrid<PointType>  down_size_filter;
        size_t max_candidate_size = 5;
        int neighbor_size = 25;
        double radius = 15.0;
        // int max_results = 20;
    };
}
#endif