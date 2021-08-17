#include "System/LCDetection.h"
#include "System/GraphBase.h"
namespace rabbit
{
namespace system
{
    // based on euclidean distance
    void LCDetector::NaiveDetection(std::vector<int> &candidates)
    {
        auto &frame_list = system.keyframe_list;
        auto &submap_list = system.submap_list;
        auto &submap_id_for_keyframes = system.submap_id_for_keyframes;
        Frame &f = frame_list.back();

        // use kdtree to find the cloest frames.
        Vec3List positions;
        KDTree<> kdtree;
        for(int i = 0; i< (int)(frame_list.size()) - system.sliding_window_volume_n ; ++i)
        {
            if(system.sliding_window_type == 0)
            positions.push_back((submap_list[submap_id_for_keyframes[i]].pose *
                frame_list[i].pose).translation());
            else
            positions.push_back(frame_list[i].pose.translation());
        }
        // kdtree doesn't supports updates
        kdtree.BuildTree(positions);
        Vec3 curr_pos = f.pose.translation();
        std::vector<float > dists;
        candidates.clear();
        kdtree.KnnRadiusSearch(curr_pos, candidates, dists, max_candidate_size * interval,  radius);
        if(!candidates.size()) return;
        if(interval > 1)
        {
            size_t ptr = 0;
            for(size_t i = 1; i < candidates.size(); ++i)
            {
                size_t j = 0;
                for(; j < i; ++j)
                {
                    if(std::fabs(candidates[i] - candidates[j]) < interval)
                    break;
                }
                if(j == i)
                candidates[++ptr] = candidates[i];
            }
            candidates.resize(ptr+1);
        }
        if(candidates.size() > max_candidate_size)
        candidates.resize(max_candidate_size);            
    }
    // based on Scan Context
    void LCDetector::SCDetection(std::vector<int> &candidates)
    {
        PointCloud downsized_pcd;
        auto &frame_list = system.keyframe_list;
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
        for(size_t i = 0 ; i < candidates.size(); ++i)
        {
            // filter out the neighbor frames in time and far frames in euclidean space
            if((curr_pos - frame_list[candidates[i]].pose.translation()).norm() <= radius &&
                (frame_list.size() -candidates[i] )> system.sliding_window_volume_n   )
                candidates[ptr++] = candidates[i];
        }
        if(!candidates.size()) return;
        if(interval > 1)
        {
            ptr = 0;
            for(size_t i = 1; i < candidates.size(); ++i)
            {
                size_t j = 0;
                for(; j < i; ++j)
                {
                    if(std::fabs(candidates[i] - candidates[j]) < interval)
                    break;
                }
                if(j == i)
                candidates[++ptr] = candidates[i];
            }
            candidates.resize(ptr+1);
        }
        if(candidates.size() > max_candidate_size)
        candidates.resize(max_candidate_size); 
    }    
}
}