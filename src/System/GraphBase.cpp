#include "System/GraphBase.h"
namespace rabbit
{
namespace system
{
    using namespace optimization;
    bool GraphBase::NewKeyframe(const SE3 &delta_pose)
    {
        Vec3 rotation_vector = delta_pose.log().block<3, 1>(0, 0);
        Vec3 translation = delta_pose.translation();
        return util::Rad2Deg(rotation_vector.norm()) > angle_threshold 
            || translation.norm() > distance_threshold;
    }
    void GraphBase::AddNewFrame(const  Frame &new_frame)
    {
        Frame f = new_frame;
        f.frame_id = relative_pose_list.size();
        // compute loam feature
        f.ComputeLOAMFeature();

        if(f.frame_id == 0)
        {
            f.pose = SE3();
            relative_pose_list.push_back(SE3());
            keyframe_list.push_back(f);
            return;
        }
        Frame &last_keyframe = keyframe_list.back();
        SE3 relative_T = relative_pose_list.back(); 
        bool matching_success = odometry.Loam(f, last_keyframe, relative_T);
        if(matching_success)
        {
            if(!NewKeyframe(relative_T)) relative_pose_list.push_back(relative_T);
            else
            {
                std::cout<<"keyframe: "<<f.frame_id<<std::endl;
                int curr_keyframe_id = keyframe_list.size();
                f.pose = last_keyframe.pose * relative_T;
                keyframe_list.push_back(f);
                relative_pose_list.push_back(SE3());
                keyframe_corrs.push_back(FrameCorrespondence(curr_keyframe_id , curr_keyframe_id - 1, relative_T, true));
                
                // loop closure detection
                // std::vector<int> candidate_ids;
                // lcdetector.SCDetection(candidate_ids);
                // for(size_t i = 0; i != candidate_ids.size(); ++i)
                // {
                //     // check if the closed loop is correct;
                //     Frame &old_keyframe = keyframe_list[candidate_ids[i]];
                //     SE3 tmp_relative_T;
                //     bool tmp_matching_success = odometry.Loam(f, old_keyframe, tmp_relative_T);
                //     if(tmp_matching_success)
                //     {
                //         std::cout<<"Detect Loop closure, candidate id: "<<candidate_ids[i]<<std::endl;
                //         FrameCorrespondence tmp_corr(curr_keyframe_id, candidate_ids[i], tmp_relative_T, true);
                //         keyframe_corrs.push_back(tmp_corr);
                //     }
                // }
                // pose optimization
                // Optimize();
            }
        }
        else 
        {
            std::cout<<YELLOW<<"[WARNING]::[GraphBase]::Track last keyframe failed."<<RESET<<std::endl;
        }
        
    }
    void GraphBase::Optimize()
    {
        if(!with_imu)
        optimizer.Optimize(keyframe_list, keyframe_corrs);
    }
}
}