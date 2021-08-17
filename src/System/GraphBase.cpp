#include "System/GraphBase.h"
#include "Utils/IO.h"
namespace rabbit
{
namespace system
{
    using namespace optimization;
    bool GraphBase::NewKeyframe(const SE3 &delta_pose)
    {
        Vec3 rotation_vector = delta_pose.log().block<3, 1>(3, 0);
        Vec3 translation = delta_pose.translation();
        // std::cout<<"angle: "<<util::Rad2Deg(rotation_vector.norm())<<" "<<rotation_vector.norm()<<std::endl;
        // std::cout<<"distance: "<<translation.norm()<<std::endl;
        return util::Rad2Deg(rotation_vector.norm()) > angle_threshold 
            || translation.norm() > distance_threshold;
    }
    bool GraphBase::Matching(const Frame &s, const Frame &t, SE3 &T)
    {
        double mean_dist, inlier_ratio;
        if(matching_method == OdometryMethod::LOAM)
        {
            std::tie(mean_dist, inlier_ratio) = odometry.Loam(s, t, T, matching_score_threshold);
        }
        else if(matching_method == OdometryMethod::ICP)
        {
            std::tie(mean_dist, inlier_ratio) = odometry.ICP(s, t, T, matching_score_threshold);
        }
        else if(matching_method == OdometryMethod::GICP)
        {
            std::tie(mean_dist, inlier_ratio) = odometry.GICP(s, t, T, matching_score_threshold);
        }
        else if(matching_method == OdometryMethod::NDT)
        {
            std::tie(mean_dist, inlier_ratio) = odometry.NDT(s, t, T, matching_score_threshold);
        }
        else if(matching_method == OdometryMethod::NDTOMP)
        {
            std::tie(mean_dist, inlier_ratio) = odometry.NDTOMP(s, t, T, matching_score_threshold);
        }
        else if(matching_method == OdometryMethod::GICPOMP)
        {
            std::tie(mean_dist, inlier_ratio) = odometry.GICPOMP(s, t, T, matching_score_threshold);
        }
        return inlier_ratio > matching_inlier_ratio;
    }
    bool GraphBase::Mapping(const Frame &s, const Frame &t, SE3 &T)
    {
        // if(use_ground_priority) GroundMapping(s, t, T);
        double mean_dist, inlier_ratio;
        if(mapping_method == OdometryMethod::LOAM)
        {
            std::tie(mean_dist, inlier_ratio) = odometry.LoamMapping(s, t, T, mapping_score_threshold);
        }
        else if(mapping_method == OdometryMethod::ICP)
        {
            std::tie(mean_dist, inlier_ratio) = odometry.ICP(s, t, T, mapping_score_threshold);
        }
        else if(mapping_method == OdometryMethod::GICP)
        {
            std::tie(mean_dist, inlier_ratio) = odometry.GICP(s, t, T, mapping_score_threshold);
        }
        else if(mapping_method == OdometryMethod::NDT)
        {
            std::tie(mean_dist, inlier_ratio) = odometry.NDT(s, t, T, mapping_score_threshold);
        } 
        else if(mapping_method == OdometryMethod::GICPOMP)
        {
            std::tie(mean_dist, inlier_ratio) = odometry.GICPOMP(s, t, T, mapping_score_threshold);
        }
        else if(mapping_method == OdometryMethod::NDTOMP)
        {
            std::tie(mean_dist, inlier_ratio) = odometry.NDTOMP(s, t, T, mapping_score_threshold);
        }
        return inlier_ratio > mapping_inlier_ratio;
    }
    /*
    bool GraphBase::GroundMapping(const Frame &s, const Frame &t, SE3 &T)
    {
        
        if(mapping_method == OdometryMethod::LOAM)
        {
            return odometry.LoamGroundMapping(s, t, T);
        }
        else if(mapping_method == OdometryMethod::ICP)
        {
            return odometry.ICP(s, t, T);
        }
        else if(mapping_method == OdometryMethod::GICP)
        {
            return odometry.GICP(s, t, T);
        }
        else if(mapping_method == OdometryMethod::NDT)
        {
            return odometry.NDT(s, t, T);
        }         
    }
    */
    std::vector<int> GraphBase::GetFrameIDInSlidingWindow()
    {
        std::vector<int> ids;
        if(sliding_window_type == 0 || sliding_window_type == 1)
        {
            int start = (keyframe_list.size() >= sliding_window_volume_n) ? (keyframe_list.size() - sliding_window_volume_n):0;
            for(int i = 0; i < sliding_window_volume_n && i + start < keyframe_list.size(); ++i)
            ids.push_back(i + start);
        }
        else if(sliding_window_type == 2)
        {
            // Vec3 pos = keyframe_list.back().pose.translation();
            // std::vector<int> cube_ids;
            // moving_box.GetCloseCubeIDs(pos, cube_ids);
            // for(size_t i = 0; i != cube_ids.size(); ++i)
            // ids.insert(ids.end(), moving_box.cubes[cube_ids[i]].begin(), 
            //     moving_box.cubes[cube_ids[i]].end());
            for(size_t i = 0; i != moving_box.cubes.size(); ++i)
            ids.insert(ids.end(), moving_box.cubes[i].begin(), 
                moving_box.cubes[i].end());
        }
        return ids;

    }
    SE3 GraphBase::AddNewFrame(const Frame &new_frame)
    {
        if(sliding_window_type == 0) 
        {
            return AddNewFrameSubmap(new_frame);
        }
        else 
        {
            return AddNewFrameSlidingWindow(new_frame);
        }
    }
    SE3 GraphBase::AddNewFrameSubmap(const Frame &new_frame)
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
            submap_list.push_back(f);
            submap_id_for_keyframes.push_back(0);
            last_frame = f;
            return f.pose;
        }
        Frame &last_keyframe = keyframe_list.back();
        // this is important, use last relative pose to estimate current relative pose.
        bool matching_success = Matching(f, last_frame, relative_T_to_last_frame);
        if(!matching_success) std::cout<<YELLOW<<"[WARNING]::[GraphBase]::Track last keyframe failed."<<RESET<<std::endl;
        f.pose = (submap_list.back().pose * last_frame.pose) * relative_T_to_last_frame;
        relative_T_to_last_keyframe = (submap_list.back().pose * last_keyframe.pose).inverse() * f.pose;
        
        if(!NewKeyframe(relative_T_to_last_keyframe)) 
        {
            f.pose = submap_list.back().pose.inverse() * f.pose;
            relative_pose_list.push_back(relative_T_to_last_frame);
        }
        else
        {
            std::cout<<"keyframe: "<<f.frame_id<<std::endl;
            int curr_keyframe_id = keyframe_list.size();
            bool mapping_success = true;

            // matching current keyframe to current submap
            
            bool new_submap_flag = (keyframe_list.size() % submap_len == 0);
            if(keyframe_list.size() >= 5)
            {
                ConstructSubmapVolume(submap_list.back().frame_id, sliding_window_volume);
                mapping_success = Mapping(f, sliding_window_volume, f.pose);
            }
            if(!mapping_success) std::cout<<YELLOW<<"[WARNING]::[GraphBase]::mapping to submap failed."<<RESET<<std::endl;
            if(new_submap_flag)
            {   
                if(use_ground_priority)
                {
                    double tmp_indicator;
                    Vec3List submap_ground_points;
                    ToEigenPoints(*(submap_list.back().less_ground_points), submap_ground_points);
                    std::tie(submap_list.back().ground_plane_normal, submap_list.back().ground_plane_dist, tmp_indicator) = 
                        FitPlane(submap_ground_points);
                    std::cout<<"submap ground normal: "<<submap_list.back().ground_plane_normal.transpose()<<std::endl;
                }
                std::cout<<"new submap: "<<submap_list.size()<<std::endl;
                Frame new_submap = Frame();
                new_submap.frame_id = submap_list.size();
                new_submap.less_sharp_points = PointCloudPtr(new PointCloud());
                new_submap.less_flat_points = PointCloudPtr(new PointCloud()); 
                new_submap.less_ground_points = PointCloudPtr(new PointCloud()); 
                new_submap.sharp_points = PointCloudPtr(new PointCloud());
                new_submap.flat_points = PointCloudPtr(new PointCloud()); 
                new_submap.ground_points = PointCloudPtr(new PointCloud()); 
                new_submap.pcd = PointCloudPtr(new PointCloud());
                new_submap.ground_plane_dist = submap_list.back().ground_plane_dist;
                new_submap.ground_plane_normal = submap_list.back().ground_plane_normal;
                submap_list.push_back(new_submap);  
                if(submap_list.size() >= 2 )      
                submap_corrs.push_back(FrameCorrespondence(submap_list.size() - 1 , submap_list.size() - 2, SE3(), true));    
            }
            Frame &current_submap = submap_list.back();
            // add current keyframe to current submap
            f.pose = current_submap.pose.inverse() * f.pose;
            PointCloud tmp_pcd;
            pcl::transformPointCloud (*(f.less_sharp_points), tmp_pcd, f.pose.matrix());
            (*(current_submap.less_sharp_points)) +=  tmp_pcd;                    

            pcl::transformPointCloud (*(f.less_flat_points), tmp_pcd, f.pose.matrix());
            (*(current_submap.less_flat_points)) += tmp_pcd;           


            pcl::transformPointCloud (*(f.sharp_points), tmp_pcd, f.pose.matrix());
            (*(current_submap.sharp_points)) +=  tmp_pcd;                    

            pcl::transformPointCloud (*(f.flat_points), tmp_pcd, f.pose.matrix());
            (*(current_submap.flat_points)) += tmp_pcd;


            FilterPCD(*current_submap.less_sharp_points, *current_submap.less_sharp_points, sharp_leaf_size,
                sharp_leaf_size, sharp_leaf_size);
            FilterPCD(*current_submap.less_flat_points, *current_submap.less_flat_points, flat_leaf_size,
                flat_leaf_size, flat_leaf_size);


            FilterPCD(*current_submap.sharp_points, *current_submap.sharp_points, sharp_leaf_size,
                sharp_leaf_size, sharp_leaf_size);
            FilterPCD(*current_submap.flat_points, *current_submap.flat_points, flat_leaf_size,
                flat_leaf_size, flat_leaf_size);

            current_submap.less_sharp_kdtree =
                pcl::KdTreeFLANN<PointType>::Ptr (new pcl::KdTreeFLANN<PointType>());
            current_submap.less_flat_kdtree =
                pcl::KdTreeFLANN<PointType>::Ptr (new pcl::KdTreeFLANN<PointType>());    
            current_submap.less_sharp_kdtree->setInputCloud(current_submap.less_sharp_points);
            current_submap.less_flat_kdtree->setInputCloud(current_submap.less_flat_points);    

            if(use_ground_priority)
            {
                pcl::transformPointCloud (*(f.less_ground_points), tmp_pcd, f.pose.matrix());
                (*(current_submap.less_ground_points)) += tmp_pcd;  
                pcl::transformPointCloud (*(f.ground_points), tmp_pcd, f.pose.matrix());
                (*(current_submap.ground_points)) += tmp_pcd;  
                FilterPCD(*current_submap.less_ground_points, *current_submap.less_ground_points, flat_leaf_size,
                    flat_leaf_size, flat_leaf_size);
                FilterPCD(*current_submap.ground_points, *current_submap.ground_points, flat_leaf_size,
                    flat_leaf_size, flat_leaf_size);
                current_submap.less_ground_kdtree =
                    pcl::KdTreeFLANN<PointType>::Ptr (new pcl::KdTreeFLANN<PointType>());  
                current_submap.less_ground_kdtree->setInputCloud(current_submap.less_ground_points);     
            }

            pcl::transformPointCloud (*(f.pcd), tmp_pcd, f.pose.matrix());
            (*(current_submap.pcd)) += tmp_pcd;                

            FilterPCD(*current_submap.pcd, *current_submap.pcd, flat_leaf_size,
                flat_leaf_size, flat_leaf_size);
            // WritePLY("submap_"+std::to_string(current_submap.frame_id)+".ply", *current_submap.pcd);

            // add keyframe
            keyframe_list.push_back(f);
            relative_pose_list.push_back(SE3());  
            submap_id_for_keyframes.push_back(current_submap.frame_id); 
            // keyframe_corrs.push_back(FrameCorrespondence(curr_keyframe_id , curr_keyframe_id - 1, relative_T_to_last_keyframe, true));
            if(lcd_detection)
            {
                std::vector<int> candidate_ids;

                if(lcd_detection == 1) lcdetector.NaiveDetection(candidate_ids);
                else if (lcd_detection == 2) lcdetector.SCDetection(candidate_ids);
                std::cout<<"number of lcd candidates: "<<candidate_ids.size()<<std::endl;
                for(size_t id = 0; id != candidate_ids.size(); ++id)
                {
                    Frame &candidate_frame = keyframe_list[candidate_ids[id]];
                    Frame &candidate_submap = submap_list[submap_id_for_keyframes[candidate_ids[id]]];
                    // current_submap.pose
                    // should never happened
                    if(candidate_submap.frame_id == current_submap.frame_id) continue;
                    SE3 loop_relative_T = (candidate_submap.pose * candidate_frame.pose).inverse() * 
                        (current_submap.pose * f.pose); 
                    // std::cout<<loop_relative_T.matrix()<<"\n------------------"<<std::endl;
                    bool mapping_success_tmp = Matching(f, candidate_frame, loop_relative_T);
                    if(mapping_success_tmp)
                    {
                        Frame tmp_volume;
                        ConstructSubmapVolume(candidate_submap.frame_id, tmp_volume);
    
                        SE3 loop_relative_submap_T = candidate_frame.pose * loop_relative_T * f.pose.inverse();
                        mapping_success_tmp = Mapping(current_submap, tmp_volume, loop_relative_submap_T);
                        if(mapping_success_tmp)
                        {
                            std::cout<<"Detect Loop closure, candidate id: "<<keyframe_list[candidate_ids[id]].frame_id<<" "<<candidate_ids[id]<<std::endl;
                            FrameCorrespondence tmp_corr(candidate_submap.frame_id, current_submap.frame_id, loop_relative_submap_T, true);
                            submap_corrs.push_back(tmp_corr);
                            last_lcd_id = keyframe_list.size() - 1;                            
                        }
                    }
                    else std::cout<<"use distance threshold to filter out false loop closure detection! "<<std::endl;
                }
                
            }                
            

            // pose optimization
            OptimizeSubmap();
            
        }
        last_frame = f;
        return submap_list.back().pose * f.pose;
    }
    void GraphBase::ConstructSWVolume(const std::vector<int> &ids_in_sw, Frame &volume)
    {
        volume.less_sharp_points = PointCloudPtr(new PointCloud());
        volume.less_flat_points = PointCloudPtr(new PointCloud());
        volume.less_ground_points = PointCloudPtr(new PointCloud());

        volume.sharp_points = PointCloudPtr(new PointCloud());
        volume.flat_points = PointCloudPtr(new PointCloud());
        volume.ground_points = PointCloudPtr(new PointCloud());
        volume.pcd = PointCloudPtr(new PointCloud());
        if(sliding_window_type)
        {
            for(size_t i = 0; i != ids_in_sw.size(); ++i)
            {
                PointCloud tmp_pcd;
                int tmp_kid = ids_in_sw[i];

                pcl::transformPointCloud (*(keyframe_list[tmp_kid].sharp_points), tmp_pcd, keyframe_list[tmp_kid].pose.matrix());
                (*(volume.sharp_points)) +=  tmp_pcd;                    

                pcl::transformPointCloud (*(keyframe_list[tmp_kid].flat_points), tmp_pcd, keyframe_list[tmp_kid].pose.matrix());
                (*(volume.flat_points)) += tmp_pcd;


                pcl::transformPointCloud (*(keyframe_list[tmp_kid].less_sharp_points), tmp_pcd, keyframe_list[tmp_kid].pose.matrix());
                (*(volume.less_sharp_points)) +=  tmp_pcd;                    

                pcl::transformPointCloud (*(keyframe_list[tmp_kid].less_flat_points), tmp_pcd, keyframe_list[tmp_kid].pose.matrix());
                (*(volume.less_flat_points)) += tmp_pcd;
                if(use_ground_priority)
                {
                    pcl::transformPointCloud (*(keyframe_list[tmp_kid].ground_points), tmp_pcd, keyframe_list[tmp_kid].pose.matrix());
                    (*(volume.ground_points)) += tmp_pcd;

                    pcl::transformPointCloud (*(keyframe_list[tmp_kid].less_ground_points), tmp_pcd, keyframe_list[tmp_kid].pose.matrix());
                    (*(volume.less_ground_points)) += tmp_pcd;
                }
                pcl::transformPointCloud (*(keyframe_list[tmp_kid].pcd), tmp_pcd, keyframe_list[tmp_kid].pose.matrix());
                (*(volume.pcd)) +=  tmp_pcd;   
                FilterPCD(*volume.pcd,  
                    *volume.pcd, flat_leaf_size, 
                    flat_leaf_size, flat_leaf_size);   
            }
        }
        else
        {
            for(size_t i = 0; i != ids_in_sw.size(); ++i)
            {
                PointCloud tmp_pcd;
                int tmp_kid = ids_in_sw[i];

                pcl::transformPointCloud (*(keyframe_list[tmp_kid].sharp_points), tmp_pcd, (submap_list[submap_id_for_keyframes[tmp_kid]].pose *
                    keyframe_list[tmp_kid].pose).matrix());
                (*(volume.sharp_points)) +=  tmp_pcd;                    

                pcl::transformPointCloud (*(keyframe_list[tmp_kid].flat_points), tmp_pcd, (submap_list[submap_id_for_keyframes[tmp_kid]].pose *
                    keyframe_list[tmp_kid].pose).matrix());
                (*(volume.flat_points)) += tmp_pcd;


                pcl::transformPointCloud (*(keyframe_list[tmp_kid].less_sharp_points), tmp_pcd, (submap_list[submap_id_for_keyframes[tmp_kid]].pose *
                    keyframe_list[tmp_kid].pose).matrix());
                (*(volume.less_sharp_points)) +=  tmp_pcd;                    

                pcl::transformPointCloud (*(keyframe_list[tmp_kid].less_flat_points), tmp_pcd, (submap_list[submap_id_for_keyframes[tmp_kid]].pose *
                    keyframe_list[tmp_kid].pose).matrix());
                (*(volume.less_flat_points)) += tmp_pcd;
                if(use_ground_priority)
                {
                    pcl::transformPointCloud (*(keyframe_list[tmp_kid].ground_points), tmp_pcd, (submap_list[submap_id_for_keyframes[tmp_kid]].pose *
                    keyframe_list[tmp_kid].pose).matrix());
                    (*(volume.ground_points)) += tmp_pcd;

                    pcl::transformPointCloud (*(keyframe_list[tmp_kid].less_ground_points), tmp_pcd, (submap_list[submap_id_for_keyframes[tmp_kid]].pose *
                    keyframe_list[tmp_kid].pose).matrix());
                    (*(volume.less_ground_points)) += tmp_pcd;
                }
                pcl::transformPointCloud (*(keyframe_list[tmp_kid].pcd), tmp_pcd, (submap_list[submap_id_for_keyframes[tmp_kid]].pose *
                    keyframe_list[tmp_kid].pose).matrix());
                (*(volume.pcd)) +=  tmp_pcd;   
                FilterPCD(*volume.pcd,  
                    *volume.pcd, flat_leaf_size, 
                    flat_leaf_size, flat_leaf_size);   
            }            
        }
        FilterPCD(*volume.sharp_points,  
            *volume.sharp_points, sharp_leaf_size, 
                sharp_leaf_size, sharp_leaf_size);

        FilterPCD(*volume.flat_points,  
            *volume.flat_points, flat_leaf_size, 
            flat_leaf_size, flat_leaf_size);

        FilterPCD(*volume.less_sharp_points,  
            *volume.less_sharp_points, sharp_leaf_size, 
                sharp_leaf_size, sharp_leaf_size);

        FilterPCD(*volume.less_flat_points,  
            *volume.less_flat_points, flat_leaf_size, 
            flat_leaf_size, flat_leaf_size);

        volume.less_sharp_kdtree =
            pcl::KdTreeFLANN<PointType>::Ptr (new pcl::KdTreeFLANN<PointType>());
        volume.less_flat_kdtree =
            pcl::KdTreeFLANN<PointType>::Ptr (new pcl::KdTreeFLANN<PointType>());  

        volume.less_sharp_kdtree->setInputCloud(volume.less_sharp_points);
        volume.less_flat_kdtree->setInputCloud(volume.less_flat_points);


        if(use_ground_priority)
        {
            FilterPCD(*volume.ground_points,  
                *volume.ground_points, flat_leaf_size, 
                flat_leaf_size, flat_leaf_size);
            FilterPCD(*volume.less_ground_points,  
                *volume.less_ground_points, flat_leaf_size, 
                flat_leaf_size, flat_leaf_size);
            volume.less_ground_kdtree =
                pcl::KdTreeFLANN<PointType>::Ptr (new pcl::KdTreeFLANN<PointType>()); 
            volume.less_ground_kdtree->setInputCloud(volume.less_ground_points); 
        }
    }
    void GraphBase::ConstructSubmapVolume(int submap_id, Frame &volume)
    {
        volume = submap_list[submap_id];
        int start = submap_id - 1;
        int end = submap_id + 1;
        if(start < 0) start = 0;
        if(end >=submap_list.size()) end = submap_list.size() - 1;
        PointCloud tmp_pcd;
        for(int i = start; i <= end; ++i)
        {
            if(i == submap_id) continue;
            SE3 relative_pose_to_current_submap = submap_list[submap_id].pose.inverse() * submap_list[i].pose;
            pcl::transformPointCloud (*(submap_list[i].sharp_points), tmp_pcd, relative_pose_to_current_submap.matrix());
            (*(volume.sharp_points)) +=  tmp_pcd;                    

            pcl::transformPointCloud (*(submap_list[i].flat_points), tmp_pcd, relative_pose_to_current_submap.matrix());
            (*(volume.flat_points)) += tmp_pcd;


            pcl::transformPointCloud (*(submap_list[i].less_sharp_points), tmp_pcd, relative_pose_to_current_submap.matrix());
            (*(volume.less_sharp_points)) +=  tmp_pcd;                    

            pcl::transformPointCloud (*(submap_list[i].less_flat_points), tmp_pcd, relative_pose_to_current_submap.matrix());
            (*(volume.less_flat_points)) += tmp_pcd;
            if(use_ground_priority)
            {
                pcl::transformPointCloud (*(submap_list[i].ground_points), tmp_pcd, relative_pose_to_current_submap.matrix());
                (*(volume.ground_points)) += tmp_pcd;

                pcl::transformPointCloud (*(submap_list[i].less_ground_points), tmp_pcd, relative_pose_to_current_submap.matrix());
                (*(volume.less_ground_points)) += tmp_pcd;
            }
            pcl::transformPointCloud (*(submap_list[i].pcd), tmp_pcd, relative_pose_to_current_submap.matrix());
            (*(volume.pcd)) +=  tmp_pcd;   
            FilterPCD(*volume.pcd,  
                *volume.pcd, flat_leaf_size, 
                flat_leaf_size, flat_leaf_size);   
        }

        FilterPCD(*volume.sharp_points,  
            *volume.sharp_points, sharp_leaf_size, 
                sharp_leaf_size, sharp_leaf_size);

        FilterPCD(*volume.flat_points,  
            *volume.flat_points, flat_leaf_size, 
            flat_leaf_size, flat_leaf_size);

        FilterPCD(*volume.less_sharp_points,  
            *volume.less_sharp_points, sharp_leaf_size, 
                sharp_leaf_size, sharp_leaf_size);

        FilterPCD(*volume.less_flat_points,  
            *volume.less_flat_points, flat_leaf_size, 
            flat_leaf_size, flat_leaf_size);

        volume.less_sharp_kdtree =
            pcl::KdTreeFLANN<PointType>::Ptr (new pcl::KdTreeFLANN<PointType>());
        volume.less_flat_kdtree =
            pcl::KdTreeFLANN<PointType>::Ptr (new pcl::KdTreeFLANN<PointType>());  

        volume.less_sharp_kdtree->setInputCloud(volume.less_sharp_points);
        volume.less_flat_kdtree->setInputCloud(volume.less_flat_points);


        if(use_ground_priority)
        {
            FilterPCD(*volume.ground_points,  
                *volume.ground_points, flat_leaf_size, 
                flat_leaf_size, flat_leaf_size);
            FilterPCD(*volume.less_ground_points,  
                *volume.less_ground_points, flat_leaf_size, 
                flat_leaf_size, flat_leaf_size);
            volume.less_ground_kdtree =
                pcl::KdTreeFLANN<PointType>::Ptr (new pcl::KdTreeFLANN<PointType>()); 
            volume.less_ground_kdtree->setInputCloud(volume.less_ground_points); 
        }
    }
    SE3 GraphBase::AddNewFrameSlidingWindow(const Frame &new_frame)
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
            last_frame = f;
            odometry.SetGroundPlane(f.ground_plane_normal, f.ground_plane_dist);
            return f.pose;
        }
        Frame &last_keyframe = keyframe_list.back();
        // this is important, use last relative pose to estimate current relative pose.
        bool matching_success = Matching(f, last_frame, relative_T_to_last_frame);
        if(!matching_success)
        {
            std::cout<<YELLOW<<"[WARNING]::[GraphBase]::Track last keyframe failed."<<RESET<<std::endl;
        }
        f.pose = last_frame.pose * relative_T_to_last_frame;
        relative_T_to_last_keyframe = last_keyframe.pose.inverse() * f.pose ;
        if(!NewKeyframe(relative_T_to_last_keyframe)) 
        {
            relative_pose_list.push_back(relative_T_to_last_frame);    
            FrameCorrespondence tmp_corr(frame_corrs.size(), frame_corrs.size()+1, relative_T_to_last_frame, true);
            frame_corrs.push_back(tmp_corr);            
        }
        else
        {
            int curr_keyframe_id = keyframe_list.size();
            std::cout<<"keyframe: "<<f.frame_id<<" "<<curr_keyframe_id<<std::endl;
            // matching current keyframe to sliding-window-volume
            bool mapping_success = true;
            std::vector<int> keyframe_ids_in_sw = GetFrameIDInSlidingWindow();
            if(keyframe_ids_in_sw.size() >= min_mapping_n)
            {
                ConstructSWVolume(keyframe_ids_in_sw, sliding_window_volume);
                mapping_success = Mapping(f, sliding_window_volume, f.pose);
                relative_T_to_last_keyframe = last_keyframe.pose.inverse() * f.pose;
            }
            if(sliding_window_type == 2)
            {
                Vec3 tmp_pos = f.pose.translation();
                int cid = moving_box.GetCubeID(tmp_pos);
                if(cid == -1)
                moving_box.CenterCubesAt(tmp_pos);
                else 
                moving_box.cubes[cid].push_back(curr_keyframe_id);
            }
            if(!mapping_success)
            {
                std::cout<<YELLOW<<"[WARNING]::[GraphBase]::Keyframe mapping failed."<<RESET<<std::endl;
            }

            keyframe_list.push_back(f);
            relative_pose_list.push_back(SE3());   
            keyframe_corrs.push_back(FrameCorrespondence(curr_keyframe_id , curr_keyframe_id - 1, relative_T_to_last_keyframe, true));

            // relative_pose_list.push_back(relative_T_to_last_keyframe);    

            FrameCorrespondence tmp_corr(frame_corrs.size(), frame_corrs.size()+1, 
                relative_T_to_last_frame);
            frame_corrs.push_back(tmp_corr);
                // OptimizeLocal(); 
            // loop closure detection
            // loop closure detection still need
            if(lcd_detection && keyframe_list.size() - last_lcd_id - 1 >= lcd_detection_interval )
            {
                std::vector<int> candidate_ids;
                if(lcd_detection == 1) lcdetector.NaiveDetection(candidate_ids);
                else if (lcd_detection == 2) lcdetector.SCDetection(candidate_ids);
                std::cout<<"number of lcd candidates: "<<candidate_ids.size()<<std::endl;
                if(candidate_ids.size() > 0) 
                {
                    std::vector<int> ids_in_sw;
                    int start = int(keyframe_list.size())-sliding_window_volume_n / 2 - 1;
                    if(start < 0) start = 0; 
                    for(size_t i = start; i< keyframe_list.size(); ++i) ids_in_sw.push_back(i);
                    ConstructSWVolume(ids_in_sw, sliding_window_volume);
                }
                for(size_t id = 0; id != candidate_ids.size(); ++id)
                {
                    Frame &candidate_frame = keyframe_list[candidate_ids[id]];
                    SE3 loop_relative_T = candidate_frame.pose.inverse() * f.pose; 
                    // if use loam initialization
                    // std::cout<<loop_relative_T.matrix()<<"\n------------------"<<std::endl;
                    bool mapping_success_tmp = Matching(f, candidate_frame, loop_relative_T);
                    
                    // check if the closed loop is correct;
                    if(mapping_success_tmp)
                    {
                        int start = (candidate_ids[id] >= sliding_window_volume_n/2 ) ? (candidate_ids[id] - sliding_window_volume_n/2 ) : 0;
                        int end = (candidate_ids[id] + sliding_window_volume_n/2  >= keyframe_list.size()) ? (keyframe_list.size() - 1) : (candidate_ids[id] + sliding_window_volume_n/2 ); 
                        std::vector<int> ids_in_loop_sw;
                        for(int i = start; i <= end; ++i) ids_in_loop_sw.push_back(i);
                        Frame candidate_volume;
                        ConstructSWVolume(ids_in_loop_sw, candidate_volume);
                        if(exaustive_mapping)
                        {
                            SE3 loop_global_relative_T = f.pose.inverse() * candidate_frame.pose * loop_relative_T;
                            mapping_success_tmp = Mapping(sliding_window_volume, candidate_volume, loop_global_relative_T);
                            loop_relative_T = candidate_frame.pose.inverse() * f.pose * loop_global_relative_T;
                            // std::cout<<loop_relative_T.matrix()<<"\n------------------"<<std::endl;
#if 1
                            if(!mapping_success_tmp)
                            WritePLY("source_"+std::to_string(keyframe_list.size() - 1)+"_exaustive.ply", *sliding_window_volume.pcd);
                            WritePLY("target_"+std::to_string(keyframe_list.size() - 1)+
                                "_"+std::to_string(candidate_ids[id])+"_exaustive.ply", *candidate_volume.pcd);
#endif
                        }
                        else
                        {
                            SE3 loop_global_T = candidate_frame.pose * loop_relative_T;
#if 0
                            PointCloud tmp_pcd_;
                            pcl::transformPointCloud(*f.pcd, tmp_pcd_,loop_global_T.matrix());
                            WritePLY("source_"+std::to_string(keyframe_list.size() - 1)+".ply", tmp_pcd_);
                            WritePLY("target_"+std::to_string(keyframe_list.size() - 1)+
                                "_"+std::to_string(candidate_ids[id])+".ply", *candidate_volume.pcd);
#endif
                            mapping_success_tmp = Mapping(f, candidate_volume, loop_global_T);

                            loop_relative_T = candidate_frame.pose.inverse() *loop_global_T;                    
                        }
                        if(mapping_success_tmp)
                        {
                            if(use_ground_priority && 
                                ((candidate_frame.pose * loop_relative_T).so3() * f.ground_plane_normal).normalized().dot(keyframe_list[0].ground_plane_normal) < 0.5)
                            {
                                std::cout<<"use ground priority to filter out false loop closure detection! "<<std::endl;
                                continue;
                            }
                            std::cout<<"Detect Loop closure, candidate id: "<<keyframe_list[candidate_ids[id]].frame_id<<" "<<candidate_ids[id]<<std::endl;
                            FrameCorrespondence tmp_corr(curr_keyframe_id, candidate_ids[id], loop_relative_T, true);
                            keyframe_corrs.push_back(tmp_corr);
                            last_lcd_id = keyframe_list.size() - 1;
                        }
                        else 
                        {
                            std::cout<<"use distance threshold (mapping) to filter out false loop closure detection! "<<std::endl;
                        }
                    }
                    else std::cout<<"use distance threshold (matching) to filter out false loop closure detection! "<<std::endl;
                }
                
            }
            // pose optimization
            Optimize();
        }
        last_frame = f;
        return f.pose;
        
    }
    void GraphBase::Optimize()
    {
        if(!with_imu)
        {
            if(use_ground_priority)
            optimizer.OptimizeWithGroundNormal(keyframe_list, keyframe_corrs, ground_weight);
            else 
            optimizer.Optimize(keyframe_list, keyframe_corrs);
        }
    }
    void GraphBase::OptimizeSubmap()
    {
        if(use_ground_priority) 
            optimizer.OptimizeWithGroundNormal(submap_list, submap_corrs);
        else
            optimizer.Optimize(submap_list, submap_corrs);
    }
    void GraphBase::OptimizeLocal()
    {
        if(keyframe_list.size() < 2) return;
        int key_frame_id_2 = keyframe_list[keyframe_list.size() - 1].frame_id;
        int key_frame_id_1 = keyframe_list[keyframe_list.size() - 2].frame_id;  
        if(key_frame_id_2 - key_frame_id_1 < 2) return;
        std::vector<SE3 > tmp_relative_pose_list;
        //
        tmp_relative_pose_list.resize(key_frame_id_2 - key_frame_id_1 + 1);
        for(int i = key_frame_id_1 + 1, j = 1; i != key_frame_id_2; ++i, ++j)
        tmp_relative_pose_list[j] = tmp_relative_pose_list[j - 1] * relative_pose_list[i];
        tmp_relative_pose_list.back() = relative_T_to_last_keyframe;

        // frame_corrs[0].weight *= 2;
        // frame_corrs.back().weight *= 2;
        //fix first pose and last pose
        optimizer.Optimize(tmp_relative_pose_list, frame_corrs, true, true);
        for(int i = key_frame_id_1+1, j = 1; i < key_frame_id_2; ++i, ++j)
        relative_pose_list[i] = tmp_relative_pose_list[j - 1].inverse() * tmp_relative_pose_list[j];   
        std::cout<<"local optimization! "<<std::endl;  
        frame_corrs.clear();
           
    }
}
}