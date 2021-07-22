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
        if(matching_method == OdometryMethod::LOAM)
        {
            return odometry.Loam(s, t, T);
        }
        else if(matching_method == OdometryMethod::ICP)
        {
            return odometry.ICP(s, t, T);
        }
        else if(matching_method == OdometryMethod::GICP)
        {
            return odometry.GICP(s, t, T);
        }
        else if(matching_method == OdometryMethod::NDT)
        {
            return odometry.NDT(s, t, T);
        }
    }
    bool GraphBase::Mapping(const Frame &s, const Frame &t, SE3 &T)
    {
        if(mapping_method == OdometryMethod::LOAM)
        {
            return odometry.LoamMapping(s, t, T);
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
    std::vector<int> GraphBase::GetFrameIDInSlidingWindow()
    {
        std::vector<int> ids;
        if(sliding_window_type == 1)
        {
            int start = (keyframe_list.size() >= sliding_window_volume_n) ? (keyframe_list.size() - sliding_window_volume_n):0;
            for(int i = 0; i < sliding_window_volume_n && i + start < keyframe_list.size(); ++i)
            ids.push_back(i + start);
        }
        else if(sliding_window_type == 2)
        {
            for(size_t i = 0; i != moving_box.cubes.size(); ++i)
            ids.insert(ids.end(), moving_box.cubes[i].begin(), moving_box.cubes[i].end());
        }
        return ids;

    }
    void GraphBase::AddNewFrameSubmap(const Frame &new_frame)
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
            last_frame = f;
            return;
        }
        Frame &last_keyframe = keyframe_list.back();
        // this is important, use last relative pose to estimate current relative pose.
        bool matching_success = Matching(f, last_frame, relative_T_to_last_frame);
        if(matching_success)
        {
            f.pose = last_frame.pose * relative_T_to_last_frame;
            relative_T_to_last_keyframe = last_keyframe.pose.inverse() * f.pose ;
            if(!NewKeyframe(relative_T_to_last_keyframe)) relative_pose_list.push_back(relative_T_to_last_frame);
            else
            {
                std::cout<<"keyframe: "<<f.frame_id<<std::endl;
                int curr_keyframe_id = keyframe_list.size();
                bool mapping_success = true;

                // matching current keyframe to current submap
                
                bool new_submap_flag = (keyframe_list.size() % submap_len == 0);
                if(new_submap_flag || keyframe_list.size() % submap_len >= 5)
                mapping_success = Mapping(f, submap_list.back(), f.pose);

                if(mapping_success)
                {
                    if(new_submap_flag)
                    {
                        // add new submap
                        // WritePCD("./submap_"+std::to_string(submap_list.size() - 1)+".pcd", *(submap_list.back().pcd));                   
                        std::cout<<"new submap: "<<submap_list.size()<<std::endl;
                        Frame new_submap;
                        new_submap.less_sharp_points = PointCloudPtr(new PointCloud());
                        new_submap.less_flat_points = PointCloudPtr(new PointCloud()); 
                        new_submap.sharp_points = PointCloudPtr(new PointCloud());
                        new_submap.flat_points = PointCloudPtr(new PointCloud()); 
                        new_submap.pcd = PointCloudPtr(new PointCloud());
                        submap_list.push_back(new_submap);    
                    }
                    Frame &current_submap = submap_list.back();
                    // add current keyframe to current submap
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
                    
                    pcl::transformPointCloud (*(f.pcd), tmp_pcd, f.pose.matrix());
                    (*(current_submap.pcd)) += tmp_pcd;                

                    FilterPCD(*current_submap.pcd, *current_submap.pcd, flat_leaf_size,
                        flat_leaf_size, flat_leaf_size);

                    // submap mapping
                    if(submap_list.size() >= 2)
                    {
                        SE3 relative_T_to_last_submap;
                        int last_submap_id = submap_list.size() - 1;
                        if(Mapping(submap_list[last_submap_id], 
                            submap_list[last_submap_id - 1], relative_T_to_last_submap))
                        {
                            std::cout<<"submap mapping: "<<relative_T_to_last_submap.matrix()<<std::endl;
                            for(size_t i = last_submap_id * submap_len; i != keyframe_list.size(); ++i)
                            {
                                keyframe_list[i].pose =  keyframe_list[i].pose * relative_T_to_last_submap;
                            }
                            f.pose = f.pose * relative_T_to_last_submap;
                        }
                    }



                    // add keyframe
                    keyframe_list.push_back(f);
                    relative_pose_list.push_back(SE3());   
                    keyframe_corrs.push_back(FrameCorrespondence(curr_keyframe_id , curr_keyframe_id - 1, relative_T_to_last_keyframe, true));
                
                }
                else 
                {
                    std::cout<<YELLOW<<"[WARNING]::[GraphBase]::Keyframe mapping failed."<<RESET<<std::endl;
                }
                // pose optimization
                // Optimize();
                
            }
            last_frame = f;
        }
        else 
        {
            std::cout<<YELLOW<<"[WARNING]::[GraphBase]::Track last keyframe failed."<<RESET<<std::endl;
        }
    }
    void GraphBase::AddNewFrameSlidingWindow(const Frame &new_frame)
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
            return;
        }
        Frame &last_keyframe = keyframe_list.back();
        // this is important, use last relative pose to estimate current relative pose.
        bool matching_success = Matching(f, last_frame, relative_T_to_last_frame);
        if(matching_success)
        {
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
                std::cout<<"keyframe: "<<f.frame_id<<std::endl;
                int curr_keyframe_id = keyframe_list.size();
                // matching current keyframe to sliding-window-volume
                bool mapping_success = true;
                std::vector<int> keyframe_ids_in_sw = GetFrameIDInSlidingWindow();
                if(keyframe_ids_in_sw.size() >= min_mapping_n)
                {
                    sliding_window_volume.less_sharp_points = PointCloudPtr(new PointCloud());
                    sliding_window_volume.less_flat_points = PointCloudPtr(new PointCloud());
                    sliding_window_volume.sharp_points = PointCloudPtr(new PointCloud());
                    sliding_window_volume.flat_points = PointCloudPtr(new PointCloud());
                    sliding_window_volume.pcd = PointCloudPtr(new PointCloud());

                    for(size_t i = 0; i != keyframe_ids_in_sw.size(); ++i)
                    {
                        PointCloud tmp_pcd;
                        int tmp_kid = keyframe_ids_in_sw[i];

                        pcl::transformPointCloud (*(keyframe_list[tmp_kid].sharp_points), tmp_pcd, keyframe_list[tmp_kid].pose.matrix());
                        (*(sliding_window_volume.sharp_points)) +=  tmp_pcd;                    

                        pcl::transformPointCloud (*(keyframe_list[tmp_kid].flat_points), tmp_pcd, keyframe_list[tmp_kid].pose.matrix());
                        (*(sliding_window_volume.flat_points)) += tmp_pcd;

                        pcl::transformPointCloud (*(keyframe_list[tmp_kid].less_sharp_points), tmp_pcd, keyframe_list[tmp_kid].pose.matrix());
                        (*(sliding_window_volume.less_sharp_points)) +=  tmp_pcd;                    

                        pcl::transformPointCloud (*(keyframe_list[tmp_kid].less_flat_points), tmp_pcd, keyframe_list[tmp_kid].pose.matrix());
                        (*(sliding_window_volume.less_flat_points)) += tmp_pcd;

                        pcl::transformPointCloud (*(keyframe_list[tmp_kid].pcd), tmp_pcd, keyframe_list[tmp_kid].pose.matrix());
                        (*(sliding_window_volume.pcd)) +=  tmp_pcd;   
                        FilterPCD(*sliding_window_volume.pcd,  
                            *sliding_window_volume.pcd, flat_leaf_size, 
                            flat_leaf_size, flat_leaf_size);   
                    }
                    FilterPCD(*sliding_window_volume.sharp_points,  
                        *sliding_window_volume.sharp_points, sharp_leaf_size, 
                            sharp_leaf_size, sharp_leaf_size);

                    FilterPCD(*sliding_window_volume.flat_points,  
                        *sliding_window_volume.flat_points, flat_leaf_size, 
                        flat_leaf_size, flat_leaf_size);

                    FilterPCD(*sliding_window_volume.less_sharp_points,  
                        *sliding_window_volume.less_sharp_points, sharp_leaf_size, 
                            sharp_leaf_size, sharp_leaf_size);

                    FilterPCD(*sliding_window_volume.less_flat_points,  
                        *sliding_window_volume.less_flat_points, flat_leaf_size, 
                        flat_leaf_size, flat_leaf_size);
                    
                    sliding_window_volume.less_sharp_kdtree =
                        pcl::KdTreeFLANN<PointType>::Ptr (new pcl::KdTreeFLANN<PointType>());
                    sliding_window_volume.less_flat_kdtree =
                        pcl::KdTreeFLANN<PointType>::Ptr (new pcl::KdTreeFLANN<PointType>());    
                    sliding_window_volume.less_sharp_kdtree->setInputCloud(sliding_window_volume.less_sharp_points);
                    sliding_window_volume.less_flat_kdtree->setInputCloud(sliding_window_volume.less_flat_points);


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
                if(mapping_success)
                {
                    keyframe_list.push_back(f);
                    relative_pose_list.push_back(SE3());   
                    keyframe_corrs.push_back(FrameCorrespondence(curr_keyframe_id , curr_keyframe_id - 1, relative_T_to_last_keyframe, true));

                    // relative_pose_list.push_back(relative_T_to_last_keyframe);    

                    FrameCorrespondence tmp_corr(frame_corrs.size(), frame_corrs.size()+1, 
                        relative_T_to_last_frame);
                    frame_corrs.push_back(tmp_corr);
                    // OptimizeLocal(); 
                }
                else 
                {
                    std::cout<<YELLOW<<"[WARNING]::[GraphBase]::Keyframe mapping failed."<<RESET<<std::endl;
                }
                // loop closure detection
                // loop closure detection still need
                if(lcd_detection)
                {
                    std::vector<int> candidate_ids;
                    lcdetector.SCDetection(candidate_ids);
                    std::cout<<"number of lcd candidates: "<<candidate_ids.size()<<std::endl;
                    for(size_t id = 0; id != candidate_ids.size(); ++id)
                    {
                        Frame &candidate_frame = keyframe_list[candidate_ids[id]];
                        SE3 loop_relative_T = candidate_frame.pose.inverse() * f.pose; 
                        // std::cout<<loop_relative_T.matrix()<<"\n------------------"<<std::endl;
                        bool mapping_success_tmp = odometry.ICP(f, candidate_frame, loop_relative_T);
                        
                        // check if the closed loop is correct;
                        if(mapping_success_tmp)
                        {
                            int start = (candidate_ids[id] >= sliding_window_volume_n/2 ) ? (candidate_ids[id] - sliding_window_volume_n/2 ) : 0;
                            int end = (candidate_ids[id] + sliding_window_volume_n/2  >= keyframe_list.size()) ? (keyframe_list.size() - 1) : (candidate_ids[id] + sliding_window_volume_n/2 ); 
                            sliding_window_volume.less_sharp_points = PointCloudPtr(new PointCloud());
                            sliding_window_volume.less_flat_points = PointCloudPtr(new PointCloud());
                            sliding_window_volume.sharp_points = PointCloudPtr(new PointCloud());
                            sliding_window_volume.flat_points = PointCloudPtr(new PointCloud());
                            sliding_window_volume.pcd = PointCloudPtr(new PointCloud());

                            for(int i = start; i <= end; ++i)
                            {
                                PointCloud tmp_pcd;

                                pcl::transformPointCloud (*(keyframe_list[i].sharp_points), tmp_pcd, keyframe_list[i].pose.matrix());
                                (*(sliding_window_volume.sharp_points)) +=  tmp_pcd;                    

                                pcl::transformPointCloud (*(keyframe_list[i].flat_points), tmp_pcd, keyframe_list[i].pose.matrix());
                                (*(sliding_window_volume.flat_points)) += tmp_pcd;

                                pcl::transformPointCloud (*(keyframe_list[i].less_sharp_points), tmp_pcd, keyframe_list[i].pose.matrix());
                                (*(sliding_window_volume.less_sharp_points)) +=  tmp_pcd;                    

                                pcl::transformPointCloud (*(keyframe_list[i].less_flat_points), tmp_pcd, keyframe_list[i].pose.matrix());
                                (*(sliding_window_volume.less_flat_points)) += tmp_pcd;
                                
                                pcl::transformPointCloud (*(keyframe_list[i].pcd), tmp_pcd, keyframe_list[i].pose.matrix());
                                (*(sliding_window_volume.pcd)) +=  tmp_pcd;   
                                FilterPCD(*sliding_window_volume.pcd,  
                                    *sliding_window_volume.pcd, flat_leaf_size, 
                                    flat_leaf_size, flat_leaf_size);  
                            }
                            FilterPCD(*sliding_window_volume.sharp_points,  
                                *sliding_window_volume.sharp_points, sharp_leaf_size, 
                                    sharp_leaf_size, sharp_leaf_size);

                            FilterPCD(*sliding_window_volume.flat_points,  
                                *sliding_window_volume.flat_points, flat_leaf_size, 
                                flat_leaf_size, flat_leaf_size);

                            FilterPCD(*sliding_window_volume.less_sharp_points,  
                                *sliding_window_volume.less_sharp_points, sharp_leaf_size, 
                                    sharp_leaf_size, sharp_leaf_size);

                            FilterPCD(*sliding_window_volume.less_flat_points,  
                                *sliding_window_volume.less_flat_points, flat_leaf_size, 
                                flat_leaf_size, flat_leaf_size);
                            
                            sliding_window_volume.less_sharp_kdtree =
                                pcl::KdTreeFLANN<PointType>::Ptr (new pcl::KdTreeFLANN<PointType>());
                            sliding_window_volume.less_flat_kdtree =
                                pcl::KdTreeFLANN<PointType>::Ptr (new pcl::KdTreeFLANN<PointType>());    
                            sliding_window_volume.less_sharp_kdtree->setInputCloud(sliding_window_volume.less_sharp_points);
                            sliding_window_volume.less_flat_kdtree->setInputCloud(sliding_window_volume.less_flat_points);    

                            SE3 loop_global_T = candidate_frame.pose * loop_relative_T;

                            mapping_success_tmp = Mapping(f, sliding_window_volume, loop_global_T);
                            loop_relative_T = candidate_frame.pose.inverse() * loop_global_T;
                            // std::cout<<loop_relative_T.matrix()<<"\n------------------"<<std::endl;
                            if(mapping_success_tmp)
                            {
                                std::cout<<"Detect Loop closure, candidate id: "<<keyframe_list[candidate_ids[id]].frame_id<<std::endl;
                                FrameCorrespondence tmp_corr(curr_keyframe_id, candidate_ids[id], loop_relative_T, true);
                                keyframe_corrs.push_back(tmp_corr);
                            }
                        }
                    }
                    
                }
                // pose optimization
                Optimize();
            }
            last_frame = f;
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