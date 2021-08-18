#include "Visualization.h"
#include "System/GraphBase.h"
#include "Utils/IO.h"
using namespace rabbit;

int main(int argc, char **argv)
{
    if(argc < 2)
    {
        std::cout<<"usage: RunPCD [pcd_folder] [matching_method = loam] [mapping_method = icp] [sliding_window = 1] [lcd_detection = 0] [ground_normal_opt = 0] [visualization = 0]"<<std::endl;
        return 0;
    }

    // keyframe-based lidar slam system
    // Perhaps we still need a mapping backend
    PointCloud global_pcd, pcd;
    PointCloudRGB pcd_rgb, global_pcd_rgb;
    system::GraphBase sys;
    std::string matching_method = "loam";
    std::string mapping_method = "loam";
    int sliding_window_type = 1;
    int lcd_detection = 0;
    bool use_ground_prior = false;
    bool visualization = false;

    if(argc > 2) matching_method = argv[2];
    if(argc > 3) mapping_method = argv[3];
    if(argc > 4) sliding_window_type = atoi(argv[4]);
    if(argc > 5) lcd_detection = atoi(argv[5]);
    if(argc > 6) use_ground_prior = atoi(argv[6]);
    if(argc > 7) visualization = atoi(argv[7]);
    sys.SetGroundprior(use_ground_prior);
    sys.SetMatchingMethod(matching_method);
    sys.SetMappingMethod(mapping_method);
    sys.SetSlidingWindow(sliding_window_type);
    // sys.odometry.SetNDTOMPSearchMethod(pclomp::DIRECT7);
    sys.lcd_detection = lcd_detection;
    std::vector<std::string> seq; 
    GetPCDSequence(argv[1], seq);    
    std::cout<<"Find "<< seq.size()<<" point clouds."<<std::endl;        
    if(visualization)
    {
        visualization::Visualizer visualizer;

        for(size_t i = 0; i != seq.size(); ++i)
        {
            LoadPCD(seq[i], pcd);;
            Frame pcd_frame;
            pcd_frame.SetPCD(pcd);
            SE3 curr_pose = sys.AddNewFrame(pcd_frame);
            if(sys.IsKeyframeInserted() && sys.keyframe_list.size() > 1)
            {   
                global_pcd = PointCloud();
                for(size_t i = 0; i < sys.keyframe_list.size() - 1; ++i)
                {
                if(!sliding_window_type)
                    pcl::transformPointCloud (*(sys.keyframe_list[i].pcd), pcd, 
                        (sys.submap_list[sys.submap_id_for_keyframes[i]].pose * sys.keyframe_list[i].pose).matrix());             
                else 
                    pcl::transformPointCloud (*(sys.keyframe_list[i].pcd), pcd, 
                        sys.keyframe_list[i].pose.matrix());
                    global_pcd += pcd;
                }
                // std::cout<<global_pcd.size()<<" ";
                FilterPCD(global_pcd, global_pcd, 2, 2, 2);
                // std::cout<<global_pcd.size()<<std::endl;
                ColorizePointCloud(global_pcd, global_pcd_rgb);  
                if(!sliding_window_type)
                    pcl::transformPointCloud (*(sys.keyframe_list.back().pcd), pcd, 
                        (sys.submap_list.back().pose * sys.keyframe_list.back().pose).matrix());             
                else 
                    pcl::transformPointCloud (*(sys.keyframe_list.back().pcd), pcd, 
                        sys.keyframe_list.back().pose.matrix()); 
                ColorizePointCloud(pcd, Vec3f(0, 1 ,0), pcd_rgb);         
                global_pcd_rgb += pcd_rgb;
                // draw flat, sharp and ground points
                if(!sliding_window_type)
                    pcl::transformPointCloud (*(sys.keyframe_list.back().less_flat_points), pcd, 
                        (sys.submap_list.back().pose * sys.keyframe_list.back().pose).matrix());          
                else 
                    pcl::transformPointCloud (*(sys.keyframe_list.back().less_flat_points), pcd, 
                        sys.keyframe_list.back().pose.matrix()); 
                ColorizePointCloud(pcd, Vec3f(0, 0, 1), pcd_rgb);         
                global_pcd_rgb += pcd_rgb;

                if(!sliding_window_type)
                    pcl::transformPointCloud (*(sys.keyframe_list.back().less_sharp_points), pcd, 
                        (sys.submap_list.back().pose * sys.keyframe_list.back().pose).matrix());          
                else 
                    pcl::transformPointCloud (*(sys.keyframe_list.back().less_sharp_points), pcd, 
                        sys.keyframe_list.back().pose.matrix()); 
                ColorizePointCloud(pcd, Vec3f(1, 0, 0), pcd_rgb);        

                // pcl::transformPointCloud (*(sys.keyframe_list.back().less_ground_points), pcd, 
                //     sys.keyframe_list.back().pose.matrix()); 
                // ColorizePointCloud(pcd, Vec3f(0, 1, 0), pcd_rgb);  
                global_pcd_rgb += pcd_rgb;
                size_t fid = 0;
                pcd.points.clear();
                pcd.points.reserve(sys.keyframe_list.back().frame_id);
                for(size_t i = 1; i < sys.keyframe_list.size(); ++i)
                {
                    Frame &tmp_f = sys.keyframe_list[i];
                    fid += 1;
                    SE3 last_pose = tmp_f.pose;
                    if(!sliding_window_type) last_pose =sys.submap_list[sys.submap_id_for_keyframes[i]].pose *  tmp_f.pose;
                    for(; fid < tmp_f.frame_id; ++fid)
                    {   
                        last_pose = last_pose * sys.relative_pose_list[fid];
                        Vec3 pos = (last_pose).translation();
                        PointType tmp_p;
                        tmp_p.x = pos(0);
                        tmp_p.y = pos(1);
                        tmp_p.z = pos(2);
                        pcd.push_back(tmp_p);
                    }
                }
                ColorizePointCloud(pcd, Vec3f(1, 0, 1), pcd_rgb);
                global_pcd_rgb += pcd_rgb;
                pcd.points.clear();
                pcd.points.reserve(sys.keyframe_list.size());
                for(size_t i = 0; i != sys.keyframe_list.size(); ++i)
                {
                        Vec3 pos = sys.keyframe_list[i].pose.translation();
                        if(!sliding_window_type) pos =(sys.submap_list[sys.submap_id_for_keyframes[i]].pose 
                            *  sys.keyframe_list[i].pose).translation();
                        PointType tmp_p;
                        tmp_p.x = pos(0);
                        tmp_p.y = pos(1);
                        tmp_p.z = pos(2);
                        pcd.push_back(tmp_p);                
                }
                ColorizePointCloud(pcd, Vec3f(1, 1, 0), pcd_rgb);
                global_pcd_rgb += pcd_rgb; 
                visualizer.SetPCD(global_pcd_rgb);   
                // if(sys.keyframe_list.size() % 20 == 0)            
                // WritePLY("global_pcd_rgb_"+matching_method+"_"+mapping_method+
                //     "_"+std::to_string(sliding_window_type)+"_"+std::to_string(lcd_detection)+"_"+
                //     std::to_string(use_ground_prior)+".ply", global_pcd_rgb);
                // WritePLY("global_pcd_"+matching_method+"_"+mapping_method+
                //     "_"+std::to_string(sliding_window_type)+"_"+std::to_string(lcd_detection)+".ply", global_pcd);

            }
            
            visualizer.SetOrigin(curr_pose);    
            visualizer.ShowOnce();    
        }
        WritePLY("global_pcd_rgb_"+matching_method+"_"+mapping_method+
            "_"+std::to_string(sliding_window_type)+"_"+std::to_string(lcd_detection)+"_"+
            std::to_string(use_ground_prior)+".ply", global_pcd_rgb);

        visualizer.Show();
    }
    else
    {
        for(size_t i = 0; i != seq.size(); ++i)
        {
            LoadPCD(seq[i], pcd);;
            Frame pcd_frame;
            pcd_frame.SetPCD(pcd);
            SE3 curr_pose = sys.AddNewFrame(pcd_frame);
            if(sys.IsKeyframeInserted() && sys.keyframe_list.size() > 1)
            {   
                global_pcd = PointCloud();
                for(size_t i = 0; i < sys.keyframe_list.size() - 1; ++i)
                {
                if(!sliding_window_type)
                    pcl::transformPointCloud (*(sys.keyframe_list[i].pcd), pcd, 
                        (sys.submap_list[sys.submap_id_for_keyframes[i]].pose * sys.keyframe_list[i].pose).matrix());             
                else 
                    pcl::transformPointCloud (*(sys.keyframe_list[i].pcd), pcd, 
                        sys.keyframe_list[i].pose.matrix());
                    global_pcd += pcd;
                }
                // std::cout<<global_pcd.size()<<" ";
                FilterPCD(global_pcd, global_pcd, 1, 1, 1);
                // std::cout<<global_pcd.size()<<std::endl;
                ColorizePointCloud(global_pcd, global_pcd_rgb);  
                if(!sliding_window_type)
                    pcl::transformPointCloud (*(sys.keyframe_list.back().pcd), pcd, 
                        (sys.submap_list.back().pose * sys.keyframe_list.back().pose).matrix());             
                else 
                    pcl::transformPointCloud (*(sys.keyframe_list.back().pcd), pcd, 
                        sys.keyframe_list.back().pose.matrix()); 
                ColorizePointCloud(pcd, Vec3f(0, 1 ,0), pcd_rgb);         
                global_pcd_rgb += pcd_rgb;
                // draw flat, sharp and ground points
                if(!sliding_window_type)
                    pcl::transformPointCloud (*(sys.keyframe_list.back().less_flat_points), pcd, 
                        (sys.submap_list.back().pose * sys.keyframe_list.back().pose).matrix());          
                else 
                    pcl::transformPointCloud (*(sys.keyframe_list.back().less_flat_points), pcd, 
                        sys.keyframe_list.back().pose.matrix()); 
                ColorizePointCloud(pcd, Vec3f(0, 0, 1), pcd_rgb);         
                global_pcd_rgb += pcd_rgb;

                if(!sliding_window_type)
                    pcl::transformPointCloud (*(sys.keyframe_list.back().less_sharp_points), pcd, 
                        (sys.submap_list.back().pose * sys.keyframe_list.back().pose).matrix());          
                else 
                    pcl::transformPointCloud (*(sys.keyframe_list.back().less_sharp_points), pcd, 
                        sys.keyframe_list.back().pose.matrix()); 
                ColorizePointCloud(pcd, Vec3f(1, 0, 0), pcd_rgb);        

                // pcl::transformPointCloud (*(sys.keyframe_list.back().less_ground_points), pcd, 
                //     sys.keyframe_list.back().pose.matrix()); 
                // ColorizePointCloud(pcd, Vec3f(0, 1, 0), pcd_rgb);  
                global_pcd_rgb += pcd_rgb;
                size_t fid = 0;
                pcd.points.clear();
                pcd.points.reserve(sys.keyframe_list.back().frame_id);
                for(size_t i = 1; i < sys.keyframe_list.size(); ++i)
                {
                    Frame &tmp_f = sys.keyframe_list[i];
                    fid += 1;
                    SE3 last_pose = tmp_f.pose;
                    if(!sliding_window_type) last_pose =sys.submap_list[sys.submap_id_for_keyframes[i]].pose *  tmp_f.pose;
                    for(; fid < tmp_f.frame_id; ++fid)
                    {   
                        last_pose = last_pose * sys.relative_pose_list[fid];
                        Vec3 pos = (last_pose).translation();
                        PointType tmp_p;
                        tmp_p.x = pos(0);
                        tmp_p.y = pos(1);
                        tmp_p.z = pos(2);
                        pcd.push_back(tmp_p);
                    }
                }
                ColorizePointCloud(pcd, Vec3f(1, 0, 1), pcd_rgb);
                global_pcd_rgb += pcd_rgb;
                pcd.points.clear();
                pcd.points.reserve(sys.keyframe_list.size());
                for(size_t i = 0; i != sys.keyframe_list.size(); ++i)
                {
                        Vec3 pos = sys.keyframe_list[i].pose.translation();
                        if(!sliding_window_type) pos =(sys.submap_list[sys.submap_id_for_keyframes[i]].pose 
                            *  sys.keyframe_list[i].pose).translation();
                        PointType tmp_p;
                        tmp_p.x = pos(0);
                        tmp_p.y = pos(1);
                        tmp_p.z = pos(2);
                        pcd.push_back(tmp_p);                
                }
                ColorizePointCloud(pcd, Vec3f(1, 1, 0), pcd_rgb);
                global_pcd_rgb += pcd_rgb;  
                if(sys.keyframe_list.size() % 20 == 0)            
                WritePLY("global_pcd_rgb_"+matching_method+"_"+mapping_method+
                    "_"+std::to_string(sliding_window_type)+"_"+std::to_string(lcd_detection)+"_"+
                    std::to_string(use_ground_prior)+".ply", global_pcd_rgb);
                // WritePLY("global_pcd_"+matching_method+"_"+mapping_method+
                //     "_"+std::to_string(sliding_window_type)+"_"+std::to_string(lcd_detection)+".ply", global_pcd);

            }  
        }
        WritePLY("global_pcd_rgb_"+matching_method+"_"+mapping_method+
            "_"+std::to_string(sliding_window_type)+"_"+std::to_string(lcd_detection)+"_"+
            std::to_string(use_ground_prior)+".ply", global_pcd_rgb);

    }
    return 0;
}