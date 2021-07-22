#include "Visualization.h"
#include "ParamServer.h"
#include "System/GraphBase.h"
#include "Utils/IO.h"
using namespace rabbit;

// keyframe-based lidar slam system
// Perhaps we still need a mapping backend
class LidarSlam: public ParamServer
{
    public:
    LidarSlam()
    {
        pcd_sub = nh.subscribe<sensor_msgs::PointCloud2>(pcd_topic, 30, &LidarSlam::PCDHandler, this, ros::TransportHints().tcpNoDelay());
        // sys.lcd_detection = true;
        // sys.sliding_window_volume_n = 10;
    }
    protected:

    void PCDHandler(const sensor_msgs::PointCloud2::ConstPtr& pcd_raw)
    {
        std::lock_guard<std::mutex> lock(pcd_lock);
        Frame pcd_frame(*pcd_raw);
        sys.AddNewFrameSlidingWindow(pcd_frame);
        if(sys.IsKeyframeInserted() && sys.keyframe_list.size() > 1)
        {   
            global_pcd = PointCloud();
            for(size_t i = 0; i < sys.keyframe_list.size() - 1; ++i)
            {
                pcl::transformPointCloud (*(sys.keyframe_list[i].pcd), pcd, 
                    sys.keyframe_list[i].pose.matrix());
                global_pcd += pcd;
            }
            FilterPCD(global_pcd, global_pcd, 1, 1, 1);
            ColorizePointCloud(global_pcd, Vec3f(1, 1, 1),global_pcd_rgb);  
            pcl::transformPointCloud (*(sys.keyframe_list.back().pcd), pcd, 
                sys.keyframe_list.back().pose.matrix()); 
            ColorizePointCloud(pcd, Vec3f(0, 1 ,0), pcd_rgb);         
            global_pcd_rgb += pcd_rgb;

            pcl::transformPointCloud (*(sys.keyframe_list.back().less_flat_points), pcd, 
                sys.keyframe_list.back().pose.matrix()); 
            ColorizePointCloud(pcd, Vec3f(0, 0, 1), pcd_rgb);         
            global_pcd_rgb += pcd_rgb;

            pcl::transformPointCloud (*(sys.keyframe_list.back().less_sharp_points), pcd, 
                sys.keyframe_list.back().pose.matrix()); 
            ColorizePointCloud(pcd, Vec3f(1, 0, 0), pcd_rgb);         
            global_pcd_rgb += pcd_rgb;
            size_t fid = 0;
            pcd.points.resize(sys.keyframe_list.back().frame_id);
            for(size_t i = 1; i < sys.keyframe_list.size(); ++i)
            {
                Frame &tmp_f = sys.keyframe_list[i];
                SE3 last_pose = tmp_f.pose;
                for(; fid < tmp_f.frame_id; ++fid)
                {   
                    last_pose = last_pose * sys.relative_pose_list[fid];
                    Vec3 pos = (last_pose).translation();
                    pcd.points[fid].x = pos(0);
                    pcd.points[fid].y = pos(1);
                    pcd.points[fid].z = pos(2);
                }
            }
            ColorizePointCloud(pcd, Vec3f(1, 0, 1), pcd_rgb);
            global_pcd_rgb += pcd_rgb;
            visualizer.SetPCD(global_pcd_rgb);    
            // WritePLY("./run_keyframe.ply", global_pcd_rgb);        
        }
        visualizer.ShowOnce();   
        
    }

    private:

    PointCloud global_pcd, pcd;
    PointCloudRGB pcd_rgb, global_pcd_rgb;
    std::mutex pcd_lock;
    ros::Subscriber  pcd_sub;
    bool initialized = false;
    system::GraphBase sys;
    visualization::Visualizer visualizer;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rabbit_test");
    LidarSlam lidar_slam;
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();

    return 0;
}