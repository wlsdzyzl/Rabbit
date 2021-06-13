#include "ParamServer.h"
#include "Visualization.h"
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
    }
    protected:

    void PCDHandler(const sensor_msgs::PointCloud2::ConstPtr& pcd_raw)
    {
        std::lock_guard<std::mutex> lock(pcd_lock);
        Frame pcd_frame(*pcd_raw);
        util::WritePCD(std::string("/media/wlsdzyzl/FED8431CD842D297/dataset/unity-drive/fsk_relocation/pcds/") + 
            std::to_string(sys.relative_pose_list.size())+".pcd", *(pcd_frame.pcd));
        sys.AddNewFrame(pcd_frame);
        if(sys.IsKeyframeInserted())
        {
            FilterPCD(global_pcd, global_pcd, 0.5, 0.5, 0.5);
            pcl::transformPointCloud (*(pcd_frame.pcd), pcd, sys.keyframe_list.back().pose.matrix());
            ColorizePointCloud(pcd, Vec3f(0, 1, 0) ,pcd_rgb);
            ColorizePointCloud(global_pcd, global_pcd_rgb);            
            global_pcd_rgb += pcd_rgb;
            visualizer.SetPCD(global_pcd_rgb);
            visualizer.ShowOnce();
            global_pcd += pcd;            
        }
    }

    private:

    visualization::Visualizer visualizer;    
    PointCloud global_pcd, pcd;
    PointCloudRGB pcd_rgb, global_pcd_rgb;
    std::mutex pcd_lock;
    ros::Subscriber  imu_sub;
    ros::Subscriber  pcd_sub;
    bool initialized = false;
    system::GraphBase sys;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rabbit_imu_test");
    LidarSlam lidar_slam;
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
    return 0;
}