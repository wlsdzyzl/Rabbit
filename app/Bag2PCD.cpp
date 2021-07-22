
#include "ParamServer.h"
#include "Utils/IO.h"
#include "Frame.h"
using namespace rabbit;

// keyframe-based lidar slam system
// Perhaps we still need a mapping backend
class LidarSlam: public ParamServer
{
    public:
    LidarSlam(const std::string &path_
        = "/media/wlsdzyzl/4986a128-c51f-4384-87a6-abf677343495/lidar-dataset/lego-loam/pcds/"):path(path_)
    {
        pcd_sub = nh.subscribe<sensor_msgs::PointCloud2>(pcd_topic, 30, &LidarSlam::PCDHandler, this, ros::TransportHints().tcpNoDelay());
        DeleteAndMakeDir(path);
    }
    protected:

    void PCDHandler(const sensor_msgs::PointCloud2::ConstPtr& pcd_raw)
    {
        std::lock_guard<std::mutex> lock(pcd_lock);
        Frame pcd_frame(*pcd_raw);

        util::WritePCD(path + 
            std::to_string(id)+".pcd", *(pcd_frame.pcd));
        ++id;
    }

    private:
    std::mutex pcd_lock;
    ros::Subscriber  pcd_sub;
    size_t id = 0;
    std::string path;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rabbit_test");
    LidarSlam lidar_slam;
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();

    return 0;
}