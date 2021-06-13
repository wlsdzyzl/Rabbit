#include "Nodes/ParamServer.h"
#include "System/GraphBase.h"
using namespace rabbit;

// keyframe-based lidar slam system
// Perhaps we still need a mapping backend

class NaiveIMUOdometry: public ParamServer
{
    public:
    NaiveIMUOdometry()
    {
        imu_sub =  nh.subscribe<sensor_msgs::Imu>(imu_topic, 2000, &NaiveIMUOdometry::IMUHandler, this, ros::TransportHints().tcpNoDelay());
        pcd_sub = nh.subscribe<sensor_msgs::PointCloud2>(pcd_topic, 5, &NaiveIMUOdometry::PCDHandler, this, ros::TransportHints().tcpNoDelay());
        imu::IMUFrame::gravity = Vec3(0, 0, -imu_gravity);
        imu::IMUFrame::imu_frequency = imu_freq;
        // remeber to set bias and extrinsic rotation, rpy
        std::cout<<"gravity: "<<imu::IMUFrame::gravity <<std::endl;
        std::cout<<"imu frequency: "<<imu::IMUFrame::imu_frequency<<std::endl;
    }
    protected:
    void IMUHandler(const sensor_msgs::Imu::ConstPtr& imu_raw)
    {
        std::lock_guard<std::mutex> lock(imu_lock);
        imu::IMUFrame imu_frame(*imu_raw);
        double delta_t = 0;
        if(!initialized)
        {
            imu_integrator.SetInitialBias(imu_frame.angular_velocity, imu_frame.acceleration + imu::IMUFrame::gravity);
            initialized = true;
        }
        if(last_t > 0) delta_t =  imu_frame.timestamp - last_t;
        if(delta_t <= 0) delta_t = 1.0 / imu::IMUFrame::imu_frequency;
        last_t = imu_frame.timestamp;
        imu_integrator.Integrate(imu_frame.angular_velocity, imu_frame.acceleration, delta_t);
        imu_poses.push_back(SE3(imu_integrator.delta_R, imu_integrator.delta_p));
    }

    void PCDHandler(const sensor_msgs::PointCloud2::ConstPtr& pcd_raw)
    {
        std::lock_guard<std::mutex> lock(pcd_lock);
        Frame pcd_frame(*pcd_raw);
        {
            std::lock_guard<std::mutex> lock_imu(imu_lock);
            if(imu_poses.size())
            frame_2_imu.push_back(imu_poses.size() - 1);
            else frame_2_imu.push_back(-1);       
        }
        if((frame_2_imu.size() - 1) % 10 == 0 && frame_2_imu.back() != -1)
        {
            std::cout<<imu_poses.back().matrix()<<std::endl;
            FilterPCD(global_pcd, global_pcd, 0.5, 0.5, 0.5);
            pcl::transformPointCloud (*(pcd_frame.pcd), pcd, imu_poses[frame_2_imu.back()].inverse().matrix());
            ColorizePointCloud(pcd, Vec3f(0, 1, 0) ,pcd_rgb);
            ColorizePointCloud(global_pcd, global_pcd_rgb);            
            global_pcd_rgb += pcd_rgb;
            visualizer.SetPCD(global_pcd_rgb);
            visualizer.ShowOnce();
            global_pcd += pcd;
        }
    }

    private:
    imu::IMUIntegrator imu_integrator;
    double last_t = -1;
    SE3List imu_poses;
    std::vector<int> frame_2_imu;
    visualization::Visualizer visualizer;    
    PointCloud global_pcd, pcd;
    PointCloudRGB pcd_rgb, global_pcd_rgb;
    std::mutex imu_lock;
    std::mutex pcd_lock;
    ros::Subscriber  imu_sub;
    ros::Subscriber  pcd_sub;
    bool initialized = false;
};

int main()
{
    system::GraphBase sys;
    
    return 0;
}