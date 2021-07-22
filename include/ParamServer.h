#ifndef RABBIT_PARAM_SERVER_H
#define RABBIT_PARAM_SERVER_H
/*
This file is adopted from LIO-SAM. It can be considered as a node base.
Other nodes can inherit from this server.
*/
#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <opencv/cv.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include "Utils/Utils.h"
namespace rabbit
{
    using namespace util;
    class ParamServer
    {
        public:
        ros::NodeHandle nh;
        //Topics
        std::string pcd_topic;
        std::string imu_topic;
        std::string odometry_topic;
        // we don't support gps, although it's not difficult.
        // string gps_topic

        // set imu parameter (gravity, frequency)

        Mat3 extrinsic_rot;
        Mat3 extrinsic_rpy;
        Vec3 extrinsic_trans;
        // tmp variables
        std::vector<double> ext_rot_vec;
        std::vector<double> ext_rpy_vec;
        std::vector<double> ext_trans_vec;

        // bias and noise
        double imu_acc_noise;
        double imu_gyr_noise;
        double imu_acc_bias;
        double imu_gyr_bias;
        double imu_gravity;
        double imu_freq;
        
        // set omometry parameter (N-scan)

        ParamServer()
        {
            nh.param<std::string>("rabbit/pcd", pcd_topic, "/velodyne_points");
            nh.param<std::string>("rabbit/imu", imu_topic, "imu_raw");
            nh.param<std::string>("rabbit/odometry", odometry_topic, "odometry/imu");
            
            nh.param<std::vector<double>>("rabbit/extrinsic_rot", ext_rot_vec, std::vector<double>());
            nh.param<std::vector<double>>("rabbit/extrinsic_rpy", ext_rpy_vec, std::vector<double>());
            nh.param<std::vector<double>>("rabbit/extrinsic_trans", ext_trans_vec, std::vector<double>());
            
            // set extrinsics
            if(ext_rot_vec.size() == 9)
            extrinsic_rot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>( ext_rot_vec.data(), 3, 3);
            if(ext_rpy_vec.size() == 9)
            extrinsic_rpy = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(ext_rpy_vec.data(), 3, 3);
            if(ext_trans_vec.size() == 3)
            extrinsic_trans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(ext_trans_vec.data(), 3, 1);

            nh.param<double>("rabbit/imu_acc_noise", imu_acc_noise, 0.01);
            nh.param<double>("rabbit/imu_gyr_noise", imu_gyr_noise, 0.001);
            nh.param<double>("rabbit/imu_acc_bias", imu_acc_bias, 0.0002);
            nh.param<double>("rabbit/imu_gyr_bias", imu_gyr_bias, 0.00003);
            // if so, perhapse we need a 
            nh.param<double>("rabbit/imu_gravity", imu_gravity, 9.80511);
            nh.param<double>("rabbit/imu_frequency", imu_freq, 500);
            // nh.param<float>("rabbit/imuRPYWeight", imuRPYWeight, 0.01);
        }
    };
}

#endif
// class ParamServer
// {
// public:

//     ros::NodeHandle nh;

//     std::string robot_id;

//     //Topics
//     string pointCloudTopic;
//     string imuTopic;
//     string odomTopic;
//     string gpsTopic;

//     //Frames
//     string lidarFrame;
//     string baselinkFrame;
//     string odometryFrame;
//     string mapFrame;

//     // Lidar Sensor Configuration
//     SensorType sensor;
//     int N_SCAN;
//     int Horizon_SCAN;
//     int downsampleRate;
//     float lidarMinRange;
//     float lidarMaxRange;

//     // IMU
//     float imuAccNoise;
//     float imuGyrNoise;
//     float imuAccBiasN;
//     float imuGyrBiasN;
//     float imuGravity;
//     float imuRPYWeight;
//     vector<double> extRotV;
//     vector<double> extRPYV;
//     vector<double> extTransV;
//     Eigen::Matrix3d extRot;
//     Eigen::Matrix3d extRPY;
//     Eigen::Vector3d extTrans;
//     Eigen::Quaterniond extQRPY;
//     Eigen::Matrix3d integrated_rotation;
//     Eigen::Vector3d last_angular_velocity;

//     // LOAM
//     float edgeThreshold;
//     float surfThreshold;
//     int edgeFeatureMinValidNum;
//     int surfFeatureMinValidNum;

//     // voxel filter paprams
//     float odometrySurfLeafSize;
//     float mappingCornerLeafSize;
//     float mappingSurfLeafSize ;

//     float z_tollerance; 
//     float rotation_tollerance;

//     // CPU Params
//     int numberOfCores;
//     double mappingProcessInterval;

//     // Surrounding map
//     float surroundingkeyframeAddingDistThreshold; 
//     float surroundingkeyframeAddingAngleThreshold; 
//     float surroundingKeyframeDensity;
//     float surroundingKeyframeSearchRadius;
    
//     // Loop closure
//     bool  loopClosureEnableFlag;
//     float loopClosureFrequency;
//     int   surroundingKeyframeSize;
//     float historyKeyframeSearchRadius;
//     float historyKeyframeSearchTimeDiff;
//     int   historyKeyframeSearchNum;
//     float historyKeyframeFitnessScore;

//     // global map visualization radius
//     float globalMapVisualizationSearchRadius;
//     float globalMapVisualizationPoseDensity;
//     float globalMapVisualizationLeafSize;

//     ParamServer()
//     {
//         nh.param<std::string>("/robot_id", robot_id, "roboat");


//         // nh.param<std::string>("lio_sam_udi/pointCloudTopic", pointCloudTopic, "points_raw");
//         // nh.param<std::string>("lio_sam_udi/imuTopic", imuTopic, "imu_correct");
//         nh.param<std::string>("lio_sam_udi/pointCloudTopic", pointCloudTopic, "pandar");
//         nh.param<std::string>("lio_sam_udi/imuTopic", imuTopic, "udi/imu_s/data");
//         nh.param<std::string>("lio_sam_udi/odomTopic", odomTopic, "odometry/imu");
//         nh.param<std::string>("lio_sam_udi/gpsTopic", gpsTopic, "odometry/gps");

//         nh.param<std::string>("lio_sam_udi/lidarFrame", lidarFrame, "base_link");
//         nh.param<std::string>("lio_sam_udi/baselinkFrame", baselinkFrame, "base_link");
//         nh.param<std::string>("lio_sam_udi/odometryFrame", odometryFrame, "odom");
//         nh.param<std::string>("lio_sam_udi/mapFrame", mapFrame, "map");

//         nh.param<bool>("lio_sam_udi/useImuHeadingInitialization", useImuHeadingInitialization, false);
//         nh.param<bool>("lio_sam_udi/useGpsElevation", useGpsElevation, false);
//         nh.param<float>("lio_sam_udi/gpsCovThreshold", gpsCovThreshold, 2.0);
//         nh.param<float>("lio_sam_udi/poseCovThreshold", poseCovThreshold, 25.0);

//         nh.param<bool>("lio_sam_udi/savePCD", savePCD, false);
//         nh.param<std::string>("lio_sam_udi/savePCDDirectory", savePCDDirectory, "/Downloads/LOAM/");

//         std::string sensorStr;
//         nh.param<std::string>("lio_sam_udi/sensor", sensorStr, "");
//         if (sensorStr == "velodyne")
//         {
//             sensor = SensorType::VELODYNE;
//         }
//         else if (sensorStr == "ouster")
//         {
//             sensor = SensorType::OUSTER;
//         }
//         else if(sensorStr == "hesai")
//         {
//             sensor = SensorType::HESAI;
//         }
//         else
//         {
//             ROS_ERROR_STREAM(
//                 "Invalid sensor type (must be either 'velodyne' or 'ouster'): " << sensorStr);
//             ros::shutdown();
//         }

//         nh.param<int>("lio_sam_udi/N_SCAN", N_SCAN, 32);
//         nh.param<int>("lio_sam_udi/Horizon_SCAN", Horizon_SCAN, 1800);
//         nh.param<int>("lio_sam_udi/downsampleRate", downsampleRate, 1);
//         nh.param<float>("lio_sam_udi/lidarMinRange", lidarMinRange, 1.0);
//         nh.param<float>("lio_sam_udi/lidarMaxRange", lidarMaxRange, 1000.0);

//         nh.param<float>("lio_sam_udi/imuAccNoise", imuAccNoise, 0.01);
//         nh.param<float>("lio_sam_udi/imuGyrNoise", imuGyrNoise, 0.001);
//         nh.param<float>("lio_sam_udi/imuAccBiasN", imuAccBiasN, 0.0002);
//         nh.param<float>("lio_sam_udi/imuGyrBiasN", imuGyrBiasN, 0.00003);
//         nh.param<float>("lio_sam_udi/imuGravity", imuGravity, 9.80511);
//         nh.param<float>("lio_sam_udi/imuRPYWeight", imuRPYWeight, 0.01);
//         nh.param<vector<double>>("lio_sam_udi/extrinsicRot", extRotV, vector<double>());
//         nh.param<vector<double>>("lio_sam_udi/extrinsicRPY", extRPYV, vector<double>());
//         nh.param<vector<double>>("lio_sam_udi/extrinsicTrans", extTransV, vector<double>());
//         extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
//         extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
//         extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
//         extQRPY = Eigen::Quaterniond(extRPY);
//         integrated_rotation = Eigen::Matrix3d::Identity();
//         last_angular_velocity = Eigen::Vector3d::Zero();

//         nh.param<float>("lio_sam_udi/edgeThreshold", edgeThreshold, 0.1);
//         nh.param<float>("lio_sam_udi/surfThreshold", surfThreshold, 0.1);
//         nh.param<int>("lio_sam_udi/edgeFeatureMinValidNum", edgeFeatureMinValidNum, 10);
//         nh.param<int>("lio_sam_udi/surfFeatureMinValidNum", surfFeatureMinValidNum, 100);

//         nh.param<float>("lio_sam_udi/odometrySurfLeafSize", odometrySurfLeafSize, 0.2);
//         nh.param<float>("lio_sam_udi/mappingCornerLeafSize", mappingCornerLeafSize, 0.2);
//         nh.param<float>("lio_sam_udi/mappingSurfLeafSize", mappingSurfLeafSize, 0.2);

//         nh.param<float>("lio_sam_udi/z_tollerance", z_tollerance, FLT_MAX);
//         nh.param<float>("lio_sam_udi/rotation_tollerance", rotation_tollerance, FLT_MAX);

//         nh.param<int>("lio_sam_udi/numberOfCores", numberOfCores, 2);
//         nh.param<double>("lio_sam_udi/mappingProcessInterval", mappingProcessInterval, 0.15);

//         nh.param<float>("lio_sam_udi/surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold, 1.0);
//         nh.param<float>("lio_sam_udi/surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold, 0.2);
//         nh.param<float>("lio_sam_udi/surroundingKeyframeDensity", surroundingKeyframeDensity, 1.0);
//         nh.param<float>("lio_sam_udi/surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 50.0);

//         nh.param<bool>("lio_sam_udi/loopClosureEnableFlag", loopClosureEnableFlag, false);
//         nh.param<float>("lio_sam_udi/loopClosureFrequency", loopClosureFrequency, 1.0);
//         nh.param<int>("lio_sam_udi/surroundingKeyframeSize", surroundingKeyframeSize, 50);
//         nh.param<float>("lio_sam_udi/historyKeyframeSearchRadius", historyKeyframeSearchRadius, 10.0);
//         nh.param<float>("lio_sam_udi/historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, 30.0);
//         nh.param<int>("lio_sam_udi/historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
//         nh.param<float>("lio_sam_udi/historyKeyframeFitnessScore", historyKeyframeFitnessScore, 0.3);

//         nh.param<float>("lio_sam_udi/globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius, 1e3);
//         nh.param<float>("lio_sam_udi/globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity, 10.0);
//         nh.param<float>("lio_sam_udi/globalMapVisualizationLeafSize", globalMapVisualizationLeafSize, 1.0);

//         usleep(100);
//     }
// };