#ifndef RABBIT_FRAME_H
#define RABBIT_FRAME_H
#include "Utils/Utils.h"
#include <pcl/features/vfh.h>
#include <pcl/features/fpfh.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/voxel_grid.h>
namespace rabbit
{
    // enum SupportedFeature
    // {
    //     FPFH, LOAM
    // };
    using namespace util;
    struct Frame
    {
        int frame_id;
        PointCloudPtr pcd;
        PCDFPFHPtr fpfh;
        // PCDVFHPtr vfh;
        PCDNormalPtr normal;
        PointCloudPtr sharp_points;
        PointCloudPtr less_sharp_points;
        PointCloudPtr flat_points;
        PointCloudPtr less_flat_points;
        PointCloudPtr ground_points;
        PointCloudPtr less_ground_points;
        pcl::KdTreeFLANN<PointType>::Ptr less_sharp_kdtree;
        pcl::KdTreeFLANN<PointType>::Ptr less_flat_kdtree;
        pcl::KdTreeFLANN<PointType>::Ptr less_ground_kdtree;
        // matching on range image
        // for ros bag
        double timestamp;
        RangeImSphPtr range_image;
        std::vector<int> keypoint_indices;
        PCDNARFPtr narf_deps;
        // from camera coordinate to world coordinate
        // from current to last
        SE3 pose;
        Frame() = default;
        Frame(const sensor_msgs::PointCloud2 &laser_cloud_msg)
        {
            pcd = PointCloudPtr( new PointCloud ());
            pcl::fromROSMsg(laser_cloud_msg, *pcd);
            std::vector<int> indices;
            // prepropose
            pcl::removeNaNFromPointCloud(*pcd, *pcd, indices);
            RemoveClosedPointCloud(*pcd, *pcd, mininum_range);
            timestamp = MsgTime(laser_cloud_msg);            
        }
        Frame(const PointCloud &_pcd)
        {
            pcd = PointCloudPtr( new PointCloud (_pcd));
            
            std::vector<int> indices;
            // prepropose
            pcl::removeNaNFromPointCloud(*pcd, *pcd, indices);
            RemoveClosedPointCloud(*pcd, *pcd, mininum_range);
            // timestamp = MsgTime(laser_cloud_msg);            
        }
        // void LoadFromMsg(const sensor_msgs::PointCloud2 &laser_cloud_msg);
        void SetPCD(const PointCloud &_pcd);
        void ComputeNormal();
        void ComputeFPFH();
        // void ComputeVFH();
        void ComputeLOAMFeature();

        // features on range image
        void CreateRangeImage();
        void ComputeNARF();
        static float normal_search_radius;
        static float fpfh_search_radius;
        // parameters related to loam
        // note that also loam is feature based, it does need a initial guess.
        static int parts_n;
        static int sharp_points_n_each_part;
        static int less_sharp_points_n_each_part;
        static int flat_points_n_each_part;
        // static int less_flat_points_n_each_part = 20; 
        static int ground_points_n;
        static int lidar_ring_n;
        static float mininum_range;
        // 10hz 
        static float scan_period;
        static Vec3 ground_normal;
        // for range image
        static float angular_resolution_x;
        static float angular_resolution_y;
        static float max_angle_width;
        static float max_angle_height;
        static float support_size;
        static bool ground_removal;
        static bool ground_extraction;
        bool is_keyframe = false;
        Vec3 ground_plane_normal;
        double ground_plane_dist;
    };
}
#endif