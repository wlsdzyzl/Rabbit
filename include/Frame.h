#ifndef RABBIT_FRAME_H
#define RABBIT_FRAME_H
#include "Util.h"
#include <pcl/features/vfh.h>
#include <pcl/features/fpfh.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
namespace rabbit
{
    // enum SupportedFeature
    // {
    //     FPFH, LOAM
    // };
    struct Frame
    {
        int frame_id;
        PCDXYZIPtr pcd;
        PCDFPFHPtr fpfh;
        // PCDVFHPtr vfh;
        PCDNormalPtr normal;
        PCDXYZIPtr sharp_points;
        PCDXYZIPtr less_sharp_points;
        PCDXYZIPtr flat_points;
        PCDXYZIPtr less_flat_points;

        // matching on range image

        RangeImSphPtr range_image;
        std::vector<int> keypoint_indices;
        PCDNARFPtr narf_deps;

        void LoadFromMsg(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);
        void SetPCD(const PCDXYZI &_pcd);
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
        static int lidar_ring_n;
        static float mininum_range;
        // 10hz 
        static float scan_period;

        // for range image
        static float angular_resolution_x;
        static float angular_resolution_y;
        static float max_angle_width;
        static float max_angle_height;
        static float support_size;
    };
}
#endif