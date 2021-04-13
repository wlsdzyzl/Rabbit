#ifndef RABBIT_UTIL_H
#define RABBIT_UTIL_H
#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/range_image/range_image_spherical.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>

#include <memory>
#include <random>
#include <sys/stat.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/Eigenvalues> 
#include <Eigen/Eigen>
#include <sophus/se3.hpp>

#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m" /* Yellow */
#define BLUE    "\033[34m" /* Blue */
#define PURPLE  "\033[35m" /* Purple */
#define D_GREEN "\033[36m" /* Dark Green */
#define COLOR_RANGE 512
namespace rabbit
{
    typedef pcl::PointXYZI PointType;
    typedef pcl::PointXYZINormal PointTypeN;
    typedef pcl::PointCloud<pcl::PointXYZI>  PointCloud;
    typedef pcl::PointCloud<pcl::PointXYZINormal>  PointCloudN;
    typedef pcl::PointCloud<pcl::PointXYZINormal>::Ptr  PointCloudNPtr;
    typedef pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudPtr;
    typedef pcl::PointCloud<pcl::PointXYZRGB>  PointCloudRGB;
    typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudRGBPtr;
    typedef pcl::PointCloud<pcl::Normal> PCDNormal;
    typedef pcl::PointCloud<pcl::Normal>::Ptr PCDNormalPtr;

    typedef pcl::FPFHSignature33 FPFH;
    typedef pcl::PointCloud<pcl::FPFHSignature33> PCDFPFH;
    typedef pcl::PointCloud<pcl::FPFHSignature33>::Ptr PCDFPFHPtr;
    // typedef pcl::PointCloud<pcl::VFHSignature308> PCDVFH;
    // typedef pcl::PointCloud<pcl::VFHSignature308>::Ptr PCDVFHPtr;
    typedef pcl::RangeImage RangeIm;
    typedef pcl::RangeImage::Ptr RangeImPtr;
    typedef pcl::RangeImageSpherical RangeImSph;
    typedef pcl::RangeImageSpherical::Ptr RangeImSphPtr;
    // narf descriptor
    typedef pcl::PointCloud<pcl::Narf36> PCDNARF;
    typedef pcl::PointCloud<pcl::Narf36>::Ptr PCDNARFPtr;

    typedef Eigen::Matrix<float, 4, 4> Mat4f;
    typedef std::vector<Mat4f, Eigen::aligned_allocator<Mat4f> > Mat4fList; 

    typedef Eigen::Matrix<double, 4, 4> Mat4;
    typedef std::vector<Mat4, Eigen::aligned_allocator<Mat4> > Mat4List; 
    
    typedef Eigen::Vector3f Vec3f;
    typedef std::vector<Vec3f, Eigen::aligned_allocator<Vec3f> > Vec3fList; 

    typedef Eigen::Matrix<double, 3, 1> Vec3;
    typedef std::vector<Vec3, Eigen::aligned_allocator<Vec3> > Vec3List; 


    typedef Sophus::SE3f SE3f;
    typedef std::vector<SE3f, Eigen::aligned_allocator<SE3f>> SE3fList;

    typedef Sophus::SE3d SE3;
    typedef std::vector<SE3, Eigen::aligned_allocator<SE3>> SE3List;


    typedef Eigen::Matrix<float, 6, 1> Vec6f;
    typedef Eigen::Matrix<double, 6, 1> Vec6;

    //to split a string, or reversely split a string
    std::vector<std::string> Split(const std::string &str, const std::string &delim, int split_times = -1);
    std::vector<std::string> RSplit(const std::string &str, const std::string &delim, int split_times = -1);
    bool DirExists(const std::string & folder_name);
    bool FileExists(const std::string & file_name);
    bool Exists(const std::string &f);
    bool MakeDir(const std::string & folder_name);
    std::string AbsolutePath(const std::string &f);
    void ListFileNames(const std::string &folder_name, std::vector<std::string> &files);
    void ListFilePaths(const std::string &folder_name, std::vector<std::string> &filepaths);
    int ExtractIDFromPath(const std::string &path);

    void ColorRemapping(const std::vector<float> &values, Vec3fList &mapped_color);
    void ColorizePointCloud(const PointCloud &pcd, PointCloudRGB &mapped_pcd);
    void ColorizePointCloud(const PointCloud &pcd, const Vec3f &c, PointCloudRGB &mapped_pcd);

    void EstimateNormal(const PointCloud &pcd, PCDNormal &n, float search_radius);
    // from degree to radian
    inline double Rad2Deg(double radians)
    {
    return radians * 180.0 / M_PI;
    }
    // from radian to degree
    inline double Deg2Rad(double degrees)
    {
    return degrees * M_PI / 180.0;
    }
    void RemoveClosedPointCloud(const PointCloud&cloud_in,
                                PointCloud &cloud_out, float thres = 0.1);
    void FilterPCD(const PointCloud &cloud_in, PointCloud &cloud_out, float x_leaf_size = 0.2, float y_leaf_size = 0.2, float z_leaf_size = 0.2);
}
#endif