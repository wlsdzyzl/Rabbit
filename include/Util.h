#ifndef RABBIT_UTIL_H
#define RABBIT_UTIL_H
#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
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
    typedef pcl::PointCloud<pcl::PointXYZ>  PCDXYZ;
    typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PCDXYZPtr;
    typedef pcl::PointCloud<pcl::PointXYZI>  PCDXYZI;
    typedef pcl::PointCloud<pcl::PointXYZI>::Ptr PCDXYZIPtr;
    typedef pcl::PointCloud<pcl::PointXYZL>  PCDXYZL;
    typedef pcl::PointCloud<pcl::PointXYZL>::Ptr PCDXYZLPtr;
    typedef pcl::PointCloud<pcl::PointXYZRGB>  PCDXYZRGB;
    typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCDXYZRGBPtr;
    typedef Eigen::Vector3f Vec3f;
    typedef std::vector<Vec3f, Eigen::aligned_allocator<Vec3f> > Vec3fList; 
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
    void ColorizePCDXYZI(const PCDXYZI &pcd, PCDXYZRGB &mapped_pcd);
 
}
#endif