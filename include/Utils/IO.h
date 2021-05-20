/*
This file is IO-related, including read pointclouds from a folder, and store  pose_graph, and point cloud.
Created bu Guoqing 2021.3.24 
*/
#ifndef RABBIT_IO_H
#define RABBIT_IO_H
#include "Utils/Utils.h"
namespace rabbit
{
    namespace util
    {
        void LoadPCD(const std::string &s, PointCloud &pcd);
        void LoadPLY(const std::string &s, PointCloud &pcd);
        void LoadFile(const std::string &s, PointCloud &pcd);

        void LoadPCD(const std::string &s, PointCloudRGB &pcd);
        void LoadPLY(const std::string &s, PointCloudRGB &pcd);
        void LoadFile(const std::string &s, PointCloudRGB &pcd);
        void GetPCDSequence(const std::string &folder, std::vector<std::string> &sequence, bool sorted = true);

        // TODO: Load PoseGraph
        void WritePCD(const std::string &s, const PointCloud & pcd, bool use_ascii = false);
        void WritePLY(const std::string &s, const PointCloud & pcd, bool use_ascii = false);

        void WritePCD(const std::string &s, const PointCloudRGB & pcd, bool use_ascii = false);
        void WritePLY(const std::string &s, const PointCloudRGB & pcd, bool use_ascii = false);        
        // TODO: Write PoseGraph
    }
}
#endif