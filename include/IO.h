/*
This file is IO-related, including read pointclouds from a folder, and store  pose_graph, and point cloud.
Created bu Guoqing 2021.3.24 
*/
#ifndef RABBIT_IO_H
#define RABBIT_IO_H
#include "Util.h"
namespace rabbit
{
    namespace io
    {
        void LoadPCD(const std::string &s, PCDXYZ &pcd);
        void LoadPLY(const std::string &s, PCDXYZ &pcd);
        void LoadFile(const std::string &s, PCDXYZ &pcd);

        void LoadPCD(const std::string &s, PCDXYZI &pcd);
        void LoadPLY(const std::string &s, PCDXYZI &pcd);
        void LoadFile(const std::string &s, PCDXYZI &pcd);

        void LoadPCD(const std::string &s, PCDXYZL &pcd);
        void LoadPLY(const std::string &s, PCDXYZL &pcd);
        void LoadFile(const std::string &s, PCDXYZL &pcd);

        void LoadPCD(const std::string &s, PCDXYZRGB &pcd);
        void LoadPLY(const std::string &s, PCDXYZRGB &pcd);
        void LoadFile(const std::string &s, PCDXYZRGB &pcd);
        void GetPCDSequence(const std::string &folder, std::vector<std::string> &sequence, bool sorted = true);

        // TODO: Load PoseGraph
        void WritePCD(const std::string &s, const PCDXYZ & pcd, bool use_ascii = false);
        void WritePLY(const std::string &s, const PCDXYZ & pcd, bool use_ascii = false);

        void WritePCD(const std::string &s, const PCDXYZI & pcd, bool use_ascii = false);
        void WritePLY(const std::string &s, const PCDXYZI & pcd, bool use_ascii = false);

        void WritePCD(const std::string &s, const PCDXYZL & pcd, bool use_ascii = false);
        void WritePLY(const std::string &s, const PCDXYZL & pcd, bool use_ascii = false);

        void WritePCD(const std::string &s, const PCDXYZRGB & pcd, bool use_ascii = false);
        void WritePLY(const std::string &s, const PCDXYZRGB & pcd, bool use_ascii = false);        
        // TODO: Write PoseGraph
    }
}
#endif