#include "Utils/IO.h"
#include <pcl/io/ply_io.h>
#include <algorithm>
namespace rabbit
{
    namespace utils
    {
         void LoadPCD(const std::string &filename, PointCloud &pcd)
         {
             pcl::io::loadPCDFile(filename.c_str(),  pcd);
         }
        void LoadPLY(const std::string &filename, PointCloud &pcd)
        {
            pcl::io::loadPLYFile(filename.c_str(), pcd);
        }
        void LoadFile(const std::string &filename, PointCloud &pcd)
        {
            std::string suffix = "";
            std::vector<std::string> strs = RSplit(filename, ".", 1);
            if(strs.size() == 2) suffix = strs[1];
            boost::algorithm::to_lower(suffix);
            if(suffix == "pcd")    LoadPCD(filename, pcd);
            else if(suffix == "ply")    LoadPLY(filename, pcd);
            else 
                std::cout<<YELLOW<<"[WARNING]::[IO]::Rabbit only support .ply or .pcd file."<<RESET<<std::endl;
        }
         void LoadPCD(const std::string &filename, PointCloudRGB &pcd)
         {
             pcl::io::loadPCDFile(filename.c_str(),  pcd);
         }
        void LoadPLY(const std::string &filename, PointCloudRGB &pcd)
        {
            pcl::io::loadPLYFile(filename.c_str(), pcd);
        }
        void LoadFile(const std::string &filename, PointCloudRGB &pcd)
        {
            std::string suffix = "";
            std::vector<std::string> strs = RSplit(filename, ".", 1);
            if(strs.size() == 2) suffix = strs[1];
            boost::algorithm::to_lower(suffix);
            if(suffix == "pcd")    LoadPCD(filename, pcd);
            else if(suffix == "ply")    LoadPLY(filename, pcd);
            else 
                std::cout<<YELLOW<<"[WARNING]::[IO]::Rabbit only support .ply or .pcd file."<<RESET<<std::endl;
        }

        bool ComparePCDPath(const std::string &p1, const std::string &p2)
        {
            int id1 = ExtractIDFromPath(p1);
            int id2 = ExtractIDFromPath(p2);
            return id1 < id2;
        }
        void GetPCDSequence(const std::string &folder, std::vector<std::string> &sequence, bool sorted)
        {
            // get the paths of all the files whose suffix is pcd
            sequence.clear();
            std::vector<std::string> filepaths;
            ListFilePaths(folder, filepaths);
            for(size_t i = 0; i != filepaths.size(); ++i)
            {
                std::vector<std::string> strs = RSplit(filepaths[i], ".", 1);
                if(strs.size() == 2 && strs[1] == std::string("pcd"))
                sequence.push_back(filepaths[i]);
            }
            if(sorted)
            std::sort(sequence.begin(), sequence.end(), ComparePCDPath);
        }
        // TODO: Load PoseGraph

        void WritePCD(const std::string &filename, const PointCloud & pcd, bool use_ascii)
        {
            pcl::io::savePCDFile(filename.c_str(), pcd, !use_ascii);
        }
        void WritePLY(const std::string &filename, const PointCloud & pcd, bool use_ascii)
        {
                pcl::io::savePLYFile(filename.c_str(), pcd, !use_ascii);
        }

        void WritePCD(const std::string &filename, const PointCloudRGB & pcd, bool use_ascii)
        {
            pcl::io::savePCDFile(filename.c_str(), pcd, !use_ascii);
        }
        void WritePLY(const std::string &filename, const PointCloudRGB & pcd, bool use_ascii)
        {
                pcl::io::savePLYFile(filename.c_str(), pcd, !use_ascii);
        }
        // TODO: Write PoseGraph       
    }
}