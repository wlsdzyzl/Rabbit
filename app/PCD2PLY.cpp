#include "Utils/IO.h"
using namespace rabbit;
using namespace util;
int main(int argc, char **argv)
{
    if(argc != 3)
    {
        std::cout<<"usage: PCD2PLY [pcd_file] or [pcd_folder] [output_file] or [output folder]"<<std::endl;
        return 0;
    }
    if(DirExists(argv[1]))
    {
        std::vector<std::string> seq; 
        GetPCDSequence(argv[1], seq);
        std::cout<<"Find "<< seq.size()<<" point clouds."<<std::endl;        
        MakeDir(argv[2]);
        for(size_t i = 0; i != seq.size(); ++i)
        {
            PointCloud pcd; 
            LoadPCD(seq[i], pcd);
            std::vector<std::string> strs = RSplit(seq[i], "/", 1);
            std::string filename = seq[i];
            if(strs .size()> 1) filename = strs[1];
            WritePLY(argv[2] + std::string("/") +filename + std::string(".ply") , pcd);
        }
    }
    else
    {
        PointCloud pcd; 
        LoadPCD(argv[1], pcd);
        WritePLY(argv[2] , pcd, true);
    }
    return 0;
}