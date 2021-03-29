#include "IO.h"
using namespace rabbit;
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
        io::GetPCDSequence(argv[1], seq);
        std::cout<<"Find "<< seq.size()<<" point clouds."<<std::endl;        
        MakeDir(argv[2]);
        for(size_t i = 0; i != seq.size(); ++i)
        {
            PCDXYZI pcd; 
            io::LoadPCD(seq[i], pcd);
            std::vector<std::string> strs = RSplit(seq[i], "/", 1);
            std::string filename = seq[i];
            if(strs .size()> 1) filename = strs[1];
            io::WritePLY(argv[2] + std::string("/") +filename + std::string(".ply") , pcd);
        }
    }
    else
    {
        PCDXYZI pcd; 
        io::LoadPCD(argv[1], pcd);
        io::WritePLY(argv[2] , pcd, true);
    }
    return 0;
}