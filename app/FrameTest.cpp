#include "IO.h"
#include "Frame.h"
#include "Visualization.h"
using namespace rabbit;
int main(int argc, char **argv)
{
    if(argc != 2)
    {
        std::cout<<"usage: FrameTest [pcd_file] or [pcd_folder]"<<std::endl;
        return 0;
    }
    if(DirExists(argv[1]))
    {
        std::vector<std::string> seq; 
        io::GetPCDSequence(argv[1], seq);
        std::cout<<"Find "<< seq.size()<<" point clouds."<<std::endl;        
        visualization::Visualizer visualizer;
        for(size_t i = 0; i != seq.size(); ++i)
        {
            PointCloud pcd; 
            io::LoadPCD(seq[i], pcd);
            Frame frame; 
            frame.SetPCD(pcd);
            frame.CreateRangeImage();
            visualizer.SetPCD(*frame.pcd);
            visualizer.SetRangeImage(*frame.range_image);
            visualizer.ShowOnce();
        }
    }
    else
    {
        PointCloud pcd; 
        io::LoadPCD(argv[1], pcd);
        Frame frame; 
        frame.SetPCD(pcd);
        frame.CreateRangeImage();
        visualization::Visualizer visualizer;
        visualizer.SetPCD(*frame.pcd);
        visualizer.SetRangeImage(*frame.range_image);
        visualizer.Show();
    }
    return 0;
}