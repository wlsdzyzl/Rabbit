#include "Odometry.h"
#include "Utils/IO.h"
#include "Visualization.h"
using namespace rabbit;
using namespace util;
int main(int argc, char **argv)
{
    if(argc == 3)
    {
        std::vector<std::string> seq; 
        GetPCDSequence(argv[1], seq);
        std::cout<<"Find "<< seq.size()<<" point clouds."<<std::endl;        
        visualization::Visualizer visualizer;        
        Frame last_frame;
        PointCloud pcd;
        PointCloud global_pcd;
        PointCloudRGB pcd_rgb, global_pcd_rgb;
        LidarOdometry lo;
        SE3 global_T;
        std::string method = argv[2];
        for(size_t i = 0; i != seq.size(); ++i)
        {
            LoadPCD(seq[i], pcd);
            Frame current_frame;
            current_frame.SetPCD(pcd);
            SE3 T;
            if(method == "loam") current_frame.ComputeLOAMFeature();
            if(i > 0)
            {
                if( method == "icp")
                lo.ICP(last_frame, current_frame, T);
                else if(method == "gicp")
                lo.GICP(last_frame, current_frame, T);
                else if(method == "ndt")
                lo.NDT(last_frame, current_frame, T);
                else if(method == "loam")
                lo.Loam(last_frame, current_frame, T);
                global_T = T * global_T;
            }
            pcl::transformPointCloud (pcd, pcd, global_T.inverse().matrix());
            if(i % 10 == 0)
            {
                FilterPCD(global_pcd, global_pcd, 0.5, 0.5, 0.5);
                ColorizePointCloud(pcd, Vec3f(0, 1, 0) ,pcd_rgb);
                ColorizePointCloud(global_pcd, global_pcd_rgb);
                global_pcd_rgb += pcd_rgb;
                visualizer.SetPCD(global_pcd_rgb);
                visualizer.ShowOnce();
                global_pcd += pcd;
            }
            std::cout<<"frame id: "<<i<<global_T.log().transpose()<<std::endl;
            last_frame = current_frame;
        }
        WritePLY("./odometry_test_"+method+".ply", global_pcd_rgb);
    }
    else if(argc == 4)
    {
        PointCloud source, target;
        Frame sf, tf;
        LoadPCD(argv[1], source);
        LoadPCD(argv[2], target);
        sf.SetPCD(source);
        tf.SetPCD(target);   
        LidarOdometry lo;
        SE3 T;
        std::string method = argv[3];
        if( method == "icp")
        lo.ICP(sf, tf, T);
        else if(method == "gicp")
        lo.GICP(sf, tf, T);
        else if(method == "ndt")
        lo.NDT(sf, tf, T);
        else if(method == "loam")
        {
            sf.ComputeLOAMFeature();
            tf.ComputeLOAMFeature();
            lo.Loam(sf, tf, T);
        }
        else
        {
            std::cout<<"unsupported method"<<std::endl;
            return -1;
        }
        std::cout<<T.matrix()<<std::endl;
        visualization::Visualizer visualizer;
        PointCloudRGB s_rgb, t_rgb, final_rgb;
        ColorizePointCloud(source, Vec3f(1, 0, 0), s_rgb);
        ColorizePointCloud(target, Vec3f(0, 1, 0), t_rgb);
        pcl::transformPointCloud (s_rgb, s_rgb, T.matrix());
        final_rgb += s_rgb;
        final_rgb += t_rgb;
        visualizer.SetPCD(final_rgb);
        visualizer.Show();
        return 0;
    }
    else
    {
        std::cout<<"usage: OdometryTest [source_pcd] [target_pcd] [method] or \n\tOdometryTest [pcd_folder]  [method] "<<std::endl;
        return 0;
    }
}