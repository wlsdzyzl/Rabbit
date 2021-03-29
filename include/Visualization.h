/*
The file is related to visualization. 
We will have two method. The first is a wrapper of pointcloud publisher, so it actually needs you to run the rviz  receive the message.
The second class is just the visualizer from PCL.
*/
#ifndef RABBIT_VISUALIZATION_H
#define RABBIT_VISUALIZATION_H
#include "Util.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl_conversions/pcl_conversions.h>
#include "IO.h"

namespace rabbit
{
    namespace visualization
    {
        class Visualizer
        {
            public:
            Visualizer()
            {
                viewer =  pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer ("rabbit"));
                publisher_initialized = false;
                pcd_ptr = PCDXYZPtr(new PCDXYZ ());
                pcdl_ptr = PCDXYZLPtr(new PCDXYZL());
                pcdc_ptr = PCDXYZRGBPtr( new PCDXYZRGB());
                viewer->addPointCloud(pcd_ptr, "pcd");
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "pcd");

                viewer->addPointCloud(pcdl_ptr, "pcdl");
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "pcdl");

                viewer->addPointCloud(pcdc_ptr, "pcdc");
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "pcdc");
                viewer->addCoordinateSystem (1.0);
                viewer->initCameraParameters ();
            }
            void SetPCD(const PCDXYZ &pcd)
            {
                *pcd_ptr = pcd;
                viewer->updatePointCloud(pcd_ptr, "pcd");
            }
            // histogram equalization
            void SetPCD(const PCDXYZI &pcdi)
            {
                ColorizePCDXYZI(pcdi, *pcdc_ptr);
                viewer->updatePointCloud(pcdc_ptr, "pcdc");
            }
            void SetPCD(const PCDXYZL &pcdl)
            {
                *pcdl_ptr = pcdl;
                viewer->updatePointCloud(pcdl_ptr, "pcdl");
            }
            void SetPCD(const PCDXYZRGB &pcdc)
            {
                *pcdc_ptr = pcdc;
                viewer->updatePointCloud(pcdc_ptr, "pcdc");
            }
            void SetPublisher(const ros::Publisher &ros_pub)
            {
                publisher = ros_pub;
                publisher_initialized = true;
            }
            void Publish(const PCDXYZ &pcd)
            {
                if(!publisher_initialized)
                {
                    std::cout<<RED<<"[ERROR]::[visualizer]::You need to set publisher firstly."<<RESET<<std::endl;
                    return;
                }
                sensor_msgs::PointCloud2 output;
                pcl::toROSMsg(pcd, output);
                publisher.publish(output);
            }
            void Show()
            {
                while(true)
                viewer->spinOnce(100);
            }
            void ShowOnce()
            {
                viewer->spinOnce(100);
            }
            ros::Publisher publisher;
            pcl::visualization::PCLVisualizer::Ptr viewer;
            bool publisher_initialized = false;
            PCDXYZPtr pcd_ptr;
            PCDXYZLPtr pcdl_ptr;
            PCDXYZRGBPtr pcdc_ptr;
        };
    }
}
#endif