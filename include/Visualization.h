/*
The file is related to visualization. 
We will have two method. The first is a wrapper of pointcloud publisher, so it actually needs you to run the rviz  receive the message.
The second class is just the visualizer from PCL.
*/
#ifndef RABBIT_VISUALIZATION_H
#define RABBIT_VISUALIZATION_H
#include "Util.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/range_image_visualizer.h>
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
                pcdc_ptr = PointCloudRGBPtr( new PointCloudRGB());

                viewer->addPointCloud(pcdc_ptr, "pcdc");
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "pcdc");

                viewer->addCoordinateSystem (1.0);
                viewer->initCameraParameters ();
            }
            // void SetPCD(const PCDXYZ &pcd)
            // {
            //     *pcd_ptr = pcd;
            //     viewer->updatePointCloud(pcd_ptr, "pcd");
            // }
            // void SetPCD(const PCDXYZL &pcdl)
            // {
            //     *pcdl_ptr = pcdl;
            //     viewer->updatePointCloud(pcdl_ptr, "pcdl");
            // }
            // histogram equalization
            void SetPCD(const PointCloud &pcdi)
            {
                ColorizePointCloud(pcdi, *pcdc_ptr);
                viewer->updatePointCloud(pcdc_ptr, "pcdc");
            }

            void SetPCD(const PointCloudRGB &pcdc)
            {
                *pcdc_ptr = pcdc;
                viewer->updatePointCloud(pcdc_ptr, "pcdc");
            }
            void SetRangeImage(const RangeImSph &rim)
            {
                // // we don't support to show dynamic range image
                // rangeim_ptr = RangeImPtr(new  RangeImSph (rim) );
                // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> rangeim_color_handler (rangeim_ptr, 0, 0, 0);
                // viewer->addPointCloud (rangeim_ptr, rangeim_color_handler, "range image");
                // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
                // setViewerPose(*viewer, rangeim_ptr->getTransformationToWorldSystem ());
                if(!widget_ptr)
                widget_ptr = new pcl::visualization::RangeImageVisualizer ("Range image");
                widget_ptr->showRangeImage(rim);
            }
            void Show()
            {
                while(true)
                {
                    viewer->spinOnce(100);
                    if(widget_ptr) widget_ptr->spinOnce(100);
                }
            }
            void ShowOnce()
            {
                viewer->spinOnce(100);
                if(widget_ptr)  widget_ptr->spinOnce(100);
            }
            ~Visualizer()
            {
                delete widget_ptr;
            }
            pcl::visualization::PCLVisualizer::Ptr viewer;
            bool publisher_initialized = false;
            PointCloudRGBPtr pcdc_ptr;
            // RangeImPtr rangeim_ptr;
            pcl::visualization::RangeImageVisualizer * widget_ptr = nullptr;
        };
    }
}
#endif