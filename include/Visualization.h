/*
The file is related to visualization. 
We will have two method. The first is a wrapper of pointcloud publisher, so it actually needs you to run the rviz  receive the message.
The second class is just the visualizer from PCL.
*/
#ifndef RABBIT_VISUALIZATION_H
#define RABBIT_VISUALIZATION_H
#include "Utils/Utils.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl_conversions/pcl_conversions.h>

namespace rabbit
{
namespace visualization
{
    using namespace util;
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

            viewer->addCoordinateSystem (1.0, "coor");
            viewer->initCameraParameters ();
        }
        void SetOrigin(const SE3 &ori)
        {
            viewer->updateCoordinateSystemPose("coor", Eigen::Affine3f(ori.matrix().cast<float>()));
            Vec3 current_pos = ori.translation();
            Vec3 delta_pos = current_pos - follow_car_pos;
            follow_car_pos = current_pos;

            std::vector<pcl::visualization::Camera> cameras;
            viewer->getCameras(cameras);

            Vec3 pos = Vec3(cameras[0].pos[0], cameras[0].pos[1], cameras[0].pos[2]);
            Vec3 focal = Vec3(cameras[0].focal[0], cameras[0].focal[1], cameras[0].focal[2]);
            // std::cout<<pos.transpose()<<std::endl;
            // std::cout<<focal.transpose()<<std::endl;
            // std::cout<<view.transpose()<<std::endl;

            pos += delta_pos;
            focal += delta_pos;

            viewer->setCameraPosition(
                pos(0), pos(1), pos(2),
                focal(0), focal(1), focal(2),
                cameras[0].view[0], cameras[0].view[1], cameras[0].view[2]
            );
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
        Vec3 follow_car_pos = Vec3(0, 0, 0);
    };
}
}
#endif