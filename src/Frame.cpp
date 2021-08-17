#include "Frame.h"
#include <cmath>
// #include <pcl/filters/voxel_grid.h>
namespace rabbit
{
    float cloud_curvature[400000];
    int cloud_sorted_id[400000];
    int cloud_neighbor_picked[400000];
    int cloud_label[400000];
    float Frame::normal_search_radius = 0.5;
    float Frame::fpfh_search_radius = 0.05;
    // parameters related to loam
    // note that also loam is feature based, it does need a initial guess.
    int Frame::parts_n = 6;
    int Frame::sharp_points_n_each_part= 2;
    int Frame::less_sharp_points_n_each_part = 20;
    int Frame::flat_points_n_each_part = 4;
    int Frame::ground_points_n = 25;
    // int less_flat_points_n_each_part = 20; 
    int Frame::lidar_ring_n = 16;
    float Frame::mininum_range = 0.1;
    // 10hz 
    float Frame::scan_period = 0.1;

    // for range image 
    float Frame::angular_resolution_x = Deg2Rad(0.2);
    float Frame::angular_resolution_y = Deg2Rad(2);
    float Frame::max_angle_width = Deg2Rad(360);
    float Frame::max_angle_height = Deg2Rad(30);
    Vec3 Frame::ground_normal = Vec3(0, 0, 1);
    bool Frame::ground_extraction = false;
    bool Frame::ground_removal = true;

    void Frame::ComputeNormal()
    {
        normal = PCDNormalPtr(new PCDNormal ());
        EstimateNormal(*pcd, *normal, normal_search_radius);
    }
    void Frame::ComputeFPFH()
    {
        // Create the FPFH estimation class, and pass the input dataset+normals to it
        pcl::FPFHEstimation<PointType, pcl::Normal, pcl::FPFHSignature33> fpfh_filter;
        fpfh_filter.setInputCloud (pcd);
        fpfh_filter.setInputNormals (normal);
        // alternatively, if cloud is of tpe PointNormal, do fpfh.setInputNormals (cloud);

        // Create an empty kdtree representation, and pass it to the FPFH estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
        fpfh = PCDFPFHPtr(new PCDFPFH ());
        fpfh_filter.setSearchMethod (tree);
        // Use all neighbors in a sphere of radius 5cm
        // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
        fpfh_filter.setRadiusSearch (fpfh_search_radius);

        // Compute the features
        fpfh_filter.compute (*fpfh);
    }
    // VFH is used for object detection   
    // void Frame::ComputeVFH()
    // {
    //     // Create the VFH estimation class, and pass the input dataset+normals to it
    //     pcl::VFHEstimation<PointType, pcl::Normal, pcl::VFHSignature308> vfh_filter;
    //     vfh_filter.setInputCloud (pcd);
    //     vfh_filter.setInputNormals (normal);
    //     // alternatively, if cloud is of type PointNormal, do vfh.setInputNormals (cloud);

    //     // Create an empty kdtree representation, and pass it to the FPFH estimation object.
    //     // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    //     pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());
    //     vfh_filter.setSearchMethod (tree);

    //     // Output datasets
    //     vfh = PCDVFH (new PCDVFH ());

    //     // Compute the features
    //     vfh_filter.compute (*vfh);
    // }
    void Frame::SetPCD(const PointCloud &_pcd)
    {
        pcd = PointCloudPtr ( new PointCloud (_pcd));
    }
    // the code is modified from A-LOAM: scanRegistration.cpp
    void Frame::ComputeLOAMFeature()
    {
        // compute loam feature
        std::vector<int> scan_start_id(lidar_ring_n, -1);
        std::vector<int> scan_end_id(lidar_ring_n, -1);
        std::set<int> ground_point_ids;
        // distinguish each ring
        int cloud_size = pcd->points.size();
        // compute the orientation horizontally
        float start_ori = -atan2(pcd->points[0].y, pcd->points[0].x);
        float end_ori = -atan2(pcd->points[cloud_size - 1].y,
                            pcd->points[cloud_size - 1].x) + 2 * M_PI;
        int pixel_width = max_angle_width / angular_resolution_x;
        if (end_ori - start_ori > 3 * M_PI)
        {
            end_ori -= 2 * M_PI;
        }
        else if (end_ori - start_ori < M_PI)
        {
            end_ori += 2 * M_PI;
        }
        //printf("end Ori %f\n", end_ori);

        bool half_passed = false;
        int count = cloud_size;
        PointType point;
        // put each point cloud into the corresponding scans. If we have a 16-line lidar, there will be 16 scans in each frame.
        std::vector<PointCloud> laser_cloud_scans(lidar_ring_n);
        std::vector<std::vector<PointType>> laser_cloud_matrix(lidar_ring_n);
        if(ground_extraction)
        {
            point.intensity = -1;
            for(size_t i = 0; i != lidar_ring_n; ++i)
            {
                laser_cloud_matrix[i].resize(pixel_width, point);
            }
        }
        for (int i = 0; i < cloud_size; i++)
        {
            point.x = pcd->points[i].x;
            point.y = pcd->points[i].y;
            point.z = pcd->points[i].z;

            float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
            int scan_id = 0;
            if (lidar_ring_n == 16)
            {
                scan_id = int((angle + 15) / 2 + 0.5);
                if (scan_id > (lidar_ring_n - 1) || scan_id < 0)
                {
                    count--;
                    continue;
                }
            }
            else if (lidar_ring_n == 32)
            {
                scan_id = int((angle + 92.0/3.0) * 3.0 / 4.0);
                if (scan_id > (lidar_ring_n - 1) || scan_id < 0)
                {
                    count--;
                    continue;
                }
            }
            else if (lidar_ring_n == 64)
            {   
                if (angle >= -8.83)
                    scan_id = int((2 - angle) * 3.0 + 0.5);
                else
                    scan_id = lidar_ring_n / 2 + int((-8.83 - angle) * 2.0 + 0.5);

                // use [0 50]  > 50 remove outlies 
                if (angle > 2 || angle < -24.33 || scan_id > 50 || scan_id < 0)
                {
                    count--;
                    continue;
                }
            }
            else
            {
                std::cout<<RED<<"[ERROR]::[LoamFeature]::wrong scan number!"<<RESET<<std::endl;
                std::exit(0);
            }
            // compute current orientation
            float ori = -atan2(point.y, point.x);
            if (!half_passed)
            { 
                if (ori < start_ori - M_PI / 2)
                {
                    ori += 2 * M_PI;
                }
                else if (ori > start_ori + M_PI * 3 / 2)
                {
                    ori -= 2 * M_PI;
                }

                if (ori - start_ori > M_PI)
                {
                    half_passed = true;
                }
            }
            else
            {
                ori += 2 * M_PI;
                if (ori < end_ori - M_PI * 3 / 2)
                {
                    ori += 2 * M_PI;
                }
                else if (ori > end_ori + M_PI / 2)
                {
                    ori -= 2 * M_PI;
                }
            }
            // the intensity here is not the intensity you thought. 
            // it records something about distortion
            float rel_time = (ori - start_ori) / (end_ori - start_ori);
            int wid = rel_time * (pixel_width - 1);

            // for extraction of ground points
            if(ground_extraction)
            {
                point.intensity = laser_cloud_scans[scan_id].size();
                laser_cloud_matrix[scan_id][wid]= point; 
            }
            point.intensity = scan_id + scan_period * rel_time;
            laser_cloud_scans[scan_id].push_back(point);
            
        }
        cloud_size = count;
        PointCloudPtr laser_cloud = PointCloudPtr(new PointCloud());
        // start and end index, and there is a margin between each scans. Later we will compute curvatures for these points.
        for (int i = 0; i < lidar_ring_n; i++)
        { 
            if(ground_extraction)
            for(size_t j = 0; j != pixel_width; ++j)
            {
                if(laser_cloud_matrix[i][j].intensity >= 0)
                laser_cloud_matrix[i][j].intensity += laser_cloud->size();
            }
            scan_start_id[i] = laser_cloud->size() + 5;
            *laser_cloud += laser_cloud_scans[i];
            scan_end_id[i] = laser_cloud->size() - 6;
        }
        // add extraction of ground points
        if(ground_extraction)
        {
            for (int i = 0; i != pixel_width; ++i)
            {
                for(int j = 1; j < lidar_ring_n / 2 - 1; ++j)
                {
                    if(laser_cloud_matrix[j][i].intensity < 0 || laser_cloud_matrix[j-1][i].intensity < 0) continue;
                    float diffX = laser_cloud_matrix[j][i].x - laser_cloud_matrix[j-1][i].x;
                    float diffY = laser_cloud_matrix[j][i].y - laser_cloud_matrix[j-1][i].y;
                    float diffZ = laser_cloud_matrix[j][i].z - laser_cloud_matrix[j-1][i].z;
                    float angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY) ) * 180 / M_PI;
                    if(std::fabs(angle) < 10)
                    {
                        int curr_id = (int)(laser_cloud_matrix[j][i].intensity);
                        int last_id = (int)(laser_cloud_matrix[j - 1][i].intensity);
                        cloud_neighbor_picked[curr_id] = 1;
                        cloud_neighbor_picked[last_id] = 1;

                        cloud_label[curr_id] = 5;
                        cloud_label[last_id] = 5;
                        
                        ground_point_ids.insert(curr_id);
                        ground_point_ids.insert(last_id);
                    }

                }
            }
        }
        

        for (int i = 5; i < cloud_size - 5; i++)
        { 
            // estimate the curvature of current point
            float diffX = laser_cloud->points[i - 5].x + laser_cloud->points[i - 4].x + laser_cloud->points[i - 3].x + laser_cloud->points[i - 2].x + laser_cloud->points[i - 1].x - 10 * laser_cloud->points[i].x + laser_cloud->points[i + 1].x + laser_cloud->points[i + 2].x + laser_cloud->points[i + 3].x + laser_cloud->points[i + 4].x + laser_cloud->points[i + 5].x;
            float diffY = laser_cloud->points[i - 5].y + laser_cloud->points[i - 4].y + laser_cloud->points[i - 3].y + laser_cloud->points[i - 2].y + laser_cloud->points[i - 1].y - 10 * laser_cloud->points[i].y + laser_cloud->points[i + 1].y + laser_cloud->points[i + 2].y + laser_cloud->points[i + 3].y + laser_cloud->points[i + 4].y + laser_cloud->points[i + 5].y;
            float diffZ = laser_cloud->points[i - 5].z + laser_cloud->points[i - 4].z + laser_cloud->points[i - 3].z + laser_cloud->points[i - 2].z + laser_cloud->points[i - 1].z - 10 * laser_cloud->points[i].z + laser_cloud->points[i + 1].z + laser_cloud->points[i + 2].z + laser_cloud->points[i + 3].z + laser_cloud->points[i + 4].z + laser_cloud->points[i + 5].z;

            cloud_curvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
            cloud_sorted_id[i] = i;
            cloud_neighbor_picked[i] = 0;
            cloud_label[i] = 0;
        }

        ground_points = PointCloudPtr(new PointCloud());
        less_ground_points = PointCloudPtr(new PointCloud());
        sharp_points = PointCloudPtr(new PointCloud());
        less_sharp_points = PointCloudPtr(new PointCloud());
        flat_points = PointCloudPtr(new PointCloud());
        less_flat_points = PointCloudPtr(new PointCloud());

        std::vector<int> ground_point_ids_vec(ground_point_ids.begin(), ground_point_ids.end());

        std::sort (ground_point_ids_vec.begin(), ground_point_ids_vec.end(),  
            [&](int _i, int _j){return (cloud_curvature[_i]<cloud_curvature[_j]);});

        
        for(int i = 0; i != ground_point_ids_vec.size(); ++i)
        {
            int tmp_id = ground_point_ids_vec[i];
            if(ground_points->size() < ground_points_n && cloud_curvature[tmp_id] < 0.1)
            ground_points->push_back(laser_cloud->points[tmp_id]);
            less_ground_points->push_back(laser_cloud->points[tmp_id]);
        }
        for (int i = 0; i < lidar_ring_n; i++)
        {
            if( scan_end_id[i] - scan_start_id[i] < parts_n)
                continue;
            PointCloud::Ptr current_less_flat_points(new PointCloud);
            // seperate each scan into 6 parts
            for (int j = 0; j < 6; j++)
            {
                int sp = scan_start_id[i] + (scan_end_id[i] - scan_start_id[i]) * j / parts_n; 
                int ep = scan_start_id[i] + (scan_end_id[i] - scan_start_id[i]) * (j + 1) / parts_n - 1;
                std::sort (cloud_sorted_id + sp, cloud_sorted_id + ep + 1,  [&](int _i, int _j){return (cloud_curvature[_i]<cloud_curvature[_j]);});
                int largest_picked_n = 0;
                for (int k = ep; k >= sp; k--)
                {
                    int ind = cloud_sorted_id[k]; 

                    if (cloud_neighbor_picked[ind] == 0 &&
                        cloud_curvature[ind] > 0.1)
                    {

                        largest_picked_n++;
                        if (largest_picked_n <= sharp_points_n_each_part)
                        {                        
                            cloud_label[ind] = 2;
                            sharp_points->push_back(laser_cloud->points[ind]);
                            less_sharp_points->push_back(laser_cloud->points[ind]);
                        }
                        else if (largest_picked_n <= less_sharp_points_n_each_part)
                        {                        
                            // two sharp edge points, and 20 less sharp edge points
                            cloud_label[ind] = 1; 
                            less_sharp_points->push_back(laser_cloud->points[ind]);
                        }
                        else
                        {
                            break;
                        }

                        cloud_neighbor_picked[ind] = 1; 

                        for (int l = 1; l <= 5; l++)
                        {
                            //if  distance is larger than a threshold, we do not consider this point as a neighbor anymore.
                            float diffX = laser_cloud->points[ind + l].x - laser_cloud->points[ind + l - 1].x;
                            float diffY = laser_cloud->points[ind + l].y - laser_cloud->points[ind + l - 1].y;
                            float diffZ = laser_cloud->points[ind + l].z - laser_cloud->points[ind + l - 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                            {
                                break;
                            }

                            cloud_neighbor_picked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--)
                        {
                            float diffX = laser_cloud->points[ind + l].x - laser_cloud->points[ind + l + 1].x;
                            float diffY = laser_cloud->points[ind + l].y - laser_cloud->points[ind + l + 1].y;
                            float diffZ = laser_cloud->points[ind + l].z - laser_cloud->points[ind + l + 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                            {
                                break;
                            }

                            cloud_neighbor_picked[ind + l] = 1;
                        }
                    }
                }

                int smallest_picked_n = 0;
                for (int k = sp; k <= ep; k++)
                {
                    int ind = cloud_sorted_id[k];

                    if (cloud_neighbor_picked[ind] == 0 &&
                        cloud_curvature[ind] < 0.1)
                    {

                        cloud_label[ind] = -1; 
                        flat_points->push_back(laser_cloud->points[ind]);

                        smallest_picked_n++;
                        if (smallest_picked_n >= flat_points_n_each_part)
                        { 
                            break;
                        }

                        cloud_neighbor_picked[ind] = 1;
                        for (int l = 1; l <= 5; l++)
                        { 
                            float diffX = laser_cloud->points[ind + l].x - laser_cloud->points[ind + l - 1].x;
                            float diffY = laser_cloud->points[ind + l].y - laser_cloud->points[ind + l - 1].y;
                            float diffZ = laser_cloud->points[ind + l].z - laser_cloud->points[ind + l - 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                            {
                                break;
                            }

                            cloud_neighbor_picked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--)
                        {
                            float diffX = laser_cloud->points[ind + l].x - laser_cloud->points[ind + l + 1].x;
                            float diffY = laser_cloud->points[ind + l].y - laser_cloud->points[ind + l + 1].y;
                            float diffZ = laser_cloud->points[ind + l].z - laser_cloud->points[ind + l + 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                            {
                                break;
                            }
                            cloud_neighbor_picked[ind + l] = 1;
                        }
                    }
                }

                for (int k = sp; k <= ep; k++)
                {
                    if (cloud_label[k] <= 0)
                    {
                        current_less_flat_points->push_back(laser_cloud->points[k]);
                    }
                }
            }
            // less flat surf points will contains flat points and non-sharp points. 
            PointCloud current_less_flat_points_ds;
            pcl::VoxelGrid<PointType> downsize_filter;
            downsize_filter.setInputCloud(current_less_flat_points);
            downsize_filter.setLeafSize(0.2, 0.2, 0.2);
            downsize_filter.filter(current_less_flat_points_ds);

            (*less_flat_points) += current_less_flat_points_ds;
        }
        less_sharp_kdtree =
            pcl::KdTreeFLANN<PointType>::Ptr (new pcl::KdTreeFLANN<PointType>());
        less_flat_kdtree =
            pcl::KdTreeFLANN<PointType>::Ptr (new pcl::KdTreeFLANN<PointType>());    
        less_sharp_kdtree->setInputCloud(less_sharp_points);
        less_flat_kdtree->setInputCloud(less_flat_points);    

        // std::cout<<BLUE<<"[INFO]::[LoamFeature]::\nSharp points: "<<sharp_points->points.size()
        //     <<"\nLess sharp points: "<<less_sharp_points->points.size()
        //     <<"\nFlat points: "<<flat_points->points.size()
        //     <<"\nLess flat points: "<<less_flat_points->points.size()<<RESET<<std::endl;
        // estimate the normal
        if(ground_extraction)
        {
            less_ground_kdtree = 
                pcl::KdTreeFLANN<PointType>::Ptr (new pcl::KdTreeFLANN<PointType>());
            less_ground_kdtree->setInputCloud(ground_points);
            Vec3List ground_points_eigen(less_ground_points->size());
            for(size_t i = 0; i != less_ground_points->size(); ++i)
            {
                ground_points_eigen[i] = Vec3(less_ground_points->points[i].x,
                    less_ground_points->points[i].y, less_ground_points->points[i].z);
            }
            double indicator;
            std::tie(ground_plane_normal, ground_plane_dist, indicator) 
                = FitPlane(ground_points_eigen); 
            if(ground_plane_normal.dot(ground_normal) < 0)
            {
                ground_plane_normal = - ground_plane_normal;
                ground_plane_dist = - ground_plane_dist;
            }
            // std::cout<<"ground plane normal: "<<ground_plane_normal.transpose()<<std::endl;
        }
        if(!ground_removal)
        {
            *pcd = *laser_cloud;
        }
        else
        {
            // std::cout<<pcd->size()<<" ";
            pcd->points.clear();
            pcd->points.reserve(laser_cloud->size());
            for(size_t i = 0; i != laser_cloud->size(); ++i)
            {
                if(ground_point_ids.find(i) == ground_point_ids.end())
                pcd->push_back(laser_cloud->points[i]);
            }
            // std::cout<<pcd->size()<<std::endl;
        }
    }
    void Frame::CreateRangeImage()
    {
        range_image = RangeImSphPtr(new RangeImSph ());
        pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;
        Eigen::Affine3f sensor_pose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
        range_image->createFromPointCloud(*pcd, angular_resolution_x, angular_resolution_y, max_angle_width, max_angle_width, 
            sensor_pose, coordinate_frame, 0, 0, 1);
        std::cout<<BLUE<<"[INFO]::[RangeImage]::"<<*range_image<<RESET<<std::endl;
    }
}