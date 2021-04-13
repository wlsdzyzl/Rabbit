#include "Odometry.h"
#include "Residual.hpp"
namespace rabbit
{
    bool LidarOdometry::NDT(const Frame &source, const Frame &target, SE3 &T)
    {
        // normal distribution transform
        ndt.setInputSource(source.pcd);
        ndt.setInputTarget(target.pcd);
        PointCloud final;
        ndt.align(final, T.matrix().cast<float>());
        bool is_converged = ndt.hasConverged();
        std::cout<<BLUE<<"[INFO]::[ICP]::converged: "<<is_converged<<" score: "<<ndt.getFitnessScore()<<RESET<<std::endl;
        T =  SE3f( ndt.getFinalTransformation()).cast<double>();
        return is_converged;        
    }
    // point 2 point and point 2 plane
    bool LidarOdometry::ICP(const Frame &source, const Frame &target, SE3 &T)
    {
        icp.setInputSource(source.pcd);
        icp.setInputTarget(target.pcd);
        PointCloud final;
        icp.align(final, T.matrix().cast<float>());
        bool is_converged = icp.hasConverged();
        std::cout<<BLUE<<"[INFO]::[ICP]::converged: "<<is_converged<<" score: "<<icp.getFitnessScore()<<RESET<<std::endl;
        // std::cout<<<<std::endl;
        // Mat4 trans= icp.getFinalTransformation().cast<double>();
        T =  SE3f( icp.getFinalTransformation()).cast<double>();
        // T = SE3(trans.block<3, 3>(0, 0), trans.block<3, 1>(0, 3));
        std::cout<<"is_converged: "<<is_converged<<std::endl;
        return is_converged;
    }

    bool LidarOdometry::ICPN(const Frame &source, const Frame &target, SE3 &T)
    {
        // icpn.setInputSource(source.pcd);
        // icpn.setInputTarget(target.pcd);
        // PointCloud final;
        // icpn.align(final);
        // bool is_converged = icpn.hasConverged();
        // std::cout<<BLUE<<"[INFO]::[ICP]::converged: "<<is_converged<<" score: "<<icpn.getFitnessScore()<<RESET<<std::endl;
        // T = icpn.getFinalTransformation();
        // return is_converged;
    }

    bool LidarOdometry::GICP(const Frame &source, const Frame &target, SE3 &T)
    {
        gicp.setInputSource(source.pcd);
        gicp.setInputTarget(target.pcd);
        PointCloud final;
        gicp.align(final, T.matrix().cast<float>());
        bool is_converged = gicp.hasConverged();
        std::cout<<BLUE<<"[INFO]::[ICP]::converged: "<<is_converged<<" score: "<<gicp.getFitnessScore()<<RESET<<std::endl;
        T =  SE3f( gicp.getFinalTransformation()).cast<double>();
        return is_converged;
    }

    // feature based methods, you need to set  which feature you want to use

    // ransac 
    // we need feature correspondences firstly
    // if we use pcl sample 
    bool  LidarOdometry::Ransac(const Frame &source, const Frame &target, SE3 &T)
    {

    }
    // teaser ++
    bool LidarOdometry::TeaserPP(const Frame &source, const Frame &target, SE3 &T)
    {

    }

    // feature based. however, the feature is  based on the distance from point to line and point to plane
    // Could be difficult to apply Ransac or Teaser ++
    // We use g2o to get a reasonable estimation.
    // undistort lidar point
    void TransformToStart(PointType const *const pi, const SE3 &T, PointType *const po, 
        float s)
    {
        //interpolation ratio
        //s = 1;
        SE3 proj2start= 
            Sophus::interpolate(SE3(), T, s);
        // std::cout<<proj2start.matrix()<<std::endl;
        // std::cout<<T.matrix()<<std::endl;
        Vec3 point(pi->x, pi->y, pi->z);
        Vec3 un_point = proj2start * point;

        po->x = un_point.x();
        po->y = un_point.y();
        po->z = un_point.z();
        po->intensity = pi->intensity;
    }

    bool LidarOdometry::Loam(const Frame &source, const Frame &target, SE3 &T)
    {
        int sharp_point_num = target.sharp_points ->points.size();
        int flat_point_num = target.flat_points->points.size();
        auto &target_sharp_points = target.sharp_points;
        auto &target_flat_points = target.flat_points;

        auto &source_less_sharp_points = source.less_sharp_points;
        auto &source_less_flat_points = source.less_flat_points;

        auto &source_less_sharp_kdtree = source.less_sharp_kdtree;
        auto &source_less_flat_kdtree = source.less_flat_kdtree;
        // optimization times
        for (size_t opti_counter = 0; opti_counter < 2; ++opti_counter)
        {
            int edge_corres_num = 0;
            int plane_corres_num = 0;

            //ceres::LossFunction *loss_function = NULL;
            ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
            ceres::Problem::Options problem_options;
            ceres::Problem problem(problem_options);
            problem.AddParameterBlock(T.data(), SE3::num_parameters, new LocalParameterizationSE3);

            PointType point_sel;
            std::vector<int> point_ids;
            std::vector<float> point_sq_dists;

            for (int i = 0; i < sharp_point_num; ++i)
            {
                float s;
                if (undistorted)
                    s = (target_sharp_points->points[i].intensity - 
                        int(target_sharp_points->points[i].intensity)) / scan_period;
                else
                    s = 1.0;
                TransformToStart(&(target_sharp_points->points[i]), T, &point_sel, s);
                source_less_sharp_kdtree->nearestKSearch(point_sel, 1, point_ids, point_sq_dists);

                int closest_point_id = -1, min_point_id = -1;
                if (point_sq_dists[0] < dist_threshold)
                {
                    closest_point_id = point_ids[0];
                    int closest_point_scan_id =
                         int(source_less_sharp_points->points[closest_point_id].intensity);

                    double min_point_sq_dist = dist_threshold;
                    // search in the direction of increasing scan line
                    for (int j = closest_point_id + 1; j < (int)source_less_sharp_points->points.size(); ++j)
                    {
                        // if in the same scan line, continue
                        if (int(source_less_sharp_points->points[j].intensity) <= closest_point_scan_id)
                            continue;

                        // if not in nearby scans, end the loop
                        if (int(source_less_sharp_points->points[j].intensity) > (closest_point_scan_id + nearby_num))
                            break;

                        double point_sq_dis = (source_less_sharp_points->points[j].x - point_sel.x) *
                                                (source_less_sharp_points->points[j].x - point_sel.x) +
                                            (source_less_sharp_points->points[j].y - point_sel.y) *
                                                (source_less_sharp_points->points[j].y - point_sel.y) +
                                            (source_less_sharp_points->points[j].z - point_sel.z) *
                                                (source_less_sharp_points->points[j].z - point_sel.z);

                        if (point_sq_dis < min_point_sq_dist)
                        {
                            // find nearer point
                            min_point_sq_dist = point_sq_dis;
                            min_point_id = j;
                        }
                    }

                    // search in the direction of decreasing scan line
                    for (int j = closest_point_id - 1; j >= 0; --j)
                    {
                        // if in the same scan line, continue
                        if (int(source_less_sharp_points->points[j].intensity) >= closest_point_scan_id)
                            continue;

                        // if not in nearby scans, end the loop
                        if (int(source_less_sharp_points->points[j].intensity) < (closest_point_scan_id - nearby_num))
                            break;

                        double point_sq_dis = (source_less_sharp_points->points[j].x - point_sel.x) *
                                                (source_less_sharp_points->points[j].x - point_sel.x) +
                                            (source_less_sharp_points->points[j].y - point_sel.y) *
                                                (source_less_sharp_points->points[j].y - point_sel.y) +
                                            (source_less_sharp_points->points[j].z - point_sel.z) *
                                                (source_less_sharp_points->points[j].z - point_sel.z);

                        if (point_sq_dis < min_point_sq_dist)
                        {
                            // find nearer point
                            min_point_sq_dist = point_sq_dis;
                            min_point_id = j;
                        }
                    }
                }
                if (min_point_id >= 0) // both closest_point_id and min_point_id is valid
                {
                    Vec3 curr_point(target_sharp_points->points[i].x,
                                                target_sharp_points->points[i].y,
                                                target_sharp_points->points[i].z);
                    Vec3 last_point_a(source_less_sharp_points->points[closest_point_id].x,
                                                    source_less_sharp_points->points[closest_point_id].y,
                                                    source_less_sharp_points->points[closest_point_id].z);
                    Vec3 last_point_b(source_less_sharp_points->points[min_point_id].x,
                                                    source_less_sharp_points->points[min_point_id].y,
                                                    source_less_sharp_points->points[min_point_id].z);
                    ceres::CostFunction *cost_function = Point2LineFactor::Create(curr_point, last_point_a, last_point_b, s);
                    problem.AddResidualBlock(cost_function, loss_function, T.data());
                    edge_corres_num++;
                }
            }

            // find correspondence for plane features
            for (int i = 0; i < flat_point_num; ++i)
            {
                float s;
                if (undistorted)
                    s = (target_flat_points->points[i].intensity - 
                        int(target_flat_points->points[i].intensity)) / scan_period;
                else
                    s = 1.0;
                TransformToStart(&(target_flat_points->points[i]), T, &point_sel, s);
                source_less_flat_kdtree->nearestKSearch(point_sel, 1, point_ids, point_sq_dists);

                int closest_point_id = -1, min_point_id_1 = -1, min_point_id_2 = -1;
                if (point_sq_dists[0] < dist_threshold)
                {
                    closest_point_id = point_ids[0];

                    // get closest point's scan ID
                    int closest_point_scan_id = int(source_less_flat_points->points[closest_point_id].intensity);
                    double min_point_sq_dist_1 = dist_threshold, min_point_sq_dist_2 = dist_threshold;

                    // search in the direction of increasing scan line
                    for (int j = closest_point_id + 1; j < (int)source_less_flat_points->points.size(); ++j)
                    {
                        // if not in nearby scans, end the loop
                        if (int(source_less_flat_points->points[j].intensity) > (closest_point_scan_id + nearby_num))
                            break;

                        double point_sq_dis = (source_less_flat_points->points[j].x - point_sel.x) *
                                                (source_less_flat_points->points[j].x - point_sel.x) +
                                            (source_less_flat_points->points[j].y - point_sel.y) *
                                                (source_less_flat_points->points[j].y - point_sel.y) +
                                            (source_less_flat_points->points[j].z - point_sel.z) *
                                                (source_less_flat_points->points[j].z - point_sel.z);

                        // if in the same or lower scan line
                        if (int(source_less_flat_points->points[j].intensity) <= closest_point_scan_id && point_sq_dis < min_point_sq_dist_1)
                        {
                            min_point_sq_dist_1 = point_sq_dis;
                            min_point_id_1 = j;
                        }
                        // if in the higher scan line
                        else if (int(source_less_flat_points->points[j].intensity) > closest_point_scan_id && point_sq_dis < min_point_sq_dist_2)
                        {
                            min_point_sq_dist_2 = point_sq_dis;
                            min_point_id_2 = j;
                        }
                    }

                    // search in the direction of decreasing scan line
                    for (int j = closest_point_id - 1; j >= 0; --j)
                    {
                        // if not in nearby scans, end the loop
                        if (int(source_less_flat_points->points[j].intensity) < (closest_point_scan_id - nearby_num))
                            break;

                        double point_sq_dis = (source_less_flat_points->points[j].x - point_sel.x) *
                                                (source_less_flat_points->points[j].x - point_sel.x) +
                                            (source_less_flat_points->points[j].y - point_sel.y) *
                                                (source_less_flat_points->points[j].y - point_sel.y) +
                                            (source_less_flat_points->points[j].z - point_sel.z) *
                                                (source_less_flat_points->points[j].z - point_sel.z);

                        // if in the same or higher scan line
                        if (int(source_less_flat_points->points[j].intensity) >= closest_point_scan_id && point_sq_dis < min_point_sq_dist_1)
                        {
                            min_point_sq_dist_1 = point_sq_dis;
                            min_point_id_1 = j;
                        }
                        else if (int(source_less_flat_points->points[j].intensity) < closest_point_scan_id && point_sq_dis < min_point_sq_dist_2)
                        {
                            // find nearer point
                            min_point_sq_dist_2 = point_sq_dis;
                            min_point_id_2 = j;
                        }
                    }

                    if (min_point_id_1 >= 0 && min_point_id_2 >= 0)
                    {

                        Vec3 curr_point(target_flat_points->points[i].x,
                                                    target_flat_points->points[i].y,
                                                    target_flat_points->points[i].z);
                        Vec3 last_point_a(source_less_flat_points->points[closest_point_id].x,
                                                        source_less_flat_points->points[closest_point_id].y,
                                                        source_less_flat_points->points[closest_point_id].z);
                        Vec3 last_point_b(source_less_flat_points->points[min_point_id_1].x,
                                                        source_less_flat_points->points[min_point_id_1].y,
                                                        source_less_flat_points->points[min_point_id_1].z);
                        Vec3 last_point_c(source_less_flat_points->points[min_point_id_2].x,
                                                        source_less_flat_points->points[min_point_id_2].y,
                                                        source_less_flat_points->points[min_point_id_2].z);
                        ceres::CostFunction *cost_function = Point2PlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c, s);
                        problem.AddResidualBlock(cost_function, loss_function, T.data());
                        plane_corres_num++;
                    }
                }
            }
            if ((edge_corres_num + plane_corres_num) < 10)
            {
                std::cout<<YELLOW<<"[WARNING]::[LOAM]::Less than 10 correspondences."<<RESET<<std::endl;
            }
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.max_num_iterations = 4;
            options.minimizer_progress_to_stdout = false;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
        }  
        T = T.inverse();
    }
}