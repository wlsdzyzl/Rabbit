#include "Odometry.h"
#include "Factors/Factors.h"
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
        double mean_dist = ndt.getFitnessScore();
        std::cout<<BLUE<<"[INFO]::[NDT]::converged: "<<is_converged<<" score: "<<mean_dist<<RESET<<std::endl;
        Mat4f T_matrix = ndt.getFinalTransformation();
        T_matrix.block<3, 3>(0, 0) = 
            Eigen::Quaternionf(T_matrix.block<3, 3>(0, 0)).normalized().toRotationMatrix();
        T = SE3f(T_matrix).cast<double>();
        if(is_converged && mean_dist < mean_dist_threshold) return true;
        return false;        
    }
    // point 2 point and point 2 plane
    bool LidarOdometry::ICP(const Frame &source, const Frame &target, SE3 &T)
    {
        icp.setInputSource(source.pcd);
        icp.setInputTarget(target.pcd);
        PointCloud final;
        icp.align(final, T.matrix().cast<float>());
        bool is_converged = icp.hasConverged();
        double mean_dist = icp.getFitnessScore();
        std::cout<<BLUE<<"[INFO]::[ICP]::converged: "<<is_converged<<" score: "<<mean_dist<<RESET<<std::endl;
        Mat4f T_matrix = icp.getFinalTransformation();
        T_matrix.block<3, 3>(0, 0) = 
            Eigen::Quaternionf(T_matrix.block<3, 3>(0, 0)).normalized().toRotationMatrix();
        T = SE3f(T_matrix).cast<double>();
        if(is_converged && mean_dist < mean_dist_threshold) return true;
        return false;
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
        double mean_dist = gicp.getFitnessScore();
        std::cout<<BLUE<<"[INFO]::[GICP]::converged: "<<is_converged<<" score: "<<mean_dist<<RESET<<std::endl;
        Mat4f T_matrix = gicp.getFinalTransformation();
        T_matrix.block<3, 3>(0, 0) = 
            Eigen::Quaternionf(T_matrix.block<3, 3>(0, 0)).normalized().toRotationMatrix();
        T = SE3f(T_matrix).cast<double>();
        if(is_converged && mean_dist < mean_dist_threshold) return true;
        return false;
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
    // from target to source
    bool LidarOdometry::Loam(const Frame &source, const Frame &target, SE3 &T)
    {
        int sharp_point_num = source.sharp_points ->points.size();
        int flat_point_num = source.flat_points->points.size();
        auto &source_sharp_points = source.sharp_points;
        auto &source_flat_points = source.flat_points;

        auto &target_less_sharp_points = target.less_sharp_points;
        auto &target_less_flat_points = target.less_flat_points;

        auto &target_less_sharp_kdtree = target.less_sharp_kdtree;
        auto &target_less_flat_kdtree = target.less_flat_kdtree;
        // optimization times
        for (size_t opti_counter = 0; opti_counter < 2; ++opti_counter)
        {
            int edge_corres_num = 0;
            int plane_corres_num = 0;

            //ceres::LossFunction *loss_function = NULL;
            ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
            ceres::Problem::Options problem_options;
            ceres::Problem problem(problem_options);
            problem.AddParameterBlock(T.data(), SE3::num_parameters, new factor::LocalParameterizationSE3);

            PointType point_sel;
            std::vector<int> point_ids;
            std::vector<float> point_sq_dists;

            for (int i = 0; i < sharp_point_num; ++i)
            {
                float s;
                if (undistorted)
                    s = (source_sharp_points->points[i].intensity - 
                        int(source_sharp_points->points[i].intensity)) / scan_period;
                else
                    s = 1.0;
                TransformToStart(&(source_sharp_points->points[i]), T, &point_sel, s);
                target_less_sharp_kdtree->nearestKSearch(point_sel, 1, point_ids, point_sq_dists);

                int closest_point_id = -1, min_point_id = -1;
                if (point_sq_dists[0] < dist_threshold)
                {
                    closest_point_id = point_ids[0];
                    int closest_point_scan_id =
                         int(target_less_sharp_points->points[closest_point_id].intensity);

                    double min_point_sq_dist = dist_threshold;
                    // search in the direction of increasing scan line
                    for (int j = closest_point_id + 1; j < (int)target_less_sharp_points->points.size(); ++j)
                    {
                        // if in the same scan line, continue
                        if (int(target_less_sharp_points->points[j].intensity) <= closest_point_scan_id)
                            continue;

                        // if not in nearby scans, end the loop
                        if (int(target_less_sharp_points->points[j].intensity) > (closest_point_scan_id + nearby_num))
                            break;

                        double point_sq_dis = (target_less_sharp_points->points[j].x - point_sel.x) *
                                                (target_less_sharp_points->points[j].x - point_sel.x) +
                                            (target_less_sharp_points->points[j].y - point_sel.y) *
                                                (target_less_sharp_points->points[j].y - point_sel.y) +
                                            (target_less_sharp_points->points[j].z - point_sel.z) *
                                                (target_less_sharp_points->points[j].z - point_sel.z);

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
                        if (int(target_less_sharp_points->points[j].intensity) >= closest_point_scan_id)
                            continue;

                        // if not in nearby scans, end the loop
                        if (int(target_less_sharp_points->points[j].intensity) < (closest_point_scan_id - nearby_num))
                            break;

                        double point_sq_dis = (target_less_sharp_points->points[j].x - point_sel.x) *
                                                (target_less_sharp_points->points[j].x - point_sel.x) +
                                            (target_less_sharp_points->points[j].y - point_sel.y) *
                                                (target_less_sharp_points->points[j].y - point_sel.y) +
                                            (target_less_sharp_points->points[j].z - point_sel.z) *
                                                (target_less_sharp_points->points[j].z - point_sel.z);

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
                    Vec3 curr_point(source_sharp_points->points[i].x,
                                                source_sharp_points->points[i].y,
                                                source_sharp_points->points[i].z);
                    Vec3 last_point_a(target_less_sharp_points->points[closest_point_id].x,
                                                    target_less_sharp_points->points[closest_point_id].y,
                                                    target_less_sharp_points->points[closest_point_id].z);
                    Vec3 last_point_b(target_less_sharp_points->points[min_point_id].x,
                                                    target_less_sharp_points->points[min_point_id].y,
                                                    target_less_sharp_points->points[min_point_id].z);
                    ceres::CostFunction *cost_function = factor::Point2LineFactor::Create(curr_point, last_point_a, last_point_b, s);
                    problem.AddResidualBlock(cost_function, loss_function, T.data());
                    edge_corres_num++;
                }
            }

            // find correspondence for plane features
            for (int i = 0; i < flat_point_num; ++i)
            {
                float s;
                if (undistorted)
                    s = (source_flat_points->points[i].intensity - 
                        int(source_flat_points->points[i].intensity)) / scan_period;
                else
                    s = 1.0;
                TransformToStart(&(source_flat_points->points[i]), T, &point_sel, s);
                target_less_flat_kdtree->nearestKSearch(point_sel, 1, point_ids, point_sq_dists);

                int closest_point_id = -1, min_point_id_1 = -1, min_point_id_2 = -1;
                if (point_sq_dists[0] < dist_threshold)
                {
                    closest_point_id = point_ids[0];

                    // get closest point's scan ID
                    int closest_point_scan_id = int(target_less_flat_points->points[closest_point_id].intensity);
                    double min_point_sq_dist_1 = dist_threshold, min_point_sq_dist_2 = dist_threshold;

                    // search in the direction of increasing scan line
                    for (int j = closest_point_id + 1; j < (int)target_less_flat_points->points.size(); ++j)
                    {
                        // if not in nearby scans, end the loop
                        if (int(target_less_flat_points->points[j].intensity) > (closest_point_scan_id + nearby_num))
                            break;

                        double point_sq_dis = (target_less_flat_points->points[j].x - point_sel.x) *
                                                (target_less_flat_points->points[j].x - point_sel.x) +
                                            (target_less_flat_points->points[j].y - point_sel.y) *
                                                (target_less_flat_points->points[j].y - point_sel.y) +
                                            (target_less_flat_points->points[j].z - point_sel.z) *
                                                (target_less_flat_points->points[j].z - point_sel.z);

                        // if in the same or lower scan line
                        if (int(target_less_flat_points->points[j].intensity) <= closest_point_scan_id && point_sq_dis < min_point_sq_dist_1)
                        {
                            min_point_sq_dist_1 = point_sq_dis;
                            min_point_id_1 = j;
                        }
                        // if in the higher scan line
                        else if (int(target_less_flat_points->points[j].intensity) > closest_point_scan_id && point_sq_dis < min_point_sq_dist_2)
                        {
                            min_point_sq_dist_2 = point_sq_dis;
                            min_point_id_2 = j;
                        }
                    }

                    // search in the direction of decreasing scan line
                    for (int j = closest_point_id - 1; j >= 0; --j)
                    {
                        // if not in nearby scans, end the loop
                        if (int(target_less_flat_points->points[j].intensity) < (closest_point_scan_id - nearby_num))
                            break;

                        double point_sq_dis = (target_less_flat_points->points[j].x - point_sel.x) *
                                                (target_less_flat_points->points[j].x - point_sel.x) +
                                            (target_less_flat_points->points[j].y - point_sel.y) *
                                                (target_less_flat_points->points[j].y - point_sel.y) +
                                            (target_less_flat_points->points[j].z - point_sel.z) *
                                                (target_less_flat_points->points[j].z - point_sel.z);

                        // if in the same or higher scan line
                        if (int(target_less_flat_points->points[j].intensity) >= closest_point_scan_id && point_sq_dis < min_point_sq_dist_1)
                        {
                            min_point_sq_dist_1 = point_sq_dis;
                            min_point_id_1 = j;
                        }
                        else if (int(target_less_flat_points->points[j].intensity) < closest_point_scan_id && point_sq_dis < min_point_sq_dist_2)
                        {
                            // find nearer point
                            min_point_sq_dist_2 = point_sq_dis;
                            min_point_id_2 = j;
                        }
                    }

                    if (min_point_id_1 >= 0 && min_point_id_2 >= 0)
                    {

                        Vec3 curr_point(source_flat_points->points[i].x,
                                                    source_flat_points->points[i].y,
                                                    source_flat_points->points[i].z);
                        Vec3 last_point_a(target_less_flat_points->points[closest_point_id].x,
                                                        target_less_flat_points->points[closest_point_id].y,
                                                        target_less_flat_points->points[closest_point_id].z);
                        Vec3 last_point_b(target_less_flat_points->points[min_point_id_1].x,
                                                        target_less_flat_points->points[min_point_id_1].y,
                                                        target_less_flat_points->points[min_point_id_1].z);
                        Vec3 last_point_c(target_less_flat_points->points[min_point_id_2].x,
                                                        target_less_flat_points->points[min_point_id_2].y,
                                                        target_less_flat_points->points[min_point_id_2].z);
                        ceres::CostFunction *cost_function = factor::Point2PlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c, s);
                        problem.AddResidualBlock(cost_function, loss_function, T.data());
                        plane_corres_num++;
                    }
                }
            }
            if ((edge_corres_num + plane_corres_num) < 10)
            {
                std::cout<<YELLOW<<"[WARNING]::[LOAM]::Less than 10 correspondences."<<RESET<<std::endl;
                // return false;
            }
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.max_num_iterations = 4;
            options.minimizer_progress_to_stdout = false;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
        }
        return true;  
    }
    // T need to be a reasonable guess
    bool LidarOdometry::LoamMapping(const Frame &source, const Frame &volume, SE3 &T)
    {
        int sharp_point_num = source.sharp_points->points.size();
        int flat_point_num = source.flat_points->points.size();
        auto &source_sharp_points = source.sharp_points;
        auto &source_flat_points = source.flat_points;

        // volume volume
        auto &volume_less_sharp_points = volume.less_sharp_points;
        auto &volume_less_flat_points = volume.less_flat_points;

        auto &volume_less_sharp_kdtree = volume.less_sharp_kdtree;
        auto &volume_less_flat_kdtree = volume.less_flat_kdtree;
        // optimization times
        int line_fitting_n = 5;
        int plane_fitting_n = 5;
        for (size_t opti_counter = 0; opti_counter < 2; ++opti_counter)
        {
            int edge_corres_num = 0;
            int plane_corres_num = 0;

            //ceres::LossFunction *loss_function = NULL;
            ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
            ceres::Problem::Options problem_options;
            ceres::Problem problem(problem_options);
            problem.AddParameterBlock(T.data(), SE3::num_parameters, new factor::LocalParameterizationSE3);

            PointType point_sel;
            std::vector<int> point_ids;
            std::vector<float> point_sq_dists;

            for (int i = 0; i < sharp_point_num; ++i)
            {
                TransformToStart(&(source_sharp_points->points[i]), T, &point_sel, 1);
                volume_less_sharp_kdtree->nearestKSearch(point_sel, line_fitting_n, point_ids, point_sq_dists);

                if (point_sq_dists[line_fitting_n - 1] < mapping_dist_threshold)
                {
                    // line fitting.
                    std::vector<Vec3> near_edges;
                    Vec3 center(0, 0, 0);
                    // get the mean of point set
                    for (int j = 0; j < line_fitting_n; j++)
                    {
                        Vec3 tmp(volume_less_sharp_points->points[point_ids[j]].x,
                                            volume_less_sharp_points->points[point_ids[j]].y,
                                            volume_less_sharp_points->points[point_ids[j]].z);
                        center = center + tmp;
                        near_edges.push_back(tmp);
                    }
                    center = center / line_fitting_n;
                    // get covariance matrix
                    Mat3 cov_mat = Mat3::Zero();
                    for (int j = 0; j < line_fitting_n; j++)
                    {
                        Vec3 tmp_zero_mean = near_edges[j] - center;
                        cov_mat = cov_mat + tmp_zero_mean * tmp_zero_mean.transpose();
                    }

                    Eigen::SelfAdjointEigenSolver<Mat3> saes(cov_mat);
                    // Eigen value decomposition
                    // if is indeed line feature
                    // note Eigen library sort eigenvalues in increasing order
                    Vec3 unit_direction = saes.eigenvectors().col(2);
                    Vec3 curr_point(source_sharp_points->points[i].x,
                                                source_sharp_points->points[i].y,
                                                source_sharp_points->points[i].z);
                    // to test if line fitting is reasonable
                    if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
                    {
                        ceres::CostFunction *cost_function = factor::Point2LineDirectionFactor::Create(curr_point, unit_direction, center);
                        problem.AddResidualBlock(cost_function, loss_function, T.data());
                        edge_corres_num++;
                    }				
                }
            }

            // find correspondence for plane features
            for (int i = 0; i < flat_point_num; ++i)
            {
                float s;
                if (undistorted)
                    s = (source_flat_points->points[i].intensity - 
                        int(source_flat_points->points[i].intensity)) / scan_period;
                else
                    s = 1.0;
                TransformToStart(&(source_flat_points->points[i]), T, &point_sel, 1);
                volume_less_flat_kdtree->nearestKSearch(point_sel, plane_fitting_n, point_ids, point_sq_dists);

                
                if (point_sq_dists[plane_fitting_n - 1] < dist_threshold)
                {
                    
                    Eigen::MatrixXd mat_A = Eigen::MatrixXd::Zero(plane_fitting_n, 3);
                    Eigen::MatrixXd mat_B = -1 * Eigen::MatrixXd::Ones(plane_fitting_n, 1);

                    for (int j = 0; j < plane_fitting_n; j++)
                    {
                        mat_A(j, 0) = volume_less_flat_points->points[point_ids[j]].x;
                        mat_A(j, 1) = volume_less_flat_points->points[point_ids[j]].y;
                        mat_A(j, 2) = volume_less_flat_points->points[point_ids[j]].z;
                        //printf(" pts %f %f %f ", mat_A(j, 0), mat_A(j, 1), mat_A(j, 2));
                    }
                    // find the norm of plane
                    // solve Ax + By + Cz = -1, and normalize the normal vector
                    // get equation: nx + d = 0
                    Vec3 normal = mat_A.colPivHouseholderQr().solve(mat_B);
                    double dis = 1 / normal.norm();
                    normal.normalize();
                    
                    bool plane_valid = true;
                    for (int j = 0; j < line_fitting_n; j++)
                    {
                        if (fabs(normal(0) * volume_less_flat_points->points[point_ids[j]].x +
                                normal(1) * volume_less_flat_points->points[point_ids[j]].y +
                                normal(2) * volume_less_flat_points->points[point_ids[j]].z + dis) > 0.2)
                        {
                            plane_valid = false;
                            break;
                        }
                    }
                    Vec3 curr_point(source_flat_points->points[i].x,
                                                source_flat_points->points[i].y,
                                                source_flat_points->points[i].z);
                    if (plane_valid)
                    {
                        ceres::CostFunction *cost_function = factor::Point2PlaneEquationFactor::Create(curr_point, normal, dis);
                        problem.AddResidualBlock(cost_function, loss_function, T.data());
                        plane_corres_num++;
                    }
                }
            }
            if ((edge_corres_num + plane_corres_num) < 10)
            {
                std::cout<<YELLOW<<"[WARNING]::[LOAM]::Less than 10 correspondences."<<RESET<<std::endl;
                // return false;
            }
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.max_num_iterations = 4;
            options.minimizer_progress_to_stdout = false;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
        }  
        return true;
    }
}