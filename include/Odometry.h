#ifndef RABBIT_LIDAR_ODOMETRY_H
#define RABBIT_LIDAR_ODOMETRY_H
#include "Utils/Utils.h"
#include "Frame.h"
#include "Utils/KDTree.h"
#include <pcl/registration/ndt.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include "pclomp/ndt_omp.h"
#include "pclomp/gicp_omp.h"
namespace rabbit
{
    enum OdometryMethod
    {
        NDT,
        ICP,
        GICP,
        LOAM,
        NDTOMP,
        GICPOMP
    };
    class LidarOdometry
    {
        public: 
        LidarOdometry()
        {
            // NDT parameter
            ndt.setTransformationEpsilon (0.001);
            // Setting maximum step size for More-Thuente line search.
            ndt.setStepSize (0.1);
            //Setting Resolution of NDT grid structure (VoxelGridCovariance).
            ndt.setResolution (1.0);
            ndt.setMaximumIterations (35);           

            ndtomp.setTransformationEpsilon (0.001);
            // Setting maximum step size for More-Thuente line search.
            ndtomp.setStepSize (0.1);
            //Setting Resolution of ndtomp grid structure (VoxelGridCovariance).
            ndtomp.setResolution (1.0);
            ndtomp.setMaximumIterations (35); 
            ndtomp.setNeighborhoodSearchMethod(pclomp::KDTREE);
            ndtomp.setNumThreads(omp_get_max_threads());
            //ICP parameter
            icp.setMaximumIterations(25);
            icp.setMaxCorrespondenceDistance(0.75);
            icp.setTransformationEpsilon(0.0001);
            icp.setEuclideanFitnessEpsilon(0.0001);

            icpn.setMaximumIterations(25);
            icpn.setMaxCorrespondenceDistance(0.75);
            icpn.setTransformationEpsilon(0.0001);
            icpn.setEuclideanFitnessEpsilon(0.0001);

            gicp.setMaximumIterations(25);
            gicp.setMaxCorrespondenceDistance(0.75);
            gicp.setTransformationEpsilon(0.0001);
            gicp.setEuclideanFitnessEpsilon(0.0001);

            gicpomp.setMaximumIterations(25);
            gicpomp.setMaxCorrespondenceDistance(0.75);
            gicpomp.setTransformationEpsilon(0.0001);
            gicpomp.setEuclideanFitnessEpsilon(0.0001);

            //Ransac parameter
            align.setMaximumIterations (50000); // Number of RANSAC iterations
            align.setNumberOfSamples (5); // Number of points to sample for generating/prerejecting a pose
            align.setCorrespondenceRandomness (5); // Number of nearest features to use
            align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
            align.setMaxCorrespondenceDistance (2.5f * leaf_size); // Inlier threshold
            align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
            //Teaser parameter

        }
        inline std::pair<double, double> GetFitnessScore(const Frame &source, const Frame &target, 
            const SE3 &T, double max_range = 1e7)
        {
            Vec3List target_pcd, source_pcd;
            ToEigenPoints(*(target.pcd), target_pcd);
            ToEigenPoints(*(source.pcd), source_pcd);
            KDTree<3> kdtree;
            kdtree.BuildTree(target_pcd);
            auto source_size = source_pcd.size();
            double overall_dist = 0;
            int n = 0;
            for(size_t i = 0; i != source_size; ++i)
            {
                Vec3 trans_p = T * source_pcd[i];
                std::vector<int> indices;
                std::vector<float> dists;
                kdtree.KnnSearch(trans_p, indices, dists, 1);
                if(dists.size() >= 1 && dists[0] < max_range)
                {
                    overall_dist += dists[0];
                    ++n;
                }
            }
            if(n > 0) return std::make_pair(overall_dist / n, (n + 0.0) / source_size);
            return std::make_pair(max_range, 0);
        }
        inline void SetNDTOMPSearchMethod(pclomp::NeighborSearchMethod sm_)
        {
            ndtomp.setNeighborhoodSearchMethod(sm_);
        }
        inline void SetThreadNum(int n)
        {
            if(n <= omp_get_max_threads())
            ndtomp.setNumThreads(n);
            else 
            ndtomp.setNumThreads(omp_get_max_threads());
        }
        inline void SetGroundPlane(const Vec3 &n, double d, double w = 10)
        {
            ground_normal = n;
            ground_dist = d;
            ground_weight = w;
        }
        // normal distribution
        std::pair<double, double> NDT(const Frame &source, const Frame &target, SE3 &T, double inlier_threshold = 1e7);
        std::pair<double, double> NDTOMP(const Frame &source, const Frame &target, SE3 &T, double inlier_threshold = 1e7);
        // point 2 point and point 2 plane
        std::pair<double, double> ICP(const Frame &source, const Frame &target, SE3 &T, double inlier_threshold = 1e7);
        std::pair<double, double> ICPN(const Frame &source, const Frame &target, SE3 &T, double inlier_threshold = 1e7);
        std::pair<double, double> GICP(const Frame &source, const Frame &target, SE3 &T, double inlier_threshold = 1e7);
        std::pair<double, double> GICPOMP(const Frame &source, const Frame &target, SE3 &T, double inlier_threshold = 1e7);

        // feature based methods, you need to set  which feature you want to use

        // ransac 
        std::pair<double, double> Ransac(const Frame &source, const Frame &target, SE3 &T, double inlier_threshold = 1e7);
        // teaser ++
        std::pair<double, double> TeaserPP(const Frame &source, const Frame &target, SE3 &T, double inlier_threshold = 1e7);

        // feature based. however, the feature is  based on the distance from point to line and point to plane
        // line (2 points), plane (3 points)
        std::pair<double, double> Loam(const Frame &source, const Frame &target, SE3 &T, double inlier_threshold = 1e7);
        

        // feature based. similar to loam mapping, here we still use distance from point to line and point to plane
        // however, in mapping case, source will have a larger volume, therefore
        // we use multiple points to fitting a line and plane.
        std::pair<double, double> LoamMapping(const Frame &source, const Frame &volume, SE3 &T, double inlier_threshold = 1e7);
        std::pair<double, double> LoamGroundMapping(const Frame &source, const Frame &volume, SE3 &T, double inlier_threshold = 1e7);
        protected:
        pcl::NormalDistributionsTransform<PointType, PointType> ndt;
        pclomp::NormalDistributionsTransform<PointType, PointType> ndtomp;
        pcl::SampleConsensusPrerejective<PointType, PointType,FPFH> align;
        pcl::IterativeClosestPoint<PointType, PointType> icp;
        pcl::IterativeClosestPointWithNormals<PointTypeN, PointTypeN> icpn;
        pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
        pclomp::GeneralizedIterativeClosestPoint<PointType, PointType> gicpomp;
        float leaf_size = 0.2;
        float scan_period = 0.1;
        float dist_threshold = 25;
        float mapping_dist_threshold = 1.0;
        bool undistorted = false;
        float nearby_num = 2.5; 
        Vec3 ground_normal;
        double ground_weight = 5;
        double ground_dist;

    };
    // ndt (voxel based method)
}
#endif