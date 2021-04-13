#ifndef RABBIT_LIDAR_ODOMETRY_H
#define RABBIT_LIDAR_ODOMETRY_J
#include "Util.h"
#include "Frame.h"
#include <pcl/registration/ndt.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>

namespace rabbit
{
    
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

            //Ransac parameter
            align.setMaximumIterations (50000); // Number of RANSAC iterations
            align.setNumberOfSamples (5); // Number of points to sample for generating/prerejecting a pose
            align.setCorrespondenceRandomness (5); // Number of nearest features to use
            align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
            align.setMaxCorrespondenceDistance (2.5f * leaf_size); // Inlier threshold
            align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
            //Teaser parameter

        }
        // normal distribution
        bool NDT(const Frame &source, const Frame &target, SE3 &T);
        // point 2 point and point 2 plane
        bool ICP(const Frame &source, const Frame &target, SE3 &T);
        bool GICP(const Frame &source, const Frame &target, SE3 &T);
        bool ICPN(const Frame &source, const Frame &target, SE3 &T);
        // feature based methods, you need to set  which feature you want to use

        // ransac 
        bool Ransac(const Frame &source, const Frame &target, SE3 &T);
        // teaser ++
        bool TeaserPP(const Frame &source, const Frame &target, SE3 &T);

        // feature based. however, the feature is  based on the distance from point to line and point to plane
        // It can be a litter difficult to apply ransac on such correspondence (of couse we can, but it may need extra work)
        // We use g2o to get a reasonable estimation.
        bool Loam(const Frame &source, const Frame &target, SE3 &T);

        protected:
        pcl::NormalDistributionsTransform<PointType, PointType> ndt;
        pcl::SampleConsensusPrerejective<PointType, PointType,FPFH> align;
        pcl::IterativeClosestPoint<PointType, PointType> icp;
        pcl::IterativeClosestPointWithNormals<PointTypeN, PointTypeN> icpn;
        pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
        float leaf_size = 0.2;
        float scan_period = 0.1;
        float dist_threshold = 25;
        bool undistorted = false;
        float nearby_num = 2.5; 

    };
    // ndt (voxel based method)
}
#endif