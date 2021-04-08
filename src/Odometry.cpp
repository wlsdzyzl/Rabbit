#include "Odometry.h"
#include <pcl/filters/approximate_voxel_grid.h>

namespace rabbit
{
    void LidarOdometry::NDT(const Frame &source, const Frame &target)
    {
        // normal distribution transform
    }
    // point 2 point and point 2 plane
    void LidarOdometry::ICP(const Frame &source, const Frame &target)
    {

    }

    // feature based methods, you need to set  which feature you want to use

    // ransac 
    // we need feature correspondences firstly
    // if we use pcl sample 
    void  LidarOdometry::Ransac(const Frame &source, const Frame &target)
    {

    }
    // teaser ++
    void LidarOdometry::TeaserPP(const Frame &source, const Frame &target)
    {

    }

    // feature based. however, the feature is  based on the distance from point to line and point to plane
    // Could be difficult to apply Ransac or Teaser ++
    // We use g2o to get a reasonable estimation.
    void LidarOdometry::Loam(const Frame &source, const Frame &target)
    {

    }
}