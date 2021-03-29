#ifndef RABBIT_LIDAR_ODOMETRY_H
#define RABBIT_LIDAR_ODOMETRY_J
namespace rabbit
{
namespace odometry
{
    // feature based method (fpfh, range map matching)
    // use ransac or teaser ++
    class LidarOdometry
    {
        public: 
        void Publish();
    };
    // ndt (voxel based method)
}
}
#endif