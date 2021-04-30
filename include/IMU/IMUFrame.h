#ifndef RABBIT_IMUFrame_H
#define RABBIT_IMUFrame_H
#include <sensor_msgs/Imu.h>
#include "Utils/Util.h"
#define IMU_FREQUENCY 500
#define GRAVITY 9.8
namespace rabbit
{
    namespace imu
    {
        struct IMUFrame
        {
            Vec3 acceleration;
            Vec3 angular_velocity;
            Mat3 orientation;
            double timestamp;
            void LoadFromMessage(const sensor_msgs::Imu& imu_in);
            // associated with the lidar frame
            int imu_id;
        };
    }
}
#endif