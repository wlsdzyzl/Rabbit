#ifndef RABBIT_IMUFrame_H
#define RABBIT_IMUFrame_H
#include <sensor_msgs/Imu.h>
#include "Utils/Utils.h"
namespace rabbit
{
namespace imu
{
    using namespace util;

    struct IMUFrame
    {
        Vec3 acceleration;
        Vec3 angular_velocity;
        Mat3 rotation;
        double timestamp;
        IMUFrame() = default;
        int imu_id = -1;
        // 9-axis imu will provide orientation
        IMUFrame(const sensor_msgs::Imu &imu_in, const Mat3 &extrinsic_rot = Mat3::Identity(), const Mat3 &extrinsic_rpy = Mat3::Identity())
        {
            // rotate acceleration
            acceleration = extrinsic_rot * Vec3(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
            // rotate gyroscope
            angular_velocity = extrinsic_rot *   Vec3(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
            // rotate orientation
            Eigen::Quaterniond q(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
            rotation =  q.toRotationMatrix() * extrinsic_rpy;
            timestamp = MsgTime(imu_in);
        }
        static double imu_frequency;
        static Vec3 gravity;
    };
}
}
#endif