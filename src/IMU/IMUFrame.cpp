#include "IMU/IMUFrame.h"

namespace rabbit
{
namespace imu
{
    Vec3 IMUFrame::gravity = Vec3(0, 0, -9.8);
    double IMUFrame::IMUFrame::imu_frequency = 500.0;
}
}