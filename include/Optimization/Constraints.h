#ifndef RABBIT_CONSTRAINTS_H
#define RABBIT_CONSTRAINTS_H
#include "IMU/Integration.h"
namespace rabbit
{
namespace optimization
{

    struct FrameCorrespondence
    {
        FrameCorrespondence() = default;
        FrameCorrespondence(int nid = -1, int oid = -1, util::SE3 rp = util::SE3(), bool is_v = false)
        {
            new_id = nid;
            old_id = oid;
            relative_pose = rp;
            is_valid = is_v;
        }
        int new_id = -1;
        int old_id = -1;
        // from  new frame to old frame
        util::SE3 relative_pose; 
        bool is_valid = false;
    };
    struct IMUConstraint
    {
        int old_frame_id = -1;
        int old_imu_id = -1;
        int new_frame_id = -1;
        int new_imu_id = -1;
        imu::IMUIntegrator imu_preintegrator;
        bool is_valid = false;
    };
}
}
#endif