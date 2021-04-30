#ifndef  RABBIT_PREINTEGRATION_H
#define RABBIT_PREINTEGRATION_H
#include "Utils/Util.h"
#include "Utils/MathUtil.h"
#include "IMU/IMUFrame.h"

namespace rabbit
{
namespace imu
{
    class IMUPreintegrator
    {
        using namespace utils;
        public:
        IMUPreintegrator() = default;
        Preintegrate(const Vec3 &gyr, const Vec3 &acc,  double delta_t);
        Preintegrate(const std::vector<IMUFrame> &imu_frame_list,  int start, int end)
        {
            Reset();
            
            for(int i = start; i != end; ++i)
            {
                double delta_t =  1.0 / IMU_FREQUENCY;
                if(end < imu_frame_list.size())
                delta_t = imu_frame_list[i+1].timestamp - imu_frame_list[i].timestamp;
                if(delta_t <= 0)
                {
                    std::cout<<YELLOW<<"[WARNING]::[IMUPreintegration]::Delta_t <= 0. So the delta will be reset to "<<1.0 / IMU_FREQUENCY<<"."<<RESET<<std::endl;
                    delta_t =  1.0 / IMU_FREQUENCY;
                }
                Preintegrate(imu_frame_list[i].angular_velocity, imu_frame_list[i].acceleration,delta_t);
            }
        }
        void UpdateBias(const Vec3 &delta_ba, const Vec3 &delta_bg);
        void SetInitialBias();
        void SetNoiseCov(const Mat3 &gyro_noise_cov, const Mat3 &acc_noise_cov, const Mat3 &integration_cov_ = Mat3::Zero())
        {
            white_noise_cov_g = gyro_noise_cov;
            white_noise_cov_a = acc_noise_cov;
            integration_cov = integration_cov_;
        }
        // Mat9 ComputeJacobian();
        void Reset()
        {
            delta_R = SO3();
            delta_v.setZero();
            delta_p.setZero();
            covariance.setZero();
            white_noise_cov_g.setZero();
            white_noise_cov_a.setZero();
            integration_cov.setZero();
            source_id = -1;
            target_id = -1;
            bg.setZero();
            ba.setZero();
        }
        // rotation
        SO3 delta_R = SO3();
        // velocity
        Vec3 delta_v = Vec3::Zero();
        // position
        Vec3 delta_p = Vec3::Zero();

        Vec3 ba = Vec3::Zero();
        Vec3 bg =  Vec3::Zero();
        // vins-mono also integrates the bias.
        // Vec3 result_linearized_ba, result_linearized_bg;
        // covariance, 9 * 9
        Mat9 covariance = Mat9::Zero();
        Mat3 white_noise_cov_g = Mat3::Zero();
        Mat3 white_noise_cov_a = Mat3::Zero();
        Mat3 integration_cov = Mat3::Zero();
        Mat3 J_R_bg;
        Mat3 J_v_bg;
        Mat3 J_v_ba;
        Mat3 J_p_bg;
        Mat3 J_p_ba;
        bool compute_covariance = true;
        bool compute_jacobian = true;
        // we also need to consider the noise model.
        int source_id = -1;
        int target_id = -1;
    };
}
}
#endif