#ifndef  RABBIT_PREINTEGRATION_H
#define RABBIT_PREINTEGRATION_H
#include "Utils/Utils.h"
#include "IMU/IMUFrame.h"

namespace rabbit
{
namespace imu
{
    using namespace util;
    class IMUIntegrator
    {
        public:
        IMUIntegrator() = default;
        void Preintegrate(const Vec3 &gyr, const Vec3 &acc,  double delta_t);
        void Preintegrate(const std::vector<IMUFrame> &imu_frame_list,  int start, int end)
        {
            Reset();
            
            for(int i = start; i != end; ++i)
            {
                double delta_t =  1.0 / IMUFrame::imu_frequency;
                if(end < imu_frame_list.size())
                delta_t = imu_frame_list[i+1].timestamp - imu_frame_list[i].timestamp;
                if(delta_t <= 0)
                {
                    std::cout<<YELLOW<<"[WARNING]::[IMUPreintegration]::Delta_t <= 0. So the delta will be reset to "<<1.0 / IMUFrame::imu_frequency<<"."<<RESET<<std::endl;
                    delta_t =  1.0 / IMUFrame::imu_frequency;
                }
                Preintegrate(imu_frame_list[i].angular_velocity, imu_frame_list[i].acceleration,delta_t);
            }
        }
        void Integrate(const Vec3 &gyr, const Vec3 &acc,  double delta_t)
        {
             //euler integration
            
            //  std::cout<<"delta_t: "<<delta_t<<" "
            //     <<(delta_v).transpose()<<std::endl;
            std::cout<<"dt: "<<(acc - ba).transpose()<<" "<<(gyr - bg).transpose()<<std::endl;
            delta_p += delta_v * delta_t + 0.5 * delta_t * delta_t * IMUFrame::gravity  +  0.5 * delta_t * delta_t * delta_R * (acc - ba );
            delta_v += delta_t * delta_R * (acc - ba) + delta_t * IMUFrame::gravity;
            delta_R *= SO3::exp((gyr - bg) * delta_t).matrix();           
        }
        void UpdateBias(const Vec3 &delta_bg, const Vec3 &delta_ba);
        void SetInitialBias(const Vec3 &_bg, const Vec3 &_ba)
        {
            bg = _bg;
            ba = _ba;
        }
        void SetNoiseCov(const Mat3 &gyro_noise_cov, const Mat3 &acc_noise_cov, const Mat3 &integration_cov_ = Mat3::Zero())
        {
            white_noise_cov_g = gyro_noise_cov;
            white_noise_cov_a = acc_noise_cov;
            integration_cov = integration_cov_;
        }
        // Mat9 ComputeJacobian();
        void Reset()
        {
            delta_R = Mat3::Identity();
            delta_v.setZero();
            delta_p.setZero();
            covariance.setZero();
            white_noise_cov_g.setZero();
            white_noise_cov_a.setZero();
            integration_cov.setZero();

            bg.setZero();
            ba.setZero();
        }
        // rotation
        Mat3 delta_R = Mat3::Identity();
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
    };
}
}
#endif