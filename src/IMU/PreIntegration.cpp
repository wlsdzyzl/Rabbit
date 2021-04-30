#include "IMU/PreIntegration.h"

namespace  rabbit
{
namespace imu
{
    // based on euler integration
    IMUPreintegrator::Preintegrate(const Vec3 &acc, const Vec3 &gyr,  double delta_t)
    {
        double squared_delta_t = delta_t * delta_t;
        // rotation from current state to next state
        SO3 R_c2n = SO3::exp((gyr - bg) * delta_t);
        Mat3 J_r  = RightJacobianSO3((gyr - bg) * delta_t);
        Mat3 skew_f_b = SkewSymmetric(acc - ba);
        // noise and covariance propagation
        Mat3 tmp_R_skew = delta_R *  skew_f_b;
        if(compute_covariance)
        {
            Mat9 A;
            Mat96 B;
            A.setIdentity();
            B.setZero();
            
            A.block<3, 3>(0, 0) = R_c2n.inverse().matrix();
            A.block<3, 3>(3, 0) = - delta_t * tmp ;
            A.block<3, 3>(6, 0) = - 0.5 * squared_delta_t* tmp;
            A.block<3, 3>(6, 3) = delta_t * Mat3::Identity();

            B.block<3, 3>(0, 0) = J_r.inverse() * delta_t;
            B.block<3, 3>(3, 3) = delta_t * delta_R;
            B.block<3, 3>(6, 3) = 0.5 * squared_delta_t* delta_R;

            // update noise and covariance
            Mat6 white_noise_cov = Mat6::Zero();
            // original white noise
            // why the noise need to plus inv_dt ?
            white_noise_cov.block<3, 3>(0, 0) = white_noise_g_cov * inv_dt;
            white_noise_cov.block<3, 3>(3, 3) = white_noise_a_cov * inv_dt;
            double inv_dt = 1 / std::max(dt, 1e-7);
            covariance = A * covariance * A.transpose() + B * white_noise_cov * B.transpose(); 
            covariance.block<3, 3>(3, 3) += integration_cov * dt;
        }
        if(compute_jacobian)
        {
            J_p_bg += J_v_bg * delta_t - 0.5 * squared_delta_t* tmp_R_skew * J_R_bg;
            J_p_ba += J_v_ba * delta_t - 0.5 * squared_delta_t* delta_R; 
            J_v_bg -= tmp_R_skew * J_R_bg * delta_t ;
            J_v_ba -= delta_R * delta_t;
            J_R_bg = R_c2n.inverse().matrix() *  J_R_bg - delta_t * J_r ;
            
        }
        delta_p += delta_v * delta_t + 0.5 * delta_R * (acc - ba) * squared_delta_t;
        delta_v += delta_R .matrix()* (acc - ba) * delta_t;
        delta_R *= R_c2n;
        
    }
    void IMUPreintegrator::UpdateBias(const Vec3 &delta_bg, const Vec3 &delta_ba)
    {
        // update the bias in an efficient way
        // alternative way is to Reset, and use the new bias to preintegrate again.
        delta_R *= SO3::exp(J_R_bg * delta_ba);
        delta_v += J_v_bg * delta_bg + J_v_ba * delta_ba;
        delta_p += J_p_bg * delta_bg + J_p_ba * delta_ba; 
    }

}
} // namespace  
