#ifndef RABBIT_LOCAL_PARAMETERIZATION_H
#define RABBIT_LOCAL_PARAMETERIZATION_H
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>
#include <pcl/kdtree/kdtree_flann.h>
#include "Utils/Utils.h"

#include "sophus/interpolate.hpp"
#include <ceres/local_parameterization.h>
namespace rabbit
{
namespace factor
{
    using namespace util;

    class LocalParameterizationSE3 : public ceres::LocalParameterization 
    {
        public:
        virtual ~LocalParameterizationSE3() {}

        // SE3 plus operation for Ceres
        //
        //  T * exp(x)
        //
        virtual bool Plus(double const* T_raw, double const* delta_raw,
                            double* T_plus_delta_raw) const {
            Eigen::Map<SE3 const> const T(T_raw);
            Eigen::Map<Vec6 const> const delta(delta_raw);
            Eigen::Map<SE3> T_plus_delta(T_plus_delta_raw);
            T_plus_delta = T * SE3::exp(delta);
            return true;
        }

        // Jacobian of SE3 plus operation for Ceres
        //
        // Dx T * exp(x)  with  x=0
        //
        virtual bool ComputeJacobian(double const* T_raw,
                                    double* jacobian_raw) const {
            Eigen::Map<SE3 const> T(T_raw);
            Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> jacobian(
                jacobian_raw);
            jacobian = T.Dx_this_mul_exp_x_at_0();
            return true;
        }

        virtual int GlobalSize() const { return SE3::num_parameters; }

        virtual int LocalSize() const { return SE3::DoF; }
    };

    class LocalParameterizationSO3 : public ceres::LocalParameterization 
    {
        public:
        virtual ~LocalParameterizationSO3() {}

        // SE3 plus operation for Ceres
        //
        //  T * exp(x)
        //
        // for updating
        virtual bool Plus(double const* T_raw, double const* delta_raw,
                            double* T_plus_delta_raw) const {
            Eigen::Map<SO3 const> const T(T_raw);
            Eigen::Map<Vec3 const> const delta(delta_raw);
            Eigen::Map<SO3> T_plus_delta(T_plus_delta_raw);
            T_plus_delta = T * SO3::exp(delta);
            return true;
        }

        // Jacobian of SE3 plus operation for Ceres
        //
        // Dx T * exp(x)  with  x=0
        //
        // for compute the delta
        virtual bool ComputeJacobian(double const* T_raw,
                                    double* jacobian_raw) const {
            Eigen::Map<SO3 const> T(T_raw);
            Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor>> jacobian(
                jacobian_raw);
            jacobian = T.Dx_this_mul_exp_x_at_0();
            return true;
        }

        virtual int GlobalSize() const { return SO3::num_parameters; }

        virtual int LocalSize() const { return SO3::DoF; }
    };
        
}
}
#endif