#ifndef RABBIT_MATHUTIL_H
#define RABBIT_MATHUTIL_H
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/Eigenvalues> 
#include <Eigen/Eigen>
#include "sophus/se3.hpp"
#include "sophus_utils.hpp"
/*
The file is part of rabbit. Some of the codes are adopted from Lio-mapping and vins-mono.
*/
namespace rabbit
{
namespace util
{
    typedef Eigen::Matrix<double, 3, 3> Mat3;
    typedef std::vector<Mat3, Eigen::aligned_allocator<Mat3> > Mat3List; 
    
    typedef Eigen::Matrix<float, 4, 4> Mat4f;
    typedef std::vector<Mat4f, Eigen::aligned_allocator<Mat4f> > Mat4fList; 

    typedef Eigen::Matrix<double, 4, 4> Mat4;
    typedef std::vector<Mat4, Eigen::aligned_allocator<Mat4> > Mat4List; 
    
    typedef Eigen::Matrix<double, 6, 6> Mat6;
    typedef Eigen::Matrix<double, 9, 9> Mat9;
    typedef Eigen::Matrix<double, 9, 6> Mat96;

     typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatX;


    typedef Eigen::Vector3f Vec3f;
    typedef std::vector<Vec3f, Eigen::aligned_allocator<Vec3f> > Vec3fList; 

    typedef Eigen::Matrix<double, 3, 1> Vec3;
    typedef std::vector<Vec3, Eigen::aligned_allocator<Vec3> > Vec3List; 

    typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VecX;
    typedef std::vector<VecX, Eigen::aligned_allocator<VecX> > VecXList; 

    template <int T>
        using Vec = Eigen::Matrix<double, T, 1>;

    template <int T>
        using VecList = std::vector<Eigen::Matrix<double, T, 1>, 
            Eigen::aligned_allocator<Eigen::Matrix<double, T, 1>>>;

    

    typedef Sophus::SE3f SE3f;
    typedef std::vector<SE3f, Eigen::aligned_allocator<SE3f>> SE3fList;

    typedef Sophus::SE3d SE3;
    typedef std::vector<SE3, Eigen::aligned_allocator<SE3>> SE3List;

    typedef Sophus::SO3f SO3f;
    typedef std::vector<SO3f, Eigen::aligned_allocator<SO3f>> SO3fList;

    typedef Sophus::SO3d SO3;
    typedef std::vector<SO3, Eigen::aligned_allocator<SO3>> SO3List;

    typedef Eigen::Matrix<float, 6, 1> Vec6f;
    typedef Eigen::Matrix<double, 6, 1> Vec6;

    // from degree to radian
    inline double Rad2Deg(double radians)
    {
        return radians * 180.0 / M_PI;
    }
    // from radian to degree
    inline double Deg2Rad(double degrees)
    {
        return degrees * M_PI / 180.0;
    }
    template<typename Derived>
    inline Eigen::Matrix<typename Derived::Scalar, 3, 3> SkewSymmetric(const Eigen::MatrixBase<Derived> &v3d) 
    {
        Eigen::Matrix<typename Derived::Scalar, 3, 3> m;
        m << typename Derived::Scalar(0), -v3d.z(), v3d.y(),
            v3d.z(), typename Derived::Scalar(0), -v3d.x(),
            -v3d.y(), v3d.x(), typename Derived::Scalar(0);
        return m;
        }
        inline Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R)
        {
        Eigen::Vector3d n = R.col(0);
        Eigen::Vector3d o = R.col(1);
        Eigen::Vector3d a = R.col(2);

        Eigen::Vector3d ypr(3);
        double y = atan2(n(1), n(0));
        double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
        double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
        ypr(0) = y;
        ypr(1) = p;
        ypr(2) = r;

        return ypr / M_PI * 180.0;
    }
    template <typename Derived>
    inline Eigen::Matrix<typename Derived::Scalar, 3, 3> ypr2R(const Eigen::MatrixBase<Derived> &ypr)
    {
        typedef typename Derived::Scalar Scalar_t;

        Scalar_t y = ypr(0) / 180.0 * M_PI;
        Scalar_t p = ypr(1) / 180.0 * M_PI;
        Scalar_t r = ypr(2) / 180.0 * M_PI;

        Eigen::Matrix<Scalar_t, 3, 3> Rz;
        Rz << cos(y), -sin(y), 0,
            sin(y), cos(y), 0,
            0, 0, 1;

        Eigen::Matrix<Scalar_t, 3, 3> Ry;
        Ry << cos(p), 0., sin(p),
            0., 1., 0.,
            -sin(p), 0., cos(p);

        Eigen::Matrix<Scalar_t, 3, 3> Rx;
        Rx << 1., 0., 0.,
            0., cos(r), -sin(r),
            0., sin(r), cos(r);

        return Rz * Ry * Rx;
    }

    inline Mat3 RightJacobianSO3(const Vec3 &phi)
    {
        Mat3 result;
        Sophus::rightJacobianSO3(phi, result);
        return result;
    }

    inline Mat3 RightJacobianInvSO3(const Vec3 &phi)
    {
        Mat3 result;
        Sophus::rightJacobianInvSO3(phi, result);
        return result;
    }

}
}
#endif