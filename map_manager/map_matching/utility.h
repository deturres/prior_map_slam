#ifndef UTILITY_H
#define UTILITY_H

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "bm_defs.h"
#include "opencv2/core/core.hpp"

namespace utility
{
    typedef std::pair< int,Vector6d > FakeGPSSeq;
    typedef std::vector<FakeGPSSeq, Eigen::aligned_allocator<Vector6d> > NewPosesGPS;


    template <typename Derived>
    inline Eigen::Matrix<typename Derived::Scalar, 3, 3> quat2mat(const Eigen::MatrixBase<Derived>& q) {
        const typename Derived::Scalar& qx = q.x();
        const typename Derived::Scalar& qy = q.y();
        const typename Derived::Scalar& qz = q.z();
        typename Derived::Scalar qw = sqrt(1.f - q.squaredNorm());
        Eigen::Matrix<typename Derived::Scalar, 3, 3> R;
        R << qw*qw + qx*qx - qy*qy - qz*qz, 2*(qx*qy - qw*qz) , 2*(qx*qz + qw*qy),
                2*(qx*qy + qz*qw) , qw*qw - qx*qx + qy*qy - qz*qz, 2*(qy*qz - qx*qw),
                2*(qx*qz - qy*qw) , 2*(qy*qz + qx*qw), qw*qw - qx*qx - qy*qy + qz*qz;

        return R;
    }
    template <typename Derived>
    inline Eigen::Transform<typename Derived::Scalar, 3, Eigen::Isometry> v2t(const Eigen::MatrixBase<Derived>& x_) {
      Eigen::Transform<typename Derived::Scalar, 3, Eigen::Isometry> X;
      Eigen::Matrix<typename Derived::Scalar, 6, 1> x(x_);
      X.template linear() = quat2mat(x.template block<3,1>(3,0));
      X.template translation() = x.template block<3,1>(0,0);
      return X;
    }

    template <typename Scalar>
    inline Eigen::Matrix<Scalar, 6, 1> t2v(const Eigen::Transform<Scalar, 3, Eigen::Isometry>& X)
    {
      Eigen::Matrix<Scalar,6,1> v;
      v.template block<3,1>(0,0) = X.translation();
      v.template block<3,1>(3,0) = mat2quat(X.linear());
      return v;
    }

    //reading fake gps pose from file
    bool read_FakeGPS(std::ifstream& is, NewPosesGPS& gps);
    //Isometry matrix to vector
    Eigen::Vector3d t2v_2d(const Eigen::Isometry2d& iso);
    //Vector to isometry matrix
    Eigen::Isometry2d v2t_2d(const Eigen::Vector3d& v);
}

#endif //UTILITY_H
