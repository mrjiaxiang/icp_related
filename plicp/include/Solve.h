//
// Created by melody on 2022/9/15.
//

#ifndef PLICP_SOLVE_H
#define PLICP_SOLVE_H
#include <math.h>

#include <iostream>
#include <ceres/ceres.h>
#include "pose2d.h"

template <typename T>
Eigen::Matrix<T, 2, 2> RotationMatrix2D(T yaw_radians) {
    const T cos_yaw = cos(yaw_radians);
    const T sin_yaw = sin(yaw_radians);

    Eigen::Matrix<T, 2, 2> rotation;
    rotation << cos_yaw, -sin_yaw, sin_yaw, cos_yaw;
    return rotation;
}

template <typename T>
inline T NormalizeAngle(const T &angle_radians) {
    T two_pi(2.0 * M_PI);
    return angle_radians - two_pi * ceres::floor((angle_radians + T(M_PI)) / two_pi);
}

template <typename T>
inline T GetSin(T x) {
    return sin(x);
}

template <typename T>
inline T GetCos(T x) {
    return cos(x);
}

class CostFunctor {
public:
    CostFunctor(double s_x, double s_y, double s_theta, double r_x, double r_y, double r_theta)
            : s_(s_x, s_y),
              s_yaw_(s_theta),
              r_(r_x, r_y),
              r_yaw_(r_theta) {}
    template <typename T>
    bool operator()(const T *const data, T *residual) const {
        //const Eigen::Matrix<T, 3, 1> l_p(data_correct[0], data_correct[1], data_correct[2]);
        Eigen::Matrix<T, 3, 1> tmp_xy;
        tmp_xy(0, 0) = -data[0] * GetCos(data[2]) - data[1] * GetSin(data[2]);
        tmp_xy(1, 0) = data[0] * GetSin(data[2]) - data[1] * GetCos(data[2]);
        tmp_xy(2, 0) = -(data[2]);

        tmp_xy(0, 0) = tmp_xy(0, 0) + r_(0, 0) * GetCos(r_yaw_) - r_(1, 0) * GetSin(r_yaw_);
        tmp_xy(1, 0) = tmp_xy(1, 0) + r_(0, 0) * GetSin(r_yaw_) + r_(1, 0) * GetCos(r_yaw_);
        tmp_xy(2, 0) = NormalizeAngle(tmp_xy(2, 0) + r_yaw_);

        tmp_xy(0, 0) = tmp_xy(0, 0) + data[0] * GetCos(data[2]) - data[1] * GetSin(data[2]);
        tmp_xy(1, 0) = tmp_xy(1, 0) + data[0] * GetSin(data[2]) + data[1] * GetCos(data[2]);
        tmp_xy(2, 0) = NormalizeAngle(tmp_xy(2, 0) + data[2]);

        residual[0] = s_(0, 0) - tmp_xy(0, 0);
        residual[1] = s_(1, 0) - tmp_xy(1, 0);
        residual[2] = NormalizeAngle(s_yaw_ - tmp_xy(2, 0));

        return true;
    }

    /*
    static ceres::CostFunction *Create(double s_x, double s_y, double s_theta, double r_x, double r_y, double r_theta) {
        return (new ceres::AutoDiffCostFunction<CostFunctor, 3, 3>(
                new CostFunctor(s_x, s_y, s_theta, r_x, r_y, r_theta)));
    }
    */

private:
    const Eigen::Vector2d s_;
    const double s_yaw_;
    const Eigen::Vector2d r_;
    const double r_yaw_;
};

class Solver {
public:
    Solver();
    ~Solver();
    void Calib(std::vector<std::pair<Pose2D,Pose2D>> &sync_data, double *data);
};

#endif //PLICP_SOLVE_H
