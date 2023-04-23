#ifndef LIDAR_CERES_
#define LIDAR_CERES_

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace test_ceres
{

    Eigen::Matrix3d skew(const Eigen::Vector3d &mat);

    void getTransformFromSe3(const Eigen::Matrix<double, 6, 1> &se3, Eigen::Quaterniond &q, Eigen::Vector3d &t);


    /**
     * 如果参数块的维度以及残差向量的维度能够在编译时确定，可以使用SizedCostFunction类
    */
    class EdgeAnalyticCostFuntion : public ceres::SizedCostFunction<3, 7>
    {
    public:
        EdgeAnalyticCostFuntion(Eigen::Vector3d cur_pt_, Eigen::Vector3d near_pt_);
        virtual ~EdgeAnalyticCostFuntion() {}

        //函数Evaluate()实现残差和雅可比矩阵计算
        virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

        Eigen::Vector3d cur_pt, near_pt;
    };

    /**
     * 优化不再欧几里得空间而，不支持广义的加法等操作。
     * ceres中LocalParameterization类支持自定义加法和Jocobian计算。
     */
    class PoseSE3Parameterization : public ceres::LocalParameterization
    {
    public:
        PoseSE3Parameterization(){};
        virtual ~PoseSE3Parameterization(){};
        // Generalization of the addition operation,
        //
        //   x_plus_delta = Plus(x, delta)
        //
        // with the condition that Plus(x, 0) = x.
        virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;

        // The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
        //
        // jacobian is a row-major GlobalSize() x LocalSize() matrix.
        virtual bool ComputeJacobian(const double *x, double *jacobian) const;

         // Size of x.
        virtual int GlobalSize() const { return 7; }

         // Size of delta.
        virtual int LocalSize() const { return 6; }
    };
} // namespace test_ceres

#endif