#ifndef LIDAR_G2O_
#define LIDAR_G2O_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <sophus/se3.hpp>

#include <g2o/core/g2o_core_api.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/slam3d/types_slam3d.h> //顶点类型

#include <iostream>
#include <cmath>

namespace XICP
{
    /**
     * 使用g2o求解
     */
    class GICPVertex : public g2o::BaseVertex<6, Sophus::SE3d>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /**
         * 顶点是可以设置成固定的。当不需要变动某个顶点时，使用setFixed函数来固定。
         * 通常，一个优化问题中，至少需要固定一个顶点，否则所有的顶点都在浮动，优化效果也不会好。
         */
        GICPVertex(bool fixed = false)
        {
            setToOriginImpl();
            setFixed(fixed);
        }

        GICPVertex(const Sophus::SE3d e, bool fixed = false)
        {
            _estimate = e;
            setFixed(fixed);
        }

        /**
         * 用于重置顶点的数据。顶点包含的数据变量是_estimate。
         * 该变量的类型即是g2o::BaseVertex<1, double>中设置的double。
         * 该函数正是用于重置_estimate和使顶点恢复默认状态。
         */
        virtual void setToOriginImpl() override
        {
            _estimate = Sophus::SE3d();
            ;
        }

        /**
         * 用于叠加优化量的步长。注意有时候这样的叠加操作并不是线性的。
         */
        virtual void oplusImpl(const double *update) override
        {
            Eigen::Matrix<double, 6, 1> delta_r;
            delta_r << update[0], update[1], update[2], update[3], update[4], update[5];
            // _estimate = Sophus::SE3d::exp(delta_r) * _estimate;  //  左乘
            _estimate = _estimate * Sophus::SE3d::exp(delta_r); // 不知道原理
        }

        // 存盘和读盘：留空
        virtual bool read(std::istream &in) { return true; }

        virtual bool write(std::ostream &out) const { return true; }
    };

    /**
     * 误差模型 模板参数：误差值维度，测量值类型，连接顶点类型
     *
     */
    class ICPEdge : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, GICPVertex>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ICPEdge(Eigen::Vector3d x) : BaseUnaryEdge(), _p(x) {}

        // 计算模型误差
        virtual void computeError() override
        {
            const GICPVertex *v = static_cast<const GICPVertex *>(_vertices[0]);
            const Sophus::SE3d T = v->estimate();
            _error = T * _p - _measurement;
        }

        /**
         * 计算雅可比矩阵
         * 支持自动求导，可以不实现。但实现会更快。
         */
        virtual void linearizeOplus() override
        {
            const GICPVertex *v = static_cast<const GICPVertex *>(_vertices[0]);
            const Sophus::SE3d T = v->estimate();
            //  e =Tp-p  ,de/T的李代数
            _jacobianOplusXi.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
            _jacobianOplusXi.block<3, 3>(0, 3) = -(T.matrix()).block<3, 3>(0, 0) * Sophus::SO3d::hat(_p); //  右乘扰动
        }

        virtual bool read(std::istream &in) {}

        virtual bool write(std::ostream &out) const {}

    private:
        Eigen::Vector3d _p;
    };

    /**
     * 二元边
     * 误差模型 模板参数：误差值维度，测量值类型，连接顶点类型
     *
     */
    class ICPEdge2 : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, GICPVertex, GICPVertex>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ICPEdge2(Eigen::Vector3d x) : BaseBinaryEdge(), _p(x) {}

        // 计算模型误差
        virtual void computeError() override
        {
            const GICPVertex *v1 = static_cast<const GICPVertex *>(_vertices[0]);
            const GICPVertex *v2 = static_cast<const GICPVertex *>(_vertices[1]);
            const Sophus::SE3d T1 = v1->estimate();
            const Sophus::SE3d T2 = v2->estimate();
            _error = T1 * _p - T2 * _measurement;
        }

        /**
         * 计算雅可比矩阵
         * 支持自动求导，可以不实现。但实现会更快。
         */
        // virtual void linearizeOplus() override
        // {
        //     const GICPVertex *v = static_cast<const GICPVertex *>(_vertices[0]);
        //     const Sophus::SE3d T = v->estimate();
        //     //  e =Tp-p  ,de/T的李代数
        //     _jacobianOplusXi.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        //     _jacobianOplusXi.block<3, 3>(0, 3) = -(T.matrix()).block<3, 3>(0, 0) * Sophus::SO3d::hat(_p); //  右乘扰动
        // }

        virtual bool read(std::istream &in) {}

        virtual bool write(std::ostream &out) const {}

    private:
        Eigen::Vector3d _p;
    };

    /**
     * 使用g2o自带的顶点
     * 普通ICP
    */
    class g2oSE3Edge : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3Expmap>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        g2oSE3Edge(const Eigen::Vector3d &point) : _point(point) {}

        virtual void computeError()
        {
            const g2o::VertexSE3Expmap *pose = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
            // measurement is p, point is p'
            _error = _measurement - pose->estimate().map(_point);
        }

        // virtual void linearizeOplus()
        // {
        //     g2o::VertexSE3Expmap *pose = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
        //     g2o::SE3Quat T(pose->estimate());
        //     Eigen::Vector3d xyz_trans = T.map(_point);

        //     _jacobianOplusXi.block<3, 3>(0, 0) = Sophus::SO3::hat(xyz_trans);
        //     _jacobianOplusXi.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();

        // }

        bool read(std::istream &in) {}
        bool write(std::ostream &out) const {}

    protected:
        Eigen::Vector3d _point;
    };

    /**
     * 使用g2o自带的顶点
     * GICP
    */
    class g2oSE3GICPEdge : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        g2oSE3GICPEdge(double* data_in) 
        {
            // _point = data_in;
            _point = Eigen::Map<Eigen::Vector3d> (data_in);
            cur_C = Eigen::Map<Eigen::Matrix3d> (data_in+3); 
            near_C = Eigen::Map<Eigen::Matrix3d>(data_in+12);
        }

        virtual void computeError()
        {
            const g2o::VertexSE3 *pose = static_cast<const g2o::VertexSE3 *>(_vertices[0]);
            // measurement is p, point is p'
            // 四元数转为旋转矩阵--先归一化再转为旋转矩阵
            Eigen::MatrixXd T = Eigen::Isometry3d(pose->estimate()).matrix();
            Eigen::MatrixXd R = T.block<3, 3>(0, 0);
            Eigen::MatrixXd t_w_curr = T.block<3, 1>(0, 3);
            // cur+R*near*R^T，求逆，在分解为三角矩阵*它的转置，使用分解效果还不如不分解。
            Eigen::MatrixXd LT = (cur_C + R * near_C * R.transpose()).inverse().llt().matrixL();

            // 注意定义R，在此是R*cp-lpa
            Eigen::MatrixXd dT = R * _point - _measurement + t_w_curr;
            Eigen::MatrixXd nu = LT * dT;
            _error = nu;
        }

        // virtual void linearizeOplus()
        // {
        //     g2o::VertexSE3Expmap *pose = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
        //     g2o::SE3Quat T(pose->estimate());
        //     Eigen::Vector3d xyz_trans = T.map(_point);

        //     _jacobianOplusXi.block<3, 3>(0, 0) = Sophus::SO3::hat(xyz_trans);
        //     _jacobianOplusXi.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();

        // }

        bool read(std::istream &in) {}
        bool write(std::ostream &out) const {}

    protected:
        Eigen::Vector3d _point;
        Eigen::Matrix3d near_C, cur_C;
    };
} // namespace XICP

#endif