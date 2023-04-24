#ifndef LIDAR_CERES_
#define LIDAR_CERES_

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace ceresICP
{

    Eigen::Matrix3d skew(const Eigen::Vector3d &mat);

    void getTransformFromSe3(const Eigen::Matrix<double, 6, 1> &se3, Eigen::Quaterniond &q, Eigen::Vector3d &t);


    /**
     * 继承自ceres::SizedCostFunction
     * 如果参数块的维度以及残差向量的维度能够在编译时确定，可以使用SizedCostFunction类。
     * 若不知，则要使用CostFunction，并set相关参数
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
     * 利用李群 SE(3) 来表示位姿
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
        //  参数更新  
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


    /**
     * Point-to-Point ICP 自动求导
    */
    struct LidarEdgeFactor { 
        LidarEdgeFactor(Eigen::Vector3d origin_eigen,  Eigen::Vector3d nearest_pt)
            : cur_pt_(origin_eigen), near_pt_(nearest_pt){} 
    
        template<typename T> 
        bool operator()(const T *q, const T *t, T *residual) const { // 仿函数，用于计算残差 
            //ceres::QuaternionParameterization：内部存储顺序为(w,x,y,z)
            // ceres::EigenQuaternionParameterization：内部存储顺序为(x,y,z,w)
            // Eigen::Quaternion(w,x,y,z)：内部存储顺序为(x,y,z,w)（构造函数的时候是wxyz）
            Eigen::Matrix<T, 3, 1> cp{T(cur_pt_.x()), T(cur_pt_.y()), T(cur_pt_.z())}; 
            Eigen::Matrix<T, 3, 1> lpa{T(near_pt_.x()), T(near_pt_.y()), T(near_pt_.z())}; 
            Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
            Eigen::Matrix<T, 3, 1> t_last_curr{T(t[0]), T(t[1]), T(t[2])};
            Eigen::Matrix<T, 3, 1> lp = q_last_curr * cp + t_last_curr;
            Eigen::Matrix<T, 3, 1> nu = (lp - lpa);
            residual[0] = T(nu.x()); 
            residual[1] = T(nu.y()); 
            residual[2] = T(nu.z()); 

            return true; 
        }

        static ceres::CostFunction *autoDiffCostFun(const Eigen::Vector3d origin_eigen,  
                                    const Eigen::Vector3d nearest_pt) 
        { 	// 自动求导AutoDiffCostFunction
            return (new ceres::AutoDiffCostFunction<LidarEdgeFactor, 3, 4, 3>( 
                new LidarEdgeFactor(origin_eigen, nearest_pt) 
            )); 
        }

        Eigen::Vector3d cur_pt_, near_pt_; 

    };  

    /**
     * GICP 自动求导
    */
    struct GICPFactor { 
        GICPFactor(Eigen::Vector3d origin_eigen,  Eigen::Matrix3d origin_C,
                                            Eigen::Vector3d nearest_pt, Eigen::Matrix3d nearest_C)
            : cur_pt_(origin_eigen), cur_C_(origin_C), near_pt_(nearest_pt), near_C_(nearest_C) {
                // std::cout<<"ok-1"<<std::endl;
            } 
    
        template<typename T> 
        bool operator()(const T *q, const T *t, T *residual) const { // 仿函数，用于计算残差 
            //ceres::QuaternionParameterization：内部存储顺序为(w,x,y,z)
            // ceres::EigenQuaternionParameterization：内部存储顺序为(x,y,z,w)
            // Eigen::Quaternion(w,x,y,z)：内部存储顺序为(x,y,z,w)（构造函数的时候是wxyz）
            Eigen::Matrix<T, 3, 1> cp{T(cur_pt_.x()), T(cur_pt_.y()), T(cur_pt_.z())}; 
            Eigen::Matrix<T, 3, 1> lpa{T(near_pt_.x()), T(near_pt_.y()), T(near_pt_.z())}; 
            Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
            Eigen::Matrix<T, 3, 1> t_last_curr{T(t[0]), T(t[1]), T(t[2])};
            // std::cout<<"ok0"<<std::endl;
            Eigen::Matrix<T, 3, 3> cur_C = cur_C_.cast<T>();//B
            Eigen::Matrix<T, 3, 3> near_C = near_C_.cast<T>();//A
            //四元数转为旋转矩阵--先归一化再转为旋转矩阵
            Eigen::Matrix<T, 3, 3> R = q_last_curr.normalized().toRotationMatrix();

            // cur+R*near*R^T
            // std::cout<<"ok1"<<std::endl;
            Eigen::Matrix<T, 3, 3> tmp = (cur_C + R*near_C*R.transpose()).inverse();
            // std::cout<<"ok2"<<std::endl;
            // tmp = tmp.inverse();

            //注意定义R，在此是R*cp-lpa
            Eigen::Matrix<T, 3, 1> dT = R*cp - lpa+ t_last_curr;
            Eigen::Matrix<T, 1, 1> nu = dT.transpose()*tmp*dT;
            // // Eigen::Matrix<T, 3, 1> t_last_curr{T(t[0]), T(t[1]), T(t[2])};
            // // Eigen::Matrix<T, 3, 1> lp = q_last_curr * cp + t_last_curr;
            // // Eigen::Matrix<T, 3, 1> nu = (lp - lpa);
            residual[0] = T(nu(0, 0)); 
            // residual[1] = T(nu.y()); 
            // residual[2] = T(nu.z()); 
            return true; 
        }

        Eigen::Vector3d cur_pt_, near_pt_; 
        Eigen::Matrix3d cur_C_, near_C_;

    };

} // namespace ceresICP

#endif