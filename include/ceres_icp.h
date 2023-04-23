#ifndef CERES_ICP_
#define CERES_ICP_
#include <eigen3/Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "ceres_icp.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <yaml-cpp/yaml.h>


typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> CLOUD;
typedef CLOUD::Ptr CLOUD_PTR;

namespace ceresICP
{   

    class CERES_ICP
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        CERES_ICP(const YAML::Node &node);
        ~CERES_ICP();
        bool setTargetCloud(const CLOUD_PTR &target);
        bool scanMatch(const CLOUD_PTR &source, const Eigen::Matrix4f &predict_pose,
                       CLOUD_PTR &transformed_source_ptr, Eigen::Matrix4f &result_pose);

        /*============= AUTODIFF ================*/
        bool scanMatch_autoDiff(const CLOUD_PTR &source, const Eigen::Matrix4f &predict_pose,
                       CLOUD_PTR &transformed_source_ptr, Eigen::Matrix4f &result_pose);

        float getFitnessScore();

    private:
        CLOUD_PTR target_ptr, source_ptr;
        Eigen::Matrix4f final_pose;
        int max_iterations;
        float max_coresspoind_dis;
        float trans_eps;
        float euc_fitness_eps;

        bool is_autoDiff = false;

        double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
        //Map就是引用，同一块地址
        Eigen::Map<Eigen::Quaterniond> q_w_curr = Eigen::Map<Eigen::Quaterniond>(parameters);
        Eigen::Map<Eigen::Vector3d> t_w_curr = Eigen::Map<Eigen::Vector3d>(parameters + 4);

        pcl::KdTreeFLANN<PointType>::Ptr kdtree_flann;
    };
}

#endif