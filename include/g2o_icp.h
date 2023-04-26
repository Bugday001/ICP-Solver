#ifndef G2O_ICP_
#define G2O_ICP_
#include <eigen3/Eigen/Core>
#include <ceres/rotation.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <vector>
#include "ceres_icp.h"
#include <yaml-cpp/yaml.h>

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> CLOUD;
typedef CLOUD::Ptr CLOUD_PTR;

namespace XICP
{   
    class G2O_ICP
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        G2O_ICP(const YAML::Node &node);
        ~G2O_ICP();
        bool setGICPTargetCloud(const CLOUD_PTR &target);

        /*============= GICP ================*/
        bool GICPMatch(const CLOUD_PTR &source, const Eigen::Matrix4f &predict_pose,
                                CLOUD_PTR &transformed_source_ptr, Eigen::Matrix4f &result_pose);

    private:
        CLOUD_PTR target_ptr, source_ptr;
        GICPPoint gicp_target, gicp_source;
        Eigen::Matrix4f final_pose;
        //params
        int max_iterations;
        float max_coresspoind_dis;
        float trans_eps;
        float euc_fitness_eps;
        int thread_nums = 1;
        bool is_autoDiff = false, isDebug = false;
        int max_opt = 10;
        double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
        //Map就是引用，同一块地址
        Eigen::Map<Eigen::Quaterniond> q_w_curr = Eigen::Map<Eigen::Quaterniond>(parameters);
        Eigen::Map<Eigen::Vector3d> t_w_curr = Eigen::Map<Eigen::Vector3d>(parameters + 4);

    };
}

#endif