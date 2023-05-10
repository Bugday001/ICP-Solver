#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
// 假设你已经安装了pcl和gtsam，并且包含了相应的头文件
#include <pcl/kdtree/kdtree_flann.h>
#include "gtsam_icp.h"
#include "lidarGtsam.h"

using namespace gtsam;

namespace XICP
{
    GTSAM_ICP::GTSAM_ICP(const YAML::Node &node)
    {
        max_iterations = node["max_iter"].as<int>();
        max_coresspoind_dis = node["max_corr_dist"].as<float>();
        trans_eps = node["trans_eps"].as<float>();
        euc_fitness_eps = node["euc_fitness_eps"].as<float>();
        is_autoDiff = node["is_autoDiff"].as<bool>();
        isDebug = node["isDebug"].as<bool>();
        thread_nums = node["thread_nums"].as<int>();
        max_opt = node["max_opt"].as<int>();
    }

    GTSAM_ICP::~GTSAM_ICP()
    {
    }

    bool GTSAM_ICP::setGICPTargetCloud(const CLOUD_PTR &target)
    {
        gicp_target.setCloudPtr(target);
    }

    Eigen::Matrix4d gtsam2transPose(gtsam::Pose3 pose)
    {
        Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
        matrix(0,3) = pose.translation().x();
        matrix(1,3) = pose.translation().y();
        matrix(2,3) = pose.translation().z();
        matrix.block<3,3>(0,0) = pose.rotation().matrix().cast<double>();
        return matrix;
    }

    /**
     * GICP
     */
    bool GTSAM_ICP::GICPMatch(const CLOUD_PTR &source, const Eigen::Matrix4f &predict_pose,
                              CLOUD_PTR &transformed_source_ptr, Eigen::Matrix4f &result_pose)
    {
        CLOUD_PTR source_ptr = source;
        CLOUD_PTR transform_cloud(new CLOUD());;
        Eigen::Matrix4d T = predict_pose.cast<double>();
        for (int i = 0; i < max_iterations; ++i)
        {
            // 定义点云类型
            // construct noise model
            SharedNoiseModel model = noiseModel::Isotropic::Sigma(3, 0.01);

            // construct pose key
            const Key key = gtsam::symbol_shorthand::X(0);

            // build factor graph
            NonlinearFactorGraph graph;
            pcl::transformPointCloud(*source_ptr, *transform_cloud, T);
            for (int j = 0; j < transform_cloud->size(); ++j)
            {
                const PointType &transform_pt = transform_cloud->at(j);
                if (!pcl::isFinite(transform_pt))
                    continue;
                std::vector<float> res_dis;
                std::vector<int> indices;
                gicp_target.kdtree_flann->nearestKSearch(transform_pt, 1, indices, res_dis);
                if (res_dis.front() > max_coresspoind_dis)
                    continue;
                int idx = indices.front();
                // std::cout<<"ok1"<<std::endl;
                const Point3 nearest_pt = Point3(gicp_target.cloud_ptr->at(indices.front()).x,
                                                                gicp_target.cloud_ptr->at(indices.front()).y,
                                                                gicp_target.cloud_ptr->at(indices.front()).z);
                // std::cout<<"ok2"<<std::endl;
                const Point3 origin_pt = Point3(transform_pt.x,
                                                                transform_pt.y,
                                                                transform_pt.z);
                // std::cout<<"ok3"<<std::endl;
                graph.emplace_shared<IcpFactor>(model, key, nearest_pt, origin_pt);
            }
            std::cout<<"finish"<<std::endl;
            // set initial values
            Values initial;
            initial.insert(key, Pose3::identity());

            // optimize
            const Values optimized =
                LevenbergMarquardtOptimizer(graph, initial /*default parameters*/).optimize();
            std::cout << "   estimated = " << optimized.at<Pose3>(key) << std::endl;
            T = gtsam2transPose(optimized.at<Pose3>(key))*T;
        }
        result_pose = T.cast<float>();
        pcl::transformPointCloud(*source_ptr, *transformed_source_ptr, result_pose);
        std::cout<<"return"<<std::endl;
        return true;
    }
}
