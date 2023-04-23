#include "ceres_icp.h"
#include "lidarCeres.h"

int USE_AUTODIFF = 1;

namespace ceresICP
{
    CERES_ICP::CERES_ICP(const YAML::Node &node)
        : kdtree_flann(new pcl::KdTreeFLANN<PointType>)
    {
        max_iterations = node["max_iter"].as<int>();
        max_coresspoind_dis = node["max_corr_dist"].as<float>();
        trans_eps = node["trans_eps"].as<float>();
        euc_fitness_eps = node["euc_fitness_eps"].as<float>();
        is_autoDiff = node["is_autoDiff"].as<bool>();
    }

    CERES_ICP::~CERES_ICP()
    {
    }

    bool CERES_ICP::setTargetCloud(const CLOUD_PTR &target)
    {
        target_ptr = target;
        kdtree_flann->setInputCloud(target);
    }


    /**
     * 解析求导以及自动求导
     * 使用is_autoDiff选择
    */
    bool CERES_ICP::scanMatch(const CLOUD_PTR &source, const Eigen::Matrix4f &predict_pose,
                                  CLOUD_PTR &transformed_source_ptr, Eigen::Matrix4f &result_pose)
    {
        source_ptr = source;
        CLOUD_PTR transform_cloud(new CLOUD());
        Eigen::Matrix4d T = predict_pose.cast<double>();
        q_w_curr = Eigen::Quaterniond(T.block<3, 3>(0, 0));
        t_w_curr = T.block<3, 1>(0, 3);

        for (int i = 0; i < max_iterations; ++i)
        {
            pcl::transformPointCloud(*source_ptr, *transform_cloud, T);
            ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
            ceres::Problem::Options problem_options;
            ceres::Problem problem(problem_options);
            if(is_autoDiff) {
                std::cout<<"auto Diff, i: "<<i<<std::endl;
                ceres::LocalParameterization *q_parameterization = 
                new ceres::EigenQuaternionParameterization(); 	// xyzw顺序
                // 注意到如果使用了LocalParameterization，那么必须添加AddParameterBlock来添加不规则+-优化变量
                problem.AddParameterBlock(parameters, 4, q_parameterization); 
                problem.AddParameterBlock(parameters + 4, 3);
            } 
            else {
                //如果我们的参数属于正常的plus更新的话，也就是没有过参数（LocalParameterization），没有manifold space，那么就完全不需要调用AddParameterBlock或者SetParameterization函数；
                //void AddParameterBlock(double* values, int size, LocalParameterization* local_parameterization);
                problem.AddParameterBlock(parameters, 7, new ceresICP::PoseSE3Parameterization());
            }
            

            // std::cout << "------------ " << i << "------------" << std::endl;
            for (int j = 0; j < transform_cloud->size(); ++j)
            {
                const PointType &origin_pt = source_ptr->points[j];
                if (!pcl::isFinite(origin_pt))
                    continue;

                const PointType &transform_pt = transform_cloud->at(j);
                std::vector<float> res_dis;
                std::vector<int> indices;
                kdtree_flann->nearestKSearch(transform_pt, 1, indices, res_dis);
                if (res_dis.front() > max_coresspoind_dis)
                    continue;

                Eigen::Vector3d nearest_pt = Eigen::Vector3d(target_ptr->at(indices.front()).x,
                                                             target_ptr->at(indices.front()).y,
                                                             target_ptr->at(indices.front()).z);

                Eigen::Vector3d origin_eigen(origin_pt.x, origin_pt.y, origin_pt.z);
                if(is_autoDiff) {
                    ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<LidarEdgeFactor, 3, 4, 3>( 
                    new LidarEdgeFactor(origin_eigen, nearest_pt));
                    problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
                }
                else {
                    ceres::CostFunction *cost_function = new ceresICP::EdgeAnalyticCostFuntion(origin_eigen, nearest_pt);
                    problem.AddResidualBlock(cost_function, loss_function, parameters);
                }

            }
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.max_num_iterations = 10;
            options.minimizer_progress_to_stdout = false;
            options.check_gradients = false;
            options.gradient_check_relative_precision = 1e-4;

            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);

            T.setIdentity();
            T.block<3, 1>(0, 3) = t_w_curr;
            T.block<3, 3>(0, 0) = q_w_curr.toRotationMatrix();

            // std::cout << "T\n"
            //           << T << std::endl;
        }

        final_pose = T.cast<float>();
        result_pose = T.cast<float>();
        pcl::transformPointCloud(*source_ptr, *transformed_source_ptr, result_pose);
        return true;
    }
