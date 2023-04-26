#include "ceres_icp.h"
#include "lidarCeres.h"
#include <chrono>

using namespace std::literals::chrono_literals;
namespace XICP
{
    CERES_ICP::CERES_ICP(const YAML::Node &node)
        : kdtree_flann(new pcl::KdTreeFLANN<PointType>)
    {
        max_iterations = node["max_iter"].as<int>();
        max_coresspoind_dis = node["max_corr_dist"].as<float>();
        trans_eps = node["trans_eps"].as<float>();
        euc_fitness_eps = node["euc_fitness_eps"].as<float>();
        is_autoDiff = node["is_autoDiff"].as<bool>();
        isDebug = node["isDebug"].as<bool>();
        thread_nums = node["ceres_thread_nums"].as<int>();
    }

    CERES_ICP::~CERES_ICP()
    {
    }

    bool CERES_ICP::setTargetCloud(const CLOUD_PTR &target)
    {
        target_ptr = target;
        kdtree_flann->setInputCloud(target);
    }

    bool CERES_ICP::setGICPTargetCloud(const CLOUD_PTR &target)
    {
        gicp_target.setCloudPtr(target);
    }

    /**
     * init GICPPoint
    */
    GICPPoint::GICPPoint():kdtree_flann(new pcl::KdTreeFLANN<PointType>){ }

    /**
     * 插入点云指针，计算协方差矩阵
     */
    void GICPPoint::setCloudPtr(const CLOUD_PTR &cloud)
    {
        cloud_ptr = cloud;
        kdtree_flann->setInputCloud(cloud);
        computeCov();
    }

    /**
     * 更新点云协方差矩阵
    */
    void  GICPPoint::computeCov()
    {
                // 以下计算协方差矩阵
        int N = cloud_ptr->size();
        int K = 20; // number of closest points to use for local covariance estimate
        double mean[3];
        Cs.resize(N);
        for (int i = 0; i < N; i++)
        {
            Eigen::Matrix3d &cov = Cs[i];
            // zero out the cov and mean
            for (int k = 0; k < 3; k++)
            {
                mean[k] = 0.;
                for (int l = 0; l < 3; l++)
                {
                    cov(k, l) = 0.;
                }
            }
            PointType &cur_ptr = cloud_ptr->at(i);
            std::vector<float> res_dis;
            std::vector<int> indices;
            kdtree_flann->nearestKSearch(cur_ptr, K, indices, res_dis);

            // find the covariance matrix
            for (int j = 0; j < K; j++)
            {
                PointType &pt = cloud_ptr->at(indices[j]);

                mean[0] += pt.x;
                mean[1] += pt.y;
                mean[2] += pt.z;

                cov(0, 0) += pt.x * pt.x;

                cov(1, 0) += pt.y * pt.x;
                cov(1, 1) += pt.y * pt.y;

                cov(2, 0) += pt.z * pt.x;
                cov(2, 1) += pt.z * pt.y;
                cov(2, 2) += pt.z * pt.z;
            }

            mean[0] /= (double)K;
            mean[1] /= (double)K;
            mean[2] /= (double)K;
            // get the actual covariance
            for (int k = 0; k < 3; k++)
            {
                for (int l = 0; l <= k; l++)
                {
                    cov(k, l) /= (double)K;
                    cov(k, l) -= mean[k] * mean[l];
                    cov(l, k) = cov(k, l);
                }
            }
            // compute the SVD
            Eigen::JacobiSVD<Eigen::MatrixXd> svd(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
            //svd.singularValues(), svd.matrixU(), svd.matrixV()

            // zero out the cov matrix, since we know U = V since C is symmetric
            cov.setZero();
            Eigen::Matrix3d tmp = Eigen::MatrixXd::Identity(3, 3);
            tmp(2, 2) = gicp_epsilon_;  // smallest singular value replaced by gicp_epsilon
            // reconstitute the covariance matrix with modified singular values using the column vectors in V.
            cov = svd.matrixV()*tmp*svd.matrixV().transpose();
        }
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
            if (is_autoDiff)
            {
                if(isDebug)
                    std::cout << "auto Diff, i: " << i << std::endl;
                ceres::LocalParameterization *q_parameterization =
                    new ceres::EigenQuaternionParameterization(); // xyzw顺序
                // 注意到如果使用了LocalParameterization，那么必须添加AddParameterBlock来添加不规则+-优化变量
                problem.AddParameterBlock(parameters, 4, q_parameterization);
                problem.AddParameterBlock(parameters + 4, 3);
            }
            else
            {
                // 如果我们的参数属于正常的plus更新的话，也就是没有过参数（LocalParameterization），没有manifold space，那么就完全不需要调用AddParameterBlock或者SetParameterization函数；
                // void AddParameterBlock(double* values, int size, LocalParameterization* local_parameterization);
                problem.AddParameterBlock(parameters, 7, new XICP::PoseSE3Parameterization());
            }

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
                if (is_autoDiff)
                {
                    ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<LidarEdgeFactor, 3, 4, 3>(
                        new LidarEdgeFactor(origin_eigen, nearest_pt));
                    problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
                }
                else
                {
                    ceres::CostFunction *cost_function = new XICP::EdgeAnalyticCostFuntion(origin_eigen, nearest_pt);
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

    /**
     * GICP
    */
    bool CERES_ICP::GICPMatch(const CLOUD_PTR &source, const Eigen::Matrix4f &predict_pose,
                         CLOUD_PTR &transformed_source_ptr, Eigen::Matrix4f &result_pose)
    {   
        CLOUD_PTR transform_cloud(new CLOUD(*source));
        gicp_source.setCloudPtr(transform_cloud);
        Eigen::Matrix4d T = predict_pose.cast<double>();
        q_w_curr = Eigen::Quaterniond(T.block<3, 3>(0, 0));
        t_w_curr = T.block<3, 1>(0, 3);
        for (int i = 0; i < max_iterations; ++i)
        {
            if(isDebug) {
                std::cout << "auto Diff, i: " << i << ", using time: ";
            }
            auto begin_time = std::chrono::high_resolution_clock::now(); //记录当前时间
            pcl::transformPointCloud(*gicp_source.cloud_ptr, *transform_cloud, T);
            gicp_source.computeCov();
            ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
            ceres::Problem::Options problem_options;
            ceres::Problem problem(problem_options);
            ceres::LocalParameterization *q_parameterization =
                new ceres::EigenQuaternionParameterization(); // xyzw顺序
            // 注意到如果使用了LocalParameterization，那么必须添加AddParameterBlock来添加不规则+-优化变量
            problem.AddParameterBlock(parameters, 4, q_parameterization);
            problem.AddParameterBlock(parameters + 4, 3);

            for (int j = 0; j < transform_cloud->size(); ++j)
            {
                const PointType &origin_pt = gicp_source.cloud_ptr->points[j];
                if (!pcl::isFinite(origin_pt))
                    continue;

                const PointType &transform_pt = transform_cloud->at(j);
                std::vector<float> res_dis;
                std::vector<int> indices;
                gicp_target.kdtree_flann->nearestKSearch(transform_pt, 1, indices, res_dis);
                if (res_dis.front() > max_coresspoind_dis)
                    continue;
                int idx = indices.front();
                const Eigen::Vector3d nearest_pt = gicp_target.cloud_ptr->at(idx).getVector3fMap().template cast<double>();
                // Eigen::Vector3d nearest_pt = Eigen::Vector3d(gicp_target.cloud_ptr->at(indices.front()).x,
                //                                              gicp_target.cloud_ptr->at(indices.front()).y,
                //                                              gicp_target.cloud_ptr->at(indices.front()).z);

                Eigen::Vector3d origin_eigen = origin_pt.getVector3fMap().template cast<double>();//(origin_pt.x, origin_pt.y, origin_pt.z);
                ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<GICPFactor, 1, 4, 3>(
                    new XICP::GICPFactor(origin_eigen, gicp_source.Cs[idx], nearest_pt, gicp_target.Cs[idx]));
                problem.AddResidualBlock(cost_function, loss_function, parameters, parameters+4);
            }
            auto create_time = std::chrono::high_resolution_clock::now(); 

            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.max_num_iterations = 10;
            options.minimizer_progress_to_stdout = false;
            options.check_gradients = false;
            options.gradient_check_relative_precision = 1e-4;
            options.num_threads = thread_nums;

            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);

            T.setIdentity();
            T.block<3, 1>(0, 3) = t_w_curr;
            T.block<3, 3>(0, 0) = q_w_curr.toRotationMatrix();
            std::chrono::duration<float> duration;
            auto end_time = std::chrono::high_resolution_clock::now(); 
            duration = create_time - begin_time;
            if(isDebug) {
                std::cout << "构造用时: "<<(duration).count() << "s";
                duration = end_time - create_time;
                std::cout<<"解算用时: "<<(duration).count() << "s";
                duration = end_time - begin_time;
                std::cout<<"总共用时: "<<(duration).count() << "s"<< std::endl; //输出运行时间
            }
        }

        final_pose = T.cast<float>();
        result_pose = T.cast<float>();
        pcl::transformPointCloud(*gicp_source.cloud_ptr, *transformed_source_ptr, result_pose);
        return true;
    }

}