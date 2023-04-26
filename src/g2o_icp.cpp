#include "g2o_icp.h"
#include "lidarG2O.h"


namespace XICP
{
    G2O_ICP::G2O_ICP(const YAML::Node &node)
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

    G2O_ICP::~G2O_ICP()
    {
    }

    bool G2O_ICP::setGICPTargetCloud(const CLOUD_PTR &target)
    {
        gicp_target.setCloudPtr(target);
    }

    /**
     * GICP
    */
    bool G2O_ICP::GICPMatch(const CLOUD_PTR &source, const Eigen::Matrix4f &predict_pose,
                         CLOUD_PTR &transformed_source_ptr, Eigen::Matrix4f &result_pose)
    {   
        CLOUD_PTR transform_cloud(new CLOUD(*source));
        gicp_source.setCloudPtr(transform_cloud);
        Eigen::Matrix4d T = predict_pose.cast<double>();

        Eigen::Matrix3d R = Eigen::Matrix<double, 3, 3>::Identity();
        Eigen::Vector3d t(0,0,0);
        Sophus::SE3d SE3_Rt(R,t);

        for (int i = 0; i < max_iterations; ++i)
        {
            if(isDebug) std::cout<<"i: "<<i<<std::endl;
            pcl::transformPointCloud(*gicp_source.cloud_ptr, *transform_cloud, T);  //左乘
            gicp_source.computeCov();
            // 构建图优化，先设定g2o
            // 每个误差项优化变量维度为6，误差值维度为3
            typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType; 
            // 线性求解器类型
            typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

            // 梯度下降方法，可以从GN, LM, DogLeg 中选
            auto solver = new g2o::OptimizationAlgorithmLevenberg(
                g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
            g2o::SparseOptimizer optimizer;     // 图模型
            optimizer.setAlgorithm(solver);   // 设置求解器
            optimizer.setVerbose(isDebug);       // 打开调试输出，chi表示误差平方

            //加顶点
            // GICPVertex *v1 = new GICPVertex();
            // v1->setEstimate(Sophus::SE3d());
            // v1->setId(0);
            // // v->setFixed(false);
            // optimizer.addVertex(v1);

            // //fixed顶点
            // GICPVertex *v2 = new GICPVertex();
            // v2->setEstimate(SE3_Rt);
            // v2->setId(1);
            // v2->setFixed(true);
            // optimizer.addVertex(v2);

            // g2o::VertexSE3Expmap *v3 = new g2o::VertexSE3Expmap();
            // v3->setEstimate(g2o::SE3Quat(
            //     Eigen::Matrix3d::Identity(),
            //     Eigen::Vector3d( 0,0,0 )));
            // v3->setId(2);
            // optimizer.addVertex(v3);

            g2o::VertexSE3 *v4 = new g2o::VertexSE3;
            v4->setEstimate(Eigen::Isometry3d::Identity());
            v4->setId(3);
            optimizer.addVertex(v4);

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
                const Eigen::Vector3d nearest_pt = gicp_target.cloud_ptr->at(idx).getVector3fMap().template cast<double>();

                Eigen::Vector3d origin_eigen = transform_pt.getVector3fMap().template cast<double>();//(origin_pt.x, origin_pt.y, origin_pt.z);
                
                double* init_data = origin_eigen.data();
                double* tmp = init_data+3;
                tmp = gicp_source.Cs[idx].data();
                tmp += 9;
                tmp = gicp_target.Cs[idx].data();
                //加边
                g2oSE3GICPEdge *edge = new g2oSE3GICPEdge(init_data);
                edge->setId(j);
                edge->setVertex(0, v4);                // 设置连接的顶点
                // edge->setVertex(1, v2);                // 设置连接的顶点
                edge->setMeasurement(nearest_pt);      // 观测数值
                edge->setInformation(Eigen::Matrix<double, 3, 3>::Identity()); // 信息矩阵：协方差矩阵之逆
                optimizer.addEdge(edge);
            }
            // 执行优化
            optimizer.initializeOptimization();
            optimizer.optimize(max_opt);
            // 输出优化值
            T = Eigen::Isometry3d(v4->estimate()).matrix()* T;
        }

        final_pose = T.cast<float>();
        result_pose = T.cast<float>();
        pcl::transformPointCloud(*gicp_source.cloud_ptr, *transformed_source_ptr, result_pose);
        return true;
    }
}