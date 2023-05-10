#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/registration/PointMatcher.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
    // 假设你已经安装了pcl和gtsam，并且包含了相应的头文件
#include <pcl/kdtree/kdtree_flann.h>

using namespace gtsam;

// Define a custom factor to compute the alignment error between two point clouds
class ICPFactor : public NoiseModelFactor2<Pose3, Pose3>
{
private:
    PointMatcher<Point3> matcher_;                        // A point matcher object that performs the nearest neighbor search
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_, target_; // The source and target point clouds

public:
    // Constructor
    ICPFactor(const SharedNoiseModel &model, Key key1, Key key2,
              const pcl::PointCloud<pcl::PointXYZ>::Ptr &source, const pcl::PointCloud<pcl::PointXYZ>::Ptr &target) : NoiseModelFactor2<Pose3, Pose3>(model, key1, key2),
                                                                                                                      source_(source), target_(target)
    {
        // Initialize the point matcher with a default parameter
        matcher_ = PointMatcher<Point3>();
    }

    // The error function that computes the alignment error
    Vector evaluateError(const Pose3 &pose1, const Pose3 &pose2,
                         boost::optional<Matrix &> H1 = boost::none,
                         boost::optional<Matrix &> H2 = boost::none) const override
    {
        // Transform the source point cloud by the relative pose between the two poses
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_source(new pcl::PointCloud<pcl::PointXYZ>());
        Pose3 relative_pose = pose1.between(pose2);
        for (const auto &point : *source_)
        {
            Point3 p(point.x, point.y, point.z);
            p = relative_pose.transform_from(p);
            transformed_source->push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
        }

        // Convert the target point cloud to a vector of Point3
        Point3Vector target_vector;
        for (const auto &point : *target_)
        {
            target_vector.push_back(Point3(point.x, point.y, point.z));
        }

        // Find the correspondences between the transformed source and the target using the point matcher
        auto correspondences = matcher_.findCorrespondences(transformed_source, target_vector);

        // Compute the alignment error as the sum of squared distances between the correspondences
        double error = 0.0;
        for (const auto &c : correspondences)
        {
            error += c.distance * c.distance;
        }

        // Optionally compute the Jacobians using numerical differentiation
        if (H1)
        {
            *H1 = numericalDerivative21<Vector, Pose3, Pose3>(
                boost::bind(&ICPFactor::evaluateError, this, _1, _2, boost::none, boost::none), pose1, pose2);
        }
        if (H2)
        {
            *H2 = numericalDerivative22<Vector, Pose3, Pose3>(
                boost::bind(&ICPFactor::evaluateError, this, _1, _2, boost::none, boost::none), pose1, pose2);
        }

        return Vector1(error);
    }
};

int main()
{
    // Assume we have two point clouds source and target
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>()); // The source point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>()); // The target point cloud

    // Load the point clouds from files or other sources
    ...

        // Create a factor graph to optimize the relative pose between the two point clouds
        NonlinearFactorGraph graph;

    // Add a prior factor on the first pose with a large covariance
    noiseModel::Diagonal::shared_ptr priorModel = noiseModel::Diagonal::Sigmas(Vector6(1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0));
    graph.add(PriorFactor<Pose3>(1, Pose3(), priorModel));

    // Add an ICP factor between the two poses with a small covariance
    noiseModel::Diagonal::shared_ptr icpModel = noiseModel::Diagonal::Sigmas(Vector1(0.01));
    graph.add(ICPFactor(icpModel, 1, 2, source, target));

    // Create initial values for the two poses
    Values initial;
    initial.insert(1, Pose3());                              // The first pose is fixed by the prior
    initial.insert(2, Pose3(Rot3(), Point3(1.0, 0.0, 0.0))); // The second pose is initialized with some translation

    // Optimize the graph using Levenberg-Marquardt algorithm
    LevenbergMarquardtOptimizer optimizer(graph, initial);
    Values result = optimizer.optimize();

    // Print the optimized relative pose
    Pose3 relative_pose = result.at<Pose3>(1).between(result.at<Pose3>(2));
    std::cout << "The relative pose is: " << relative_pose << std::endl;

    return 0;
}

void pclKdtree()
{
    // 定义点云类型
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    // 定义变换矩阵类型
    typedef gtsam::Pose3 Pose;

    // 定义噪声模型
    const gtsam::noiseModel::Diagonal::shared_ptr priorNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01).finished()); // 假设先验噪声很小
    const gtsam::noiseModel::Diagonal::shared_ptr betweenNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished());     // 假设两个点云之间的变换噪声较大

    // 定义两个点云source和target
    PointCloud::Ptr source(new PointCloud);
    PointCloud::Ptr target(new PointCloud);

    // 假设你已经给source和target赋值了

    // 创建kdtree对象
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(target);

    // 创建因子图对象
    gtsam::NonlinearFactorGraph graph;

    // 添加先验因子，假设source的初始位姿为单位矩阵
    graph.add(gtsam::PriorFactor<Pose>(1, Pose(), priorNoise));

    // 遍历source中的每个点，找到target中最近的点，添加两点之间的距离作为观测值，添加两个点云之间的变换作为待估计变量
    for (int i = 0; i < source->size(); i++)
    {
        // 获取source中的第i个点
        PointT p1 = source->points[i];

        // 在target中找到最近的点
        std::vector<int> indices;
        std::vector<float> dists;
        kdtree.nearestKSearch(p1, 1, indices, dists);

        // 获取target中的最近点
        PointT p2 = target->points[indices[0]];

        // 计算两点之间的距离
        double d = sqrt(dists[0]);

        // 添加观测值因子，假设两个点云之间的变换为2号变量
        graph.add(gtsam::BetweenFactor<Pose>(1, 2, Pose(d, 0, 0, 0, 0, 0), betweenNoise));
    }

    // 创建初始值对象，假设两个点云之间的初始变换为单位矩阵
    gtsam::Values initial;
    initial.insert(1, Pose());
    initial.insert(2, Pose());

    // 创建优化器对象，使用LM算法进行优化
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial);

    // 进行优化，获取最优解
    gtsam::Values result = optimizer.optimize();

    // 打印最优解，即两个点云之间的最佳变换矩阵
    result.at<Pose>(2).print("Transformation:\n");
}