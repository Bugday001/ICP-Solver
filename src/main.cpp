#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <chrono>
#include "ceres_icp.h"
#include "g2o_icp.h"
#include "gtsam_icp.h"

using namespace std::literals::chrono_literals;

void display_2_pc(const typename pcl::PointCloud<PointType>::Ptr &Cloud1, const typename pcl::PointCloud<PointType>::Ptr &Cloud2,
                  std::string displayname, int display_downsample_ratio);

int main() {
    std::string config_file = "../config/config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file);
    std::string t_str = config_node["target_file"].as<std::string>();
    std::string s_str = config_node["source_file"].as<std::string>();
    std::cout << "load file: " << t_str << ", and " << s_str << std::endl;
    double ds_size = config_node["ds_size"].as<double>();
    std::string method = config_node["method"].as<std::string>();
    CLOUD_PTR cloud_source(new CLOUD());
    CLOUD_PTR cloud_target(new CLOUD());
    if (pcl::io::loadPCDFile<PointType>(s_str, *cloud_source) == -1)
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd\n");
        return (-1);
    }
    if (pcl::io::loadPCDFile<PointType>(t_str, *cloud_target) == -1)
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd\n");
        return (-1);
    }

    XICP::GTSAM_ICP* reg_ptr = new XICP::GTSAM_ICP(config_node[method]);

    //滤波
    std::vector<int> idx;
    pcl::removeNaNFromPointCloud(*cloud_source, *cloud_source, idx);
    pcl::removeNaNFromPointCloud(*cloud_target, *cloud_target, idx);
    pcl::VoxelGrid<PointType> ds_sample;
    ds_sample.setLeafSize(ds_size, ds_size, ds_size);
    ds_sample.setInputCloud(cloud_source);
    ds_sample.filter(*cloud_source);
    ds_sample.setInputCloud(cloud_target);
    ds_sample.filter(*cloud_target);

    Eigen::AngleAxisd r_z(M_PI / 30, Eigen::Vector3d(0, 0, 1));
    Eigen::AngleAxisd r_x(M_PI / 130, Eigen::Vector3d(1, 0, 0));
    Eigen::Matrix4d Tr = Eigen::Matrix4d::Identity();
    Tr.block<3, 3>(0, 0) = (r_x * r_z).matrix();
    Tr.block<3, 1>(0, 3) = Eigen::Vector3d(0.1, -0.3, 0.2);
    pcl::transformPointCloud(*cloud_source, *cloud_target, Tr);
    std::cout << "origi T:\n"
              << Tr << std::endl;

    display_2_pc(cloud_source, cloud_target, "before", 1);
    CLOUD_PTR transformed_source(new CLOUD());
    Eigen::Matrix4f T;
    cout<<"===========START CERES ICP TEST !==========="<<endl;
    //对于slam直接icp匹配，大概10,000是一个较好的数量。依据DLO算法论文
    cout<<"cloud_source points size: "<<cloud_source->size()<<endl;
    
    auto start_time = std::chrono::high_resolution_clock::now(); 
    //GICP
    reg_ptr->setGICPTargetCloud(cloud_target);
    reg_ptr->GICPMatch(cloud_source, Eigen::Matrix4f::Identity(), transformed_source, T);
    auto end_time = std::chrono::high_resolution_clock::now(); 
    std::chrono::duration<float> duration = end_time - start_time;
    std::cout << "构造用时: "<<duration.count() << "s"<<std::endl;

    //icp
    // reg_ptr->setTargetCloud(cloud_target);
    // reg_ptr->scanMatch(cloud_source, Eigen::Matrix4f::Identity(), transformed_source, T);
    std::cout << T << std::endl;
    display_2_pc(transformed_source, cloud_target, "after", 1);

    system("pause");
}

void display_2_pc(const typename pcl::PointCloud<PointType>::Ptr &Cloud1, const typename pcl::PointCloud<PointType>::Ptr &Cloud2,
                  std::string displayname, int display_downsample_ratio)
{

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(displayname));

    viewer->setBackgroundColor(0, 0, 0);
    char t[256];
    std::string s;
    int n = 0;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud2(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (size_t i = 0; i < Cloud1->points.size(); ++i)
    {
        if (i % display_downsample_ratio == 0)
        {
            pcl::PointXYZRGB pt;
            pt.x = Cloud1->points[i].x;
            pt.y = Cloud1->points[i].y;
            pt.z = Cloud1->points[i].z;
            pt.r = 0;
            pt.g = 125;
            pt.b = 255;
            pointcloud1->points.push_back(pt);
        }
    } // Golden

    viewer->addPointCloud(pointcloud1, "pointcloudT");

    for (size_t i = 0; i < Cloud2->points.size(); ++i)
    {
        if (i % display_downsample_ratio == 0)
        {
            pcl::PointXYZRGB pt;
            pt.x = Cloud2->points[i].x;
            pt.y = Cloud2->points[i].y;
            pt.z = Cloud2->points[i].z;
            pt.r = 255;
            pt.g = 123;
            pt.b = 0;
            pointcloud2->points.push_back(pt);
        }
    } // Silver

    viewer->addPointCloud(pointcloud2, "pointcloudS");

    cout << "Click X(close) to continue..." << endl;
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}