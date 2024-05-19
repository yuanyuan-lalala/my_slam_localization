//
// Created by xiang on 2021/8/6.
//

// #include <gflags/gflags.h>

#include "Eigen/Core"
#include "Eigen/Dense"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include"vector"
#include "algorithm"
#include <execution>
using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr = PointCloudType::Ptr;
using Vec3f = Eigen::Vector3f;

int bfnn_point( CloudPtr cloud, const Vec3f& point) {
    return std::min_element(cloud->points.begin(), cloud->points.end(),
                            [&point](const PointType& pt1, const PointType& pt2) -> bool {
                                return (pt1.getVector3fMap() - point).squaredNorm() <
                                       (pt2.getVector3fMap() - point).squaredNorm();
                            }) -
           cloud->points.begin();
}

// DEFINE_string(pcd_path, "/home/yuanyuan/my_slam_in_autonomous_driving/data/first.pcd", "点云文件路径");
// std::string pcd_path =  "/home/yuanyuan/my_slam_in_autonomous_driving/data/first.pcd";
/// 本程序可用于显示单个点云，演示PCL的基本用法
/// 实际上就是调用了pcl的可视化库，类似于pcl_viewer
 


int main(int argc, char** argv) {
    


    return 0;
}
