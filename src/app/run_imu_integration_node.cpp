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
    
    std::vector<int> num = {1,5,32,6,8,2};    
    auto min_ele = std::min_element(num.begin(), num.end());
     if (min_ele != num.end()) {
        std::cout << "Min element: " << *min_ele << std::endl;
    }
    *(std::back_inserter(num)) = 100;
    std::cout<<"num: ";
    for(auto i : num){

        std::cout<<i<<std::endl;
    }
    // std::accumulate(InputIterator first, InputIterator last, Tp init, BinaryOperation binary_op)
    std::vector<int> nums_for_each(10);
    std::for_each( nums_for_each.begin(), nums_for_each.end(), 
    [idx=0,&nums_for_each](size_t i)
    mutable { i = idx++;
    nums_for_each[i] = i;});
    
    
    std::cout<<"nums_for_each:"<<std::endl;
    for(auto i:nums_for_each){

        std::cout<<i;
    }
    std::cout<<std::endl;



    return 0;
}
