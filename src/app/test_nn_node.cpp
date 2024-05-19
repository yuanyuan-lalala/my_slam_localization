#include"iostream"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include"gtest/gtest.h"
#include "algorithm"
#include "points_proc/bfnn.h"
#include "pcl/io/pcd_io.h"
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>


#include "points_proc/bfnn.h"
#include "points_proc/grid_nn.hpp"

#include "kd_tree/kd_tree.h"
#include "octo_tree/octo_tree.h"
#include "common/point_cloud_utils.h"
#include "common/point_types.h"
#include "common/sys_utils.h"


DEFINE_string(first_scan_path, "./data/ch5/first.pcd", "第一个点云路径");
DEFINE_string(second_scan_path, "./data/ch5/second.pcd", "第二个点云路径");
DEFINE_double(ANN_alpha, 1.0, "AAN的比例因子");

TEST(NN_TEST, BFNN){

    yuan::CloudPtr first(new yuan::PointCloudType),second(new yuan::PointCloudType);
    pcl::io::loadPCDFile(FLAGS_first_scan_path, *first);
    pcl::io::loadPCDFile(FLAGS_second_scan_path, *second);

    if(first->empty()||second->empty()){
        LOG(ERROR)<<"cannot load cloud";
        FAIL();
    }
    
    
    yuan::VoxelGrid(first);
    yuan::VoxelGrid(second);







}
// int main(){






//     return 0;
// }