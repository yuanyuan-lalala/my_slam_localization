#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "eigen_types.h"




using PointType = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointType>;
using CloudPtr = PointCloud::Ptr;
using PointVec = std::vector<PointType,Eigen::aligned_allocator<PointType>>;
using IndexVec = std::vector<int>;


inline Vec3f ToVec3f(const PointType& pt){return pt.getVector3fMap();}
inline Vec3d ToVec3d(const PointType &pt){return pt.getVector3fMap().cast<double>();}

