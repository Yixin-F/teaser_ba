#ifndef _UTILITY_H_
#define _UTILITY_H_

#include <atomic>
#include <vector>
#include <random>
#include <unordered_set>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/transforms.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <glog/logging.h>

typedef pcl::PointXYZ PointType;

// std::map<frame_id, std::map<corr_id, observation>>
typedef std::map<int, std::map<int, Eigen::Vector3f>> fea_obs;

#endif _UTILITY_H_