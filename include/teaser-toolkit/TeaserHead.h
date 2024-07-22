#ifndef _TEASER_HEAD_H_
#define _TEASER_HEAD_H_

#include "Utility.h"

#include "../../thirdparty/teaser/matcher.h"
#include "../../thirdparty/teaser/registration.h"

// common
#define NORMAL_ESTIMATION_RADIUS (0.1)
#define MATCH_DISTANCE (0.1)
#define FPFH_SEARCH_RADIUS (0.1)

// fpfh correspondence
#define USE_ABSOLUTE_SCALE (false)
#define CROSS_CHECK (true)
#define TUPLE_TEST (true)
#define TUPLE_SCALE (0.9)

// teaser registeration
#define NOISE_BOUND (0.1)
#define CBAR2 (1.0)
#define ROTATION_MAX_ITERATIONS (20)
#define ROTATION_GNC_FACTOR (1.4)
#define ROTATION_COST_THRESHOLD (0.005)

class fpfh_teaser
{
public:
    fpfh_teaser();
    ~fpfh_teaser() {}

    pcl::VoxelGrid<PointType> downSizeFilterSurf; 
    pcl::KdTreeFLANN<PointType>::Ptr tree;
    pcl::PointCloud<PointType>::Ptr cloud_source;
    pcl::PointCloud<PointType>::Ptr cloud_target;
    pcl::PointCloud<PointType>::Ptr cloud_source_transformed;
    
    void allocateMemory();

    void set_source(const pcl::PointCloud<PointType>::Ptr& cloud);
    void set_target(const pcl::PointCloud<PointType>::Ptr& cloud);
    double calc_matching_error(const Eigen::Matrix4f& transformation);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr extract_fpfh(const pcl::PointCloud<PointType>::Ptr& cloud);
    std::pair<teaser::PointCloud, teaser::FPFHCloud> getFeatureInfo(const pcl::PointCloud<PointType>::Ptr& cloud);
    std::vector<std::map<int, int>> getCorrespondence(teaser::PointCloud& pc1, teaser::PointCloud& pc2,
                                                      teaser::FPFHCloud& fc1, teaser::FPFHCloud& fc2,
                                                      const Eigen::Vector3f& trans1, const Eigen::Vector3f& trans2);
    std::pair<double, Eigen::Matrix4f> match(const bool& v);
};

#endif _TEASER_HEAD_H_