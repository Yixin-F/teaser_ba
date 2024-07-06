#include <iostream>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/correspondence.h>

// 类型定义
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef pcl::FPFHSignature33 FeatureType;
typedef pcl::PointCloud<FeatureType> FeatureCloud;

void computeFPFH(PointCloud::Ptr cloud, FeatureCloud::Ptr features) {
    // 估算法线
    pcl::NormalEstimation<PointType, pcl::Normal> norm_est;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    norm_est.setInputCloud(cloud);
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>());
    norm_est.setSearchMethod(tree);
    norm_est.setRadiusSearch(0.05);
    norm_est.compute(*normals);

    // 计算FPFH特征
    pcl::FPFHEstimation<PointType, pcl::Normal, FeatureType> fpfh_est;
    fpfh_est.setInputCloud(cloud);
    fpfh_est.setInputNormals(normals);
    fpfh_est.setSearchMethod(tree);
    fpfh_est.setRadiusSearch(0.05);
    fpfh_est.compute(*features);
}

int main(int argc, char** argv) {
    // 加载点云数据
    PointCloud::Ptr cloud_source(new PointCloud);
    PointCloud::Ptr cloud_target(new PointCloud);

    if (pcl::io::loadPCDFile<PointType>("source.pcd", *cloud_source) == -1 ||
        pcl::io::loadPCDFile<PointType>("target.pcd", *cloud_target) == -1) {
        PCL_ERROR("Couldn't read file\n");
        return -1;
    }

    // 计算FPFH特征
    FeatureCloud::Ptr features_source(new FeatureCloud);
    FeatureCloud::Ptr features_target(new FeatureCloud);
    computeFPFH(cloud_source, features_source);
    computeFPFH(cloud_target, features_target);

    // 随机采样一致性先验特征匹配
    pcl::SampleConsensusPrerejective<PointType, PointType, FeatureType> align;
    align.setInputSource(cloud_source);
    align.setSourceFeatures(features_source);
    align.setInputTarget(cloud_target);
    align.setTargetFeatures(features_target);
    align.setMaximumIterations(50000); // 设置最大迭代次数
    align.setNumberOfSamples(3); // 每次随机采样的点数
    align.setCorrespondenceRandomness(5); // 每个点选取的对应点数目
    align.setSimilarityThreshold(0.9f); // 设置相似性阈值
    align.setMaxCorrespondenceDistance(2.5f * 0.01f); // 设置最大对应点距离
    align.setInlierFraction(0.25f); // 设置内点比例

    PointCloud::Ptr cloud_aligned(new PointCloud);
    align.align(*cloud_aligned);

    if (align.hasConverged()) {
        std::cout << "Alignment has converged, score is " << align.getFitnessScore() << std::endl;
        Eigen::Matrix4f transformation = align.getFinalTransformation();
        std::cout << "Transformation matrix:" << std::endl << transformation << std::endl;
    } else {
        PCL_ERROR("Alignment has not converged.\n");
        return -1;
    }

    // 提取内点对应关系
    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
    align.getCorrespondences(*correspondences);

    std::cout << "Found " << correspondences->size() << " inliers." << std::endl;

    return 0;
}
