#include <pcl/io/pcd_io.h>
#include <pcl/features/fpfh.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d.h>

// 提取FPFH特征
pcl::PointCloud<pcl::FPFHSignature33>::Ptr extractFPFH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.05);
    ne.compute(*normals);

    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(cloud);
    fpfh.setInputNormals(normals);
    fpfh.setSearchMethod(tree);
    fpfh.setRadiusSearch(0.05);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_features(new pcl::PointCloud<pcl::FPFHSignature33>);
    fpfh.compute(*fpfh_features);

    return fpfh_features;
}

// 寻找关联点
pcl::CorrespondencesPtr findCorrespondences(pcl::PointCloud<pcl::FPFHSignature33>::Ptr features1,
                                            pcl::PointCloud<pcl::FPFHSignature33>::Ptr features2) {
    pcl::KdTreeFLANN<pcl::FPFHSignature33> match_search;
    match_search.setInputCloud(features2);

    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);

    for (size_t i = 0; i < features1->size(); ++i) {
        std::vector<int> nn_indices(1);
        std::vector<float> nn_dists(1);
        if (match_search.nearestKSearch(features1->at(i), 1, nn_indices, nn_dists) > 0) {
            pcl::Correspondence corr;
            corr.index_query = static_cast<int>(i);
            corr.index_match = nn_indices[0];
            corr.distance = nn_dists[0];
            correspondences->push_back(corr);
        }
    }

    return correspondences;
}

int main(int argc, char** argv) {
    // 读取点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("cloud1.pcd", *cloud1) == -1 ||
        pcl::io::loadPCDFile<pcl::PointXYZ>("cloud2.pcd", *cloud2) == -1 ||
        pcl::io::loadPCDFile<pcl::PointXYZ>("cloud3.pcd", *cloud3) == -1) {
        PCL_ERROR("Couldn't read file\n");
        return -1;
    }

    // 提取FPFH特征
    auto fpfh1 = extractFPFH(cloud1);
    auto fpfh2 = extractFPFH(cloud2);
    auto fpfh3 = extractFPFH(cloud3);

    // 寻找关联点
    auto corr1_2 = findCorrespondences(fpfh1, fpfh2);
    auto corr2_3 = findCorrespondences(fpfh2, fpfh3);

    // 找出同时关联到点云1、点云2和点云3的点
    std::set<int> corr1_2_indices;
    for (const auto& corr : *corr1_2) {
        corr1_2_indices.insert(corr.index_query);
    }

    std::vector<int> common_indices;
    for (const auto& corr : *corr2_3) {
        if (corr1_2_indices.find(corr.index_query) != corr1_2_indices.end()) {
            common_indices.push_back(corr.index_query);
        }
    }

    // 输出结果
    std::cout << "Number of common points: " << common_indices.size() << std::endl;
    for (const auto& idx : common_indices) {
        std::cout << "Common point index: " << idx << std::endl;
    }

    return 0;
}
