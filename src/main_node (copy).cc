#include <iostream>
#include <vector>

#include <ros/ros.h>

#include "teaser-toolkit/TeaserHead.h"

int main (int argc, char* argv[])
{
    // google::InitGoogleLogging(argv[0]);
    fpfh_teaser my_teaser;
    LOG(INFO) << "Hi, I'm Teaser++ ...";

    // std::vector<cloud>
    std::vector<pcl::PointCloud<PointType>::Ptr> all_cloud;

    // downsampling
    pcl::VoxelGrid<pcl::PointXYZ> downsample;
    downsample.setLeafSize(0.01, 0.01, 0.01);

    pcl::PointCloud<PointType>::Ptr cloud1(new pcl::PointCloud<PointType>());
    pcl::io::loadPCDFile("/home/yixin/teaser_ba/src/teaser_ba/test/data/cloud1.pcd", *cloud1);
    pcl::PointCloud<PointType>::Ptr cloud2(new pcl::PointCloud<PointType>());
    pcl::io::loadPCDFile("/home/yixin/teaser_ba/src/teaser_ba/test/data/cloud2.pcd", *cloud2);
    pcl::PointCloud<PointType>::Ptr cloud3(new pcl::PointCloud<PointType>());
    pcl::io::loadPCDFile("/home/yixin/teaser_ba/src/teaser_ba/test/data/cloud3.pcd", *cloud3);

    downsample.setInputCloud(cloud1);
    downsample.filter(*cloud1);
    LOG(INFO) << "[Cloud] cloud1 size: " << cloud1->points.size();
    downsample.setInputCloud(cloud2);
    downsample.filter(*cloud2);
    LOG(INFO) << "[Cloud] cloud2 size: " << cloud2->points.size();
    downsample.setInputCloud(cloud3);
    downsample.filter(*cloud3);
    LOG(INFO) << "[Cloud] cloud3 size: " << cloud3->points.size();

    all_cloud.emplace_back(cloud1);
    all_cloud.emplace_back(cloud2);
    all_cloud.emplace_back(cloud3);


    // std::vector<std::pair<pointcloud, fpfhclouds>>
    std::vector<std::pair<teaser::PointCloud, teaser::FPFHCloud>> all_features;
    
    // get feature
    for (auto& cloud : all_cloud) {
        auto feature_pair = my_teaser.getFeatureInfo(cloud);
        all_features.emplace_back(feature_pair);
        LOG(INFO) << "[FPFH features extraction] size: " << feature_pair.second.size();
    }

    
    // std::vector<std::vector<std::pair<feature_id, feature_id>>>
    std::vector<std::vector<std::map<int, int>>> correspondence;

    // get covisibility
    for (int i = 0; i < all_features.size() - 1; i++) {
        auto point_cloud1 = all_features[i].first;
        auto fpfh_cloud1 = all_features[i].second;
        auto point_cloud2 = all_features[i + 1].first;
        auto fpfh_cloud2 = all_features[i + 1].second;

        auto corr = my_teaser.getCorrespondence(point_cloud1, point_cloud2, fpfh_cloud1, fpfh_cloud2);
        correspondence.emplace_back(corr);
        LOG(INFO) << "[FPFH features correspondece] between " << i << " and " << i + 1 << " size: " << corr.size();
    }

    // std::vector<std::vector<std::pair<frame_id, feature_id>>>
    std::vector<std::vector<std::pair<int, int>>> fusion_pair;

    std::vector<std::vector<int>> fusion;

    for (int ii = 0; ii < correspondence.size(); ii++) {
        std::vector<std::map<int, int>> curr_corr = correspondence[ii];

        // init
        if(ii == 0) {
            for (auto corr : curr_corr) {
                std::vector<int> vec0{corr.begin()->first, corr.begin()->second};
                // std::cout << corr.begin()->first << " " << corr.begin()->second << std::endl;
                fusion.emplace_back(vec0);
                std::vector<std::pair<int, int>> vec0_pair{std::make_pair(ii, corr.begin()->first),
                                                           std::make_pair(ii + 1, corr.begin()->second)};
                fusion_pair.emplace_back(vec0_pair);
            }
            continue;
        }   

        std::vector<std::vector<int>> fusion_tmp;
        std::vector<std::vector<std::pair<int, int>>> fusion_pair_tmp;

        // loop
        for (auto& corr : curr_corr) {
            int front = corr.begin()->first;
            // #pragma omp parallel for
            for (int kk = 0; kk < fusion.size(); kk++) {
                if(fusion[kk].back() == front) {
                    // std::cout << front << " " << fusion[kk].back() << std::endl;
                    fusion[kk].emplace_back(corr.begin()->second);
                    fusion_pair[kk].emplace_back(std::make_pair(ii + 1, corr.begin()->second));
                }
                else {
                    // std::cout << "add " << std::endl;
                    std::vector<int> veckk{front, corr.begin()->second};
                    fusion_tmp.emplace_back(veckk);
                    std::vector<std::pair<int, int>> veckk_pair{std::make_pair(ii, front),
                                                                std::make_pair(ii + 1, corr.begin()->second)};
                    fusion_pair_tmp.emplace_back(veckk_pair);
                }
            }
        }

        for (auto& tmp : fusion_tmp) fusion.emplace_back(tmp);
        for (auto& tmp : fusion_pair_tmp) fusion_pair.emplace_back(tmp);
    }
    assert(fusion_pair.size() == fusion.size());
    LOG(INFO) << "[covisibility set] size: " << fusion_pair.size();

    std::map<int, std::map<int, Eigen::Vector3f>> covisibility;

    for (int jj = 0; jj < fusion_pair.size(); jj++) {
        std::map<int, Eigen::Vector3f> co_tmp;
        for (auto& co : fusion_pair[jj]) {
            Eigen::Vector3f obs;
            obs << all_features[co.first].first[co.second].x, 
                   all_features[co.first].first[co.second].y,
                   all_features[co.first].first[co.second].z;
            co_tmp.insert(std::make_pair(co.first, obs));
        }
        covisibility.insert(std::make_pair(jj, co_tmp));
    }
    LOG(INFO) << "[covisibility map] size: " << covisibility.size();

    // std::map<frame_id, std::map<feature_id, std::map<frame_id, observation>>>
    // std::map<int, std::map<int, fea_obs>> covisibility;

    // for (int i = 0; i < all_features.size() - 1; i++) {
    //     auto point_cloud1 = all_features[i].first;
    //     auto fpfh_cloud1 = all_features[i].second;
    //     auto point_cloud2 = all_features[i + 1].first;
    //     auto fpfh_cloud2 = all_features[i + 1].second;

    //     auto corr = my_teaser.getCorrespondence(point_cloud1, point_cloud2, fpfh_cloud1, fpfh_cloud2);

    //     for (auto c : corr) {
    //         Eigen::Vector3f obs1;
    //         obs1 << point_cloud1[c.first].x, point_cloud1[c.first].y, point_cloud1[c.first].z;
    //         covisibility.at(i).at(c.first).at(i + 1).at(c.second) = obs1;

    //         Eigen::Vector3f obs2;
    //         obs2 << point_cloud2[c.second].x, point_cloud2[c.second].y, point_cloud2[c.second].z;
    //         covisibility.at(i + 1).at(c.second).at(i).at(c.first) = obs2;
    //     }
    // }

    // for (auto& frame_fea : covisibility) {
    //     auto frame_id = frame_fea.first;
    //     auto fea_info = frame_fea.second;
    // }
    


    return 0;
}