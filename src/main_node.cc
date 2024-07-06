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
    pcl::PointCloud<PointType>::Ptr cloud4(new pcl::PointCloud<PointType>());
    pcl::io::loadPCDFile("/home/yixin/teaser_ba/src/teaser_ba/test/data/cloud4.pcd", *cloud4);

    downsample.setInputCloud(cloud1);
    downsample.filter(*cloud1);
    LOG(INFO) << "[Cloud] cloud1 size: " << cloud1->points.size();
    downsample.setInputCloud(cloud2);
    downsample.filter(*cloud2);
    LOG(INFO) << "[Cloud] cloud2 size: " << cloud2->points.size();
    downsample.setInputCloud(cloud3);
    downsample.filter(*cloud3);
    LOG(INFO) << "[Cloud] cloud3 size: " << cloud3->points.size();
    downsample.setInputCloud(cloud4);
    downsample.filter(*cloud4);
    LOG(INFO) << "[Cloud] cloud4 size: " << cloud4->points.size();
    
    std::vector<Eigen::Vector3f> all_translation;

    Eigen::Vector3f translation1(-0.4, 0.6, 0.0);
    all_translation.emplace_back(translation1);
    Eigen::Affine3f transform1 = Eigen::Affine3f::Identity();
    transform1.translation() << translation1;
    pcl::transformPointCloud(*cloud1, *cloud1, transform1);
    pcl::io::savePCDFile("/home/yixin/teaser_ba/src/teaser_ba/test/result/cloud1_trans.pcd", *cloud1);
    all_cloud.emplace_back(cloud1);

    Eigen::Vector3f translation2(-0.4, 1.2, 0.0);
    all_translation.emplace_back(translation2);
    Eigen::Affine3f transform2 = Eigen::Affine3f::Identity();
    transform2.translation() << translation2;
    pcl::transformPointCloud(*cloud2, *cloud2, transform2);
    pcl::io::savePCDFile("/home/yixin/teaser_ba/src/teaser_ba/test/result/cloud2_trans.pcd", *cloud2);
    all_cloud.emplace_back(cloud2);

    Eigen::Vector3f translation3(0.8, 1.2, 0.0);
    all_translation.emplace_back(translation3);
    Eigen::Affine3f transform3 = Eigen::Affine3f::Identity();
    transform3.translation() << translation3;
    pcl::transformPointCloud(*cloud3, *cloud3, transform3);
    pcl::io::savePCDFile("/home/yixin/teaser_ba/src/teaser_ba/test/result/cloud3_trans.pcd", *cloud3);
    all_cloud.emplace_back(cloud3);

    Eigen::Vector3f translation4(0.8, 0.6, 0.0);
    all_translation.emplace_back(translation4);
    Eigen::Affine3f transform4 = Eigen::Affine3f::Identity();
    transform4.translation() << translation4;
    pcl::transformPointCloud(*cloud4, *cloud4, transform4);
    pcl::io::savePCDFile("/home/yixin/teaser_ba/src/teaser_ba/test/result/cloud4_trans.pcd", *cloud4);
    all_cloud.emplace_back(cloud4);

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
        auto trans1 = all_translation[i];
        auto trans2 = all_translation[i + 1];

        auto corr = my_teaser.getCorrespondence(point_cloud1, point_cloud2, 
                                                fpfh_cloud1, fpfh_cloud2,
                                                trans1, trans2);
        correspondence.emplace_back(corr);
        LOG(INFO) << "[FPFH features correspondece] between " << i << " and " << i + 1 << " size: " << corr.size();
    }

    // ---------------------------------- co-visibility with 4 frames -------------------------------------------
    // std::vector<std::vector<std::pair<frame_id, feature_id>>>
    std::vector<std::vector<std::pair<int, int>>> fusion_pair;

    for (int ii = 0; ii < correspondence.size(); ii++) {
        std::vector<std::map<int, int>> curr_corr = correspondence[ii];

        // // init
        // if(ii == 0) {
        //     for (auto corr : curr_corr) {
        //         std::vector<std::pair<int, int>> vec0_pair{std::make_pair(ii, corr.begin()->first),
        //                                                    std::make_pair(ii + 1, corr.begin()->second)};
        //         fusion_pair.emplace_back(vec0_pair);
        //     }
        //     continue;
        // }   

        std::vector<std::vector<std::pair<int, int>>> fusion_pair_tmp;

        // loop
        for (auto& corr : curr_corr) {
            int front = corr.begin()->first;
            bool find_flag = false;
            // #pragma omp parallel for
            for (int kk = 0; kk < fusion_pair.size(); kk++) {
                if (fusion_pair[kk].back().first == ii && fusion_pair[kk].back().second == front) {
                    fusion_pair[kk].emplace_back(std::make_pair(ii + 1, corr.begin()->second));
                    find_flag = true;
                    continue;
                }
            }
            if (!find_flag) {
                std::vector<std::pair<int, int>> veckk_pair{std::make_pair(ii, front),
                                                            std::make_pair(ii + 1, corr.begin()->second)};
                fusion_pair_tmp.emplace_back(veckk_pair);
            }
        }

        for (auto& tmp : fusion_pair_tmp) {
            fusion_pair.emplace_back(tmp);
        }
    }
    LOG(INFO) << "[covisibility set] size: " << fusion_pair.size();

    std::map<int, std::map<int, Eigen::Vector3f>> covisibility;
    pcl::PointCloud<pcl::PointXYZ>::Ptr coobs(new pcl::PointCloud<pcl::PointXYZ>());

    for (int jj = 0; jj < fusion_pair.size(); jj++) {
        if (fusion_pair[jj].size() < 4) {
            continue;
        }
        std::map<int, Eigen::Vector3f> co_tmp;
        for (auto& co : fusion_pair[jj]) {
            Eigen::Vector3f obs;
            obs << all_features[co.first].first[co.second].x, 
                   all_features[co.first].first[co.second].y,
                   all_features[co.first].first[co.second].z;
            co_tmp.insert(std::make_pair(co.first, obs));
            pcl::PointXYZ pt;
            pt.x = all_features[co.first].first[co.second].x;
            pt.y = all_features[co.first].first[co.second].y;
            pt.z = all_features[co.first].first[co.second].z;
            coobs->points.emplace_back(pt);
        }
        covisibility.insert(std::make_pair(jj, co_tmp));
    }

    coobs->height = 1;
    coobs->width = coobs->points.size();
    pcl::io::savePCDFile("/home/yixin/teaser_ba/src/teaser_ba/test/result/co4obs_bt1234.pcd", *coobs);
    LOG(INFO) << "[covisibility map] size: " << covisibility.size();





    // // test
    // auto corr = my_teaser.getCorrespondence(all_features[2].first, all_features[3].first, 
    //                                         all_features[2].second, all_features[3].second,
    //                                         translation3, translation4);
    // LOG(INFO) << "[FPFH features correspondece (test)] size: " << corr.size();

    // pcl::PointCloud<pcl::PointXYZ>::Ptr coobs_1(new pcl::PointCloud<pcl::PointXYZ>());
    // pcl::PointCloud<pcl::PointXYZ>::Ptr coobs_3(new pcl::PointCloud<pcl::PointXYZ>());
    // for (auto cor : corr) {
    //     pcl::PointXYZ pt_1;
    //     pt_1.x = all_features[2].first[cor.begin()->first].x;
    //     pt_1.y = all_features[2].first[cor.begin()->first].y;
    //     pt_1.z = all_features[2].first[cor.begin()->first].z;
    //     coobs_1->points.emplace_back(pt_1);

    //     pcl::PointXYZ pt_2;
    //     pt_2.x = all_features[3].first[cor.begin()->second].x;
    //     pt_2.y = all_features[3].first[cor.begin()->second].y;
    //     pt_2.z = all_features[3].first[cor.begin()->second].z;
    //     coobs_3->points.emplace_back(pt_2);
    // }
    // coobs_1->height = 1;
    // coobs_1->width = coobs_1->points.size();
    // pcl::io::savePCDFile("/home/yixin/teaser_ba/src/teaser_ba/test/result/coobs_1.pcd", *coobs_1);
    // coobs_3->height = 1;
    // coobs_3->width = coobs_3->points.size();
    // pcl::io::savePCDFile("/home/yixin/teaser_ba/src/teaser_ba/test/result/coobs_3.pcd", *coobs_3);

    return 0;
}