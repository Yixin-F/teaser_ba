#include <iostream>
#include <vector>

#include <ros/ros.h>

#include "teaser-toolkit/TeaserHead.h"

bool check(const std::vector<std::pair<int, int>>& vec1, const std::vector<std::pair<int, int>>& vec2) {
    std::unordered_set<std::pair<int, int>, boost::hash<std::pair<int, int>>> elementsSet;

    for (const auto& elem : vec1) {
        elementsSet.insert(elem);
    }

    for (const auto& elem : vec2) {
        if (elementsSet.find(elem) != elementsSet.end()) {
            return false; 
        }
    }

    return true; 
}

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
    
    // TODO: get feature
    std::vector<std::pair<teaser::PointCloud, teaser::FPFHCloud>> all_features;
    for (auto& cloud : all_cloud) {
        auto feature_pair = my_teaser.getFeatureInfo(cloud);
        all_features.emplace_back(feature_pair);
        LOG(INFO) << "[FPFH features extraction] size: " << feature_pair.second.size();
    }

    // TODO: get correspondence
    std::map<float, std::vector<std::map<int, int>>> all_correspondence;
    auto correspondence_bt12 = my_teaser.getCorrespondence(all_features[0].first, all_features[1].first, 
                                                           all_features[0].second, all_features[1].second,
                                                           all_translation[0], all_translation[1]);
    all_correspondence.insert(std::make_pair(0.01, correspondence_bt12));
    LOG(INFO) << "[FPFH features correspondece] between " << 1 << " and " << 2 << " size: " << correspondence_bt12.size();
    auto correspondence_bt13 = my_teaser.getCorrespondence(all_features[0].first, all_features[2].first, 
                                                           all_features[0].second, all_features[2].second,
                                                           all_translation[0], all_translation[2]);
    all_correspondence.insert(std::make_pair(0.02, correspondence_bt13));
    LOG(INFO) << "[FPFH features correspondece] between " << 1 << " and " << 3 << " size: " << correspondence_bt13.size();
    auto correspondence_bt14 = my_teaser.getCorrespondence(all_features[0].first, all_features[3].first, 
                                                           all_features[0].second, all_features[3].second,
                                                           all_translation[0], all_translation[3]);
    all_correspondence.insert(std::make_pair(0.03, correspondence_bt14));
    LOG(INFO) << "[FPFH features correspondece] between " << 1 << " and " << 4 << " size: " << correspondence_bt14.size();
    auto correspondence_bt23 = my_teaser.getCorrespondence(all_features[1].first, all_features[2].first, 
                                                           all_features[1].second, all_features[2].second,
                                                           all_translation[1], all_translation[2]);
    all_correspondence.insert(std::make_pair(1.02, correspondence_bt23));
    LOG(INFO) << "[FPFH features correspondece] between " << 2 << " and " << 3 << " size: " << correspondence_bt23.size();
    auto correspondence_bt24 = my_teaser.getCorrespondence(all_features[1].first, all_features[3].first, 
                                                           all_features[1].second, all_features[3].second,
                                                           all_translation[1], all_translation[3]);
    all_correspondence.insert(std::make_pair(1.03, correspondence_bt24));
    LOG(INFO) << "[FPFH features correspondece] between " << 2 << " and " << 4 << " size: " << correspondence_bt24.size();
    auto correspondence_bt34 = my_teaser.getCorrespondence(all_features[2].first, all_features[3].first, 
                                                           all_features[2].second, all_features[3].second,
                                                           all_translation[2], all_translation[3]);
    all_correspondence.insert(std::make_pair(2.03, correspondence_bt34));
    LOG(INFO) << "[FPFH features correspondece] between " << 3 << " and " << 4 << " size: " << correspondence_bt34.size();

    // TODO: get group
    std::vector<std::vector<float>> group_use2 {{0.01}, {0.02}, {0.03}, {1.02}, {1.03}, {2.03}};  // two frames
    std::vector<std::vector<float>> group_use3 {{0.01, 1.02}, {0.01, 1.03}, {0.02, 2.03}, {1.02, 2.03}}; // three frames
    std::vector<std::vector<float>> group_use4 {{0.01, 1.02, 2.03}}; // four frames

    // TODO: data association
    // ----------------------- co-visibility with two frames ---------------------------
    std::vector<std::vector<std::pair<int, int>>> association_use2;
    for (auto& group : group_use2) {
        for (auto& bt : group) {
            auto corr_feas = all_correspondence[bt];
            for (auto& cor : corr_feas) {
                std::vector<std::pair<int, int>> fusion{std::make_pair(std::floor(bt), cor.begin()->first),
                                                        std::make_pair(std::round((bt - std::floor(bt)) * 100.0), cor.begin()->second)};
                association_use2.emplace_back(fusion);
            }
            
        }
    }
    LOG(INFO) << "[data association with two frames] size: " << association_use2.size();

    // ----------------------- co-visibility with three frames ---------------------------
    std::vector<std::vector<std::pair<int, int>>> association_use3;
    for (auto& group : group_use3) {
        std::vector<std::vector<std::pair<int, int>>> association_use3_tmp;
        for (int ii = 0; ii < group.size(); ii++) {
            auto corr_feas = all_correspondence[group[ii]];
            std::vector<std::vector<std::pair<int, int>>> association_use3_tmp_tmp;
            int front = std::floor(group[ii]);
            int end = std::round((group[ii] - std::floor(group[ii])) * 100.0);
            // std::cout << group[ii] << " " << front << " " << end << std::endl;
            // std::cout << association_use3_tmp.size() << std::endl;
            for (auto& cor : corr_feas) {
                bool find_flag = false;
                for (auto& ass : association_use3_tmp) {
                    // std::cout << ass.back().first << " " << front << " " << ass.back().second << " " << cor.begin()->first << std::endl;
                    if (ass.back().first == front && ass.back().second == cor.begin()->first) {
                        ass.emplace_back(std::make_pair(end, cor.begin()->second));
                        // std::cout << "add" << std::endl;
                        find_flag = true;
                        continue;
                    }
                }
                if (!find_flag) {
                    std::vector<std::pair<int, int>> ass{std::make_pair(front, cor.begin()->first),
                                                         std::make_pair(end, cor.begin()->second)};
                    association_use3_tmp_tmp.emplace_back(ass);
                }
            }
            for (auto& tmp_tmp : association_use3_tmp_tmp) {
                association_use3_tmp.emplace_back(tmp_tmp);
            }    
        }
        for (auto& tmp : association_use3_tmp) {
            if (tmp.size() < 3) {
                continue;
            }
            association_use3.emplace_back(tmp);
        }
    }
    LOG(INFO) << "[data association with three frames] size: " << association_use3.size();

    // ----------------------- co-visibility with four frames ---------------------------
    std::vector<std::vector<std::pair<int, int>>> association_use4;
    for (auto& group : group_use4) {
        std::vector<std::vector<std::pair<int, int>>> association_use4_tmp;
        for (int ii = 0; ii < group.size(); ii++) {
            auto corr_feas = all_correspondence[group[ii]];
            std::vector<std::vector<std::pair<int, int>>> association_use4_tmp_tmp;
            int front = std::floor(group[ii]);
            int end = std::round((group[ii] - std::floor(group[ii])) * 100.0);
            for (auto& cor : corr_feas) {
                bool find_flag = false;
                for (auto& ass : association_use4_tmp) {
                    if (ass.back().first == front && ass.back().second == cor.begin()->first) {
                        ass.emplace_back(std::make_pair(end, cor.begin()->second));
                        find_flag = true;
                        continue;
                    }
                }
                if (!find_flag) {
                    std::vector<std::pair<int, int>> ass{std::make_pair(front, cor.begin()->first),
                                                         std::make_pair(end, cor.begin()->second)};
                    association_use4_tmp_tmp.emplace_back(ass);
                }
            }
            for (auto& tmp_tmp : association_use4_tmp_tmp) {
                association_use4_tmp.emplace_back(tmp_tmp);
            }    
        }
        for (auto& tmp : association_use4_tmp) {
            if (tmp.size() < 4) {
                continue;
            }
            association_use4.emplace_back(tmp);
        }
    }
    LOG(INFO) << "[data association with four frames] size: " << association_use4.size();


    // TODO: data refinement
    // ----------------------------- refine co-visibility with two frames --------------------------------
    std::vector<std::vector<std::pair<int, int>>> association_use2_new;
    for (auto& ass2 : association_use2) {
        bool flag = true;
        for (auto& ass3 : association_use3) {
            if (!check(ass2, ass3)) {
                flag = false;
                break;
            }
        }
        if (flag) {
            association_use2_new.emplace_back(ass2);
        }
    }
    association_use2.swap(association_use2_new);
    LOG(INFO) << "[data association with two frames] refined size: " << association_use2.size();

    // ----------------------------- refine co-visibility with three frames --------------------------------
    std::vector<std::vector<std::pair<int, int>>> association_use3_new;
    for (auto& ass3 : association_use3) {
        bool flag = true;
        for (auto& ass4 : association_use4) {
            if (!check(ass3, ass4)) {
                flag = false;
                break;
            }
        }
        if (flag) {
            association_use3_new.emplace_back(ass3);
        }
    }
    association_use3.swap(association_use3_new);
    LOG(INFO) << "[data association with three frames] refined size: " << association_use3.size();

    // ----------------------------- refine co-visibility with three frames --------------------------------
    LOG(INFO) << "[data association with four frames] refined size: " << association_use4.size();


    // TODO: get covisibility
    // ----------------------------- co-visibility with two frames --------------------------------
    std::map<int, std::map<int, Eigen::Vector3f>> covisibility_use2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr coobs_use2(new pcl::PointCloud<pcl::PointXYZ>());

    for (int jj = 0; jj < association_use2.size(); jj++) {
        std::map<int, Eigen::Vector3f> co_tmp;
        for (auto& co : association_use2[jj]) {
            // std::cout << co.first << " " << co.second << std::endl;
            Eigen::Vector3f obs;
            obs << all_features[co.first].first[co.second].x, 
                   all_features[co.first].first[co.second].y,
                   all_features[co.first].first[co.second].z;
            co_tmp.insert(std::make_pair(co.first, obs));
            std::cout << obs << std::endl;
            pcl::PointXYZ pt;
            pt.x = all_features[co.first].first[co.second].x;
            pt.y = all_features[co.first].first[co.second].y;
            pt.z = all_features[co.first].first[co.second].z;
            coobs_use2->points.emplace_back(pt);
        }
        covisibility_use2.insert(std::make_pair(jj, co_tmp));
    }
    coobs_use2->height = 1;
    coobs_use2->width = coobs_use2->points.size();
    pcl::io::savePCDFile("/home/yixin/teaser_ba/src/teaser_ba/test/result/co2obs.pcd", *coobs_use2);
    LOG(INFO) << "[covisibility map use two frames] size: " << covisibility_use2.size();

    // ----------------------------- co-visibility with three frames --------------------------------
    std::map<int, std::map<int, Eigen::Vector3f>> covisibility_use3;
    pcl::PointCloud<pcl::PointXYZ>::Ptr coobs_use3(new pcl::PointCloud<pcl::PointXYZ>());

    for (int jj = 0; jj < association_use3.size(); jj++) {
        std::map<int, Eigen::Vector3f> co_tmp;
        for (auto& co : association_use3[jj]) {
            // std::cout << co.first << " " << co.second << std::endl;
            Eigen::Vector3f obs;
            obs << all_features[co.first].first[co.second].x, 
                   all_features[co.first].first[co.second].y,
                   all_features[co.first].first[co.second].z;
            co_tmp.insert(std::make_pair(co.first, obs));
            pcl::PointXYZ pt;
            pt.x = all_features[co.first].first[co.second].x;
            pt.y = all_features[co.first].first[co.second].y;
            pt.z = all_features[co.first].first[co.second].z;
            coobs_use3->points.emplace_back(pt);
        }
        covisibility_use3.insert(std::make_pair(jj, co_tmp));
    }
    coobs_use3->height = 1;
    coobs_use3->width = coobs_use3->points.size();
    pcl::io::savePCDFile("/home/yixin/teaser_ba/src/teaser_ba/test/result/co3obs.pcd", *coobs_use3);
    LOG(INFO) << "[covisibility map use three frames] size: " << covisibility_use3.size();

    // ----------------------------- co-visibility with four frames --------------------------------
    std::map<int, std::map<int, Eigen::Vector3f>> covisibility_use4;
    pcl::PointCloud<pcl::PointXYZ>::Ptr coobs_use4(new pcl::PointCloud<pcl::PointXYZ>());

    for (int jj = 0; jj < association_use4.size(); jj++) {
        std::map<int, Eigen::Vector3f> co_tmp;
        for (auto& co : association_use4[jj]) {
            Eigen::Vector3f obs;
            obs << all_features[co.first].first[co.second].x, 
                   all_features[co.first].first[co.second].y,
                   all_features[co.first].first[co.second].z;
            co_tmp.insert(std::make_pair(co.first, obs));
            pcl::PointXYZ pt;
            pt.x = all_features[co.first].first[co.second].x;
            pt.y = all_features[co.first].first[co.second].y;
            pt.z = all_features[co.first].first[co.second].z;
            coobs_use4->points.emplace_back(pt);
        }
        covisibility_use4.insert(std::make_pair(jj, co_tmp));
    }
    coobs_use4->height = 1;
    coobs_use4->width = coobs_use4->points.size();
    pcl::io::savePCDFile("/home/yixin/teaser_ba/src/teaser_ba/test/result/co4obs.pcd", *coobs_use4);
    LOG(INFO) << "[covisibility map use four frames] size: " << covisibility_use4.size();

    

    // // ---------------------------- old version ------------------------------------
    
    // // std::vector<std::vector<std::pair<feature_id, feature_id>>>
    // std::vector<std::vector<std::map<int, int>>> correspondence;

    // // get covisibility
    // for (int i = 0; i < all_features.size() - 1; i++) {
    //     auto point_cloud1 = all_features[i].first;
    //     auto fpfh_cloud1 = all_features[i].second;
    //     auto point_cloud2 = all_features[i + 1].first;
    //     auto fpfh_cloud2 = all_features[i + 1].second;
    //     auto trans1 = all_translation[i];
    //     auto trans2 = all_translation[i + 1];

    //     auto corr = my_teaser.getCorrespondence(point_cloud1, point_cloud2, 
    //                                             fpfh_cloud1, fpfh_cloud2,
    //                                             trans1, trans2);
    //     correspondence.emplace_back(corr);
    //     LOG(INFO) << "[FPFH features correspondece] between " << i << " and " << i + 1 << " size: " << corr.size();
    // }

    // // ---------------------------------- co-visibility with 4 frames -------------------------------------------
    // // std::vector<std::vector<std::pair<frame_id, feature_id>>>
    // std::vector<std::vector<std::pair<int, int>>> fusion_pair;

    // for (int ii = 0; ii < correspondence.size(); ii++) {
    //     std::vector<std::map<int, int>> curr_corr = correspondence[ii];

    //     // // init
    //     // if(ii == 0) {
    //     //     for (auto corr : curr_corr) {
    //     //         std::vector<std::pair<int, int>> vec0_pair{std::make_pair(ii, corr.begin()->first),
    //     //                                                    std::make_pair(ii + 1, corr.begin()->second)};
    //     //         fusion_pair.emplace_back(vec0_pair);
    //     //     }
    //     //     continue;
    //     // }   

    //     std::vector<std::vector<std::pair<int, int>>> fusion_pair_tmp;

    //     // loop
    //     for (auto& corr : curr_corr) {
    //         int front = corr.begin()->first;
    //         bool find_flag = false;
    //         // #pragma omp parallel for
    //         for (int kk = 0; kk < fusion_pair.size(); kk++) {
    //             if (fusion_pair[kk].back().first == ii && fusion_pair[kk].back().second == front) {
    //                 fusion_pair[kk].emplace_back(std::make_pair(ii + 1, corr.begin()->second));
    //                 find_flag = true;
    //                 continue;
    //             }
    //         }
    //         if (!find_flag) {
    //             std::vector<std::pair<int, int>> veckk_pair{std::make_pair(ii, front),
    //                                                         std::make_pair(ii + 1, corr.begin()->second)};
    //             fusion_pair_tmp.emplace_back(veckk_pair);
    //         }
    //     }

    //     for (auto& tmp : fusion_pair_tmp) {
    //         fusion_pair.emplace_back(tmp);
    //     }
    // }
    // LOG(INFO) << "[covisibility set] size: " << fusion_pair.size();

    // std::map<int, std::map<int, Eigen::Vector3f>> covisibility;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr coobs(new pcl::PointCloud<pcl::PointXYZ>());

    // for (int jj = 0; jj < fusion_pair.size(); jj++) {
    //     if (fusion_pair[jj].size() < 4) {
    //         continue;
    //     }
    //     std::map<int, Eigen::Vector3f> co_tmp;
    //     for (auto& co : fusion_pair[jj]) {
    //         Eigen::Vector3f obs;
    //         obs << all_features[co.first].first[co.second].x, 
    //                all_features[co.first].first[co.second].y,
    //                all_features[co.first].first[co.second].z;
    //         co_tmp.insert(std::make_pair(co.first, obs));
    //         pcl::PointXYZ pt;
    //         pt.x = all_features[co.first].first[co.second].x;
    //         pt.y = all_features[co.first].first[co.second].y;
    //         pt.z = all_features[co.first].first[co.second].z;
    //         coobs->points.emplace_back(pt);
    //     }
    //     covisibility.insert(std::make_pair(jj, co_tmp));
    // }

    // coobs->height = 1;
    // coobs->width = coobs->points.size();
    // pcl::io::savePCDFile("/home/yixin/teaser_ba/src/teaser_ba/test/result/co4obs_bt1234.pcd", *coobs);
    // LOG(INFO) << "[covisibility map] size: " << covisibility.size();

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