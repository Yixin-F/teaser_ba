#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "teaser-toolkit/TeaserHead.h"

#include "ceres/PointToPoint.h"

// initialize 
int mode = 0; // 0 -> C, 1 -> S

std::map<int, Eigen::Vector3f> truss_positionC {{0, Eigen::Vector3f{0., 0., 0.}}, {1, Eigen::Vector3f{0., 0.6, 0.}},
                                                {2, Eigen::Vector3f{0., 1.2, 0.}}, {3, Eigen::Vector3f{0., 1.8, 0.}},
                                                {4, Eigen::Vector3f{0., 2.4, 0.}}, {5, Eigen::Vector3f{0., 3.0, 0.}},
                                                {6, Eigen::Vector3f{0., 3.6, 0.}}, {7, Eigen::Vector3f{1.2, 3.6, 0.}},
                                                {8, Eigen::Vector3f{1.2, 3.0, 0.}}, {9, Eigen::Vector3f{1.2, 2.4, 0.}},
                                                {10, Eigen::Vector3f{1.2, 1.8, 0.}}, {11, Eigen::Vector3f{1.2, 1.2, 0.}},
                                                {12, Eigen::Vector3f{1.2, 0.6, 0.}}, {13, Eigen::Vector3f{1.2, -0.05, 0.}}};

std::map<int, Eigen::Vector3f> truss_positionS {{0, Eigen::Vector3f{-0.4, 0., 0.}}, {1, Eigen::Vector3f{-0.4, 0.6, 0.}},
                                                {2, Eigen::Vector3f{-0.4, 1.2, 0.}}, {3, Eigen::Vector3f{-0.4, 1.8, 0.}},
                                                {4, Eigen::Vector3f{-0.4, 2.4, 0.}}, {5, Eigen::Vector3f{-0.4, 3.0, 0.}},
                                                {6, Eigen::Vector3f{-0.4, 3.6, 0.}}, {7, Eigen::Vector3f{0.8, 3.6, 0.}},
                                                {8, Eigen::Vector3f{0.8, 3.0, 0.}}, {9, Eigen::Vector3f{0.8, 2.4, 0.}},
                                                {10, Eigen::Vector3f{0.8, 1.8, 0.}}, {11, Eigen::Vector3f{0.8, 1.2, 0.}},
                                                {12, Eigen::Vector3f{0.8, 0.6, 0.}}, {13, Eigen::Vector3f{0.8, 0., 0.}},
                                                {14, Eigen::Vector3f{2.0, 0., 0.}}, {15, Eigen::Vector3f{2.0, 0.6, 0.}},
                                                {16, Eigen::Vector3f{2.0, 1.2, 0.}}, {17, Eigen::Vector3f{2.0, 1.8, 0.}},
                                                {18, Eigen::Vector3f{2.0, 2.4, 0.}}, {19, Eigen::Vector3f{2.0, 3.0, 0.}},
                                                {20, Eigen::Vector3f{2.0, 3.6, 0.}}};

int num_cloud = 0;
pcl::PointCloud<pcl::PointXYZ>::Ptr truss_cloud(new pcl::PointCloud<pcl::PointXYZ>);
std::vector<pcl::PointCloud<PointType>::Ptr> all_cloud;
std::vector<Eigen::Vector3f> all_translation;
std::map<int, Eigen::Matrix4f> all_trans;
std::map<float, std::vector<std::map<int, int>>> all_correspondence;

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

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_o(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud_o);

    // remove Nan
    for (size_t i = 0; i < cloud_o->points.size(); ++i) {
        if (!std::isnan(cloud_o->points[i].x) &&
            !std::isnan(cloud_o->points[i].y) &&
            !std::isnan(cloud_o->points[i].z))
        {
            cloud->emplace_back(cloud_o->points[i]);
        }
    }

    // downsampling
    pcl::VoxelGrid<pcl::PointXYZ> downsample;
    downsample.setLeafSize(0.01, 0.01, 0.01);  // 10mm
    downsample.setInputCloud(cloud);
    downsample.filter(*cloud);

    // transformation
    if (mode == 0) {
        LOG(INFO) << "use truss C ...";

        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.translation() << truss_positionC[num_cloud];
        pcl::transformPointCloud(*cloud, *cloud, transform);
        *truss_cloud += *cloud;
        all_cloud.emplace_back(cloud);

        all_translation.emplace_back(truss_positionC[num_cloud]);

        Eigen::Matrix4f trans;
        trans.setIdentity();
        trans.block(0, 3, 3, 1) = truss_positionC[num_cloud];
        all_trans.insert(std::make_pair(num_cloud, trans));

        LOG(INFO) << "recieved cloud " << num_cloud << " with size " << cloud->points.size();

        num_cloud ++;
    }
    else if (mode == 1) {
        LOG(INFO) << "use truss S ...";
        
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.translation() << truss_positionS[num_cloud];
        pcl::transformPointCloud(*cloud, *cloud, transform);
        *truss_cloud += *cloud;
        all_cloud.emplace_back(cloud);

        all_translation.emplace_back(truss_positionC[num_cloud]);

        Eigen::Matrix4f trans;
        trans.setIdentity();
        trans.block(0, 3, 3, 1) = truss_positionC[num_cloud];
        all_trans.insert(std::make_pair(num_cloud, trans));

        LOG(INFO) << "recieved cloud " << num_cloud << "with size " << cloud->points.size();

        num_cloud ++;
    }
    else {
        LOG(ERROR) << "invalid truss type !!";
    }
}


int main (int argc, char* argv[])
{
    // google::InitGoogleLogging(argv[0]);

    ros::init(argc, argv, "teaser_ba");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/point_cloud_topic", 10, cloudCallback);

    fpfh_teaser my_teaser;
    LOG(INFO) << "Hi, I'm TEASER-BA ...";

    int construction_size = 0;

    if (mode == 0) {
        construction_size = 14;
    }
    else if (mode == 1){
        construction_size = 21;
    }
    else {
        LOG(ERROR) << "invalid truss type !!";
    }

    while (all_cloud.size() != construction_size) { ros::spinOnce(); }
    LOG(INFO) << "have recieved enough frames ...";
    truss_cloud->height = 1;
    truss_cloud->width = truss_cloud->points.size();
    pcl::io::savePCDFile("/home/yixin/teaser_ba/src/teaser_ba/test/result/truss.pcd", *truss_cloud);


    ros::Time start = ros::Time::now();
    
    // TODO: get feature
    std::vector<std::pair<teaser::PointCloud, teaser::FPFHCloud>> all_features;
    for (auto& cloud : all_cloud) {
        auto feature_pair = my_teaser.getFeatureInfo(cloud);
        all_features.emplace_back(feature_pair);
        LOG(INFO) << "[FPFH features extraction] size: " << feature_pair.second.size();
    }
    
    // TRUSS C :
    //  0 ---- 1 ---- 2 ---- 3 ---- 4 ---- 5 ---- 6 
    //  | \  / | \  / | \  / | \  / | \  / | \  / |
    //  |  \/  |  \/  |  \/  |  \/  |  \/  |  \/  |
    //  |  /\  |  /\  |  /\  |  /\  |  /\  |  /\  |
    //  | /  \ | /  \ | /  \ | /  \ | /  \ | /  \ |
    // 13 --- 12 --- 11 --- 10 ---- 9 ---- 8 ---- 7

    // TODO: get group
    std::vector<std::vector<float>> group_use2 {{0.01}, {1.02}, {2.03}, {3.04}, {4.05}, {5.06}, {6.07}, {7.08}, {8.09}, {9.10}, {10.11}, {11.12}, {12.13}, {0.13},
                                                {1.12}, {2.11}, {3.10}, {4.09}, {5.08},
                                                {0.12}, {1.13}, {1.11}, {2.12}, {2.10}, {3.11}, {3.09}, {4.10}, {4.08}, {5.09}, {5.07}, {6.08}};  // two frames
    std::vector<std::vector<float>> group_use3 {{0.01, 1.12}, {0.01, 0.13}, {0.13, 12.13}, {1.12, 12.13},
                                                {1.12, 1.02}, {1.02, 2.11}, {2.11, 11.12}, {11.12, 1.12},
                                                {2.03, 2.11}, {2.03, 3.10}, {3.10, 10.11}, {10.11, 2.11},
                                                {3.10, 3.04}, {3.04, 4.09}, {4.09, 9.10}, {9.10, 3.10},
                                                {4.09, 4.05}, {4.05, 5.08}, {5.08, 8.09}, {8.09, 4.09},
                                                {5.08, 5.06}, {5.06, 6.07}, {6.07, 7.08}, {7.08, 5.08}}; // three frames
    std::vector<std::vector<float>> group_use4 {{0.01, 1.12, 12.13}, {1.02, 2.11, 11.12}, {2.03, 3.10, 10.11},
                                                {3.04, 4.09, 9.10}, {4.05, 5.08, 8.09}, {5.06, 6.07, 7.08}}; // four frames

    // TODO: get correspondence
    for (auto& group : group_use2) {
        float corr_frontend = group[0];
        int front = std::floor(corr_frontend);
        int end = std::round((corr_frontend - std::floor(corr_frontend)) * 100.0);

        auto correspondence_bt12 = my_teaser.getCorrespondence(all_features[front].first, all_features[end].first, 
                                                               all_features[front].second, all_features[end].second,
                                                               all_translation[front], all_translation[end]);
        all_correspondence.insert(std::make_pair(corr_frontend, correspondence_bt12));
        LOG(INFO) << "[FPFH features correspondece] " << corr_frontend << " between " << front << " and " << end << " size: " << correspondence_bt12.size();
    }

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
                    if ((ass.back().first == front && ass.back().second == cor.begin()->first) ||
                        (ass.front().first == front && ass.front().second == cor.begin()->first) ||
                        (ass.front().first == end && ass.front().second == cor.begin()->second) ||
                        (ass.back().first == end && ass.back().second == cor.begin()->second)) {
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

if (0) {
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

    // ----------------------------- refine co-visibility with four frames --------------------------------
    LOG(INFO) << "[data association with four frames] refined size: " << association_use4.size();
}

    // TODO: get covisibility
    int fea_id = 0;
    std::map<int, std::map<int, Eigen::Vector3f>> covisibility_all;
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
            // std::cout << obs << std::endl;
            pcl::PointXYZ pt;
            pt.x = all_features[co.first].first[co.second].x;
            pt.y = all_features[co.first].first[co.second].y;
            pt.z = all_features[co.first].first[co.second].z;
            coobs_use2->points.emplace_back(pt);
        }
        covisibility_use2.insert(std::make_pair(jj, co_tmp));
        covisibility_all.insert(std::make_pair(fea_id, co_tmp));
        fea_id ++;
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
        covisibility_all.insert(std::make_pair(fea_id, co_tmp));
        fea_id ++;
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
        covisibility_all.insert(std::make_pair(fea_id, co_tmp));
        fea_id ++;
    }
    coobs_use4->height = 1;
    coobs_use4->width = coobs_use4->points.size();
    pcl::io::savePCDFile("/home/yixin/teaser_ba/src/teaser_ba/test/result/co4obs.pcd", *coobs_use4);
    LOG(INFO) << "[covisibility map use four frames] size: " << covisibility_use4.size();

    // TODO: point BA
    // ---------------------------- ceres BA --------------------------------
    pointBA point_ba;
    std::map<int, Eigen::Vector3d> opt_landmarks;
    std::map<int, Eigen::Matrix4d> opt_poses;
    point_ba.optimize(covisibility_all, all_trans, opt_landmarks, opt_poses);

    // TODO: resulted global cloud
    // -------------------------------  save result -----------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (int i = 0; i < opt_poses.size(); i++) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*all_cloud[i], *cloud, opt_poses[i]);
        *global_cloud += *cloud;
        LOG(INFO) << "[optimized poses] " << i << ":  \n" << opt_poses[i];
    }
    pcl::io::savePCDFile("/home/yixin/teaser_ba/src/teaser_ba/test/result/global.pcd", *global_cloud);

    ros::Time end = ros::Time::now();

    LOG(INFO) << "[whole time cost] " << (end - start).toSec();

    return 0;
}