#include "TeaserHead.h"

fpfh_teaser::fpfh_teaser(){
    allocateMemory();

    LOG(INFO) << " NORMAL_ESTIMATION_RADIUS " << NORMAL_ESTIMATION_RADIUS << "\n"
              << " MATCH_DISTANCE " << MATCH_DISTANCE << "\n"
              << " FPFH_SEARCH_RADIUS " << FPFH_SEARCH_RADIUS;
    
    downSizeFilterSurf.setLeafSize(0.01, 0.01, 0.01);
}

void fpfh_teaser::allocateMemory(){
    tree.reset(new pcl::KdTreeFLANN<PointType>());
    cloud_source.reset(new pcl::PointCloud<PointType>());
    cloud_target.reset(new pcl::PointCloud<PointType>());
    cloud_source_transformed.reset(new pcl::PointCloud<PointType>());
}

void fpfh_teaser::set_source(const pcl::PointCloud<PointType>::Ptr& cloud){
    cloud_source->clear();
    downSizeFilterSurf.setInputCloud(cloud);
    downSizeFilterSurf.filter(*cloud);
    *cloud_source += *cloud;
    LOG(INFO) << " source cloud size: " << cloud_source->points.size();
}

void fpfh_teaser::set_target(const pcl::PointCloud<PointType>::Ptr& cloud){
    cloud_target->clear();
    downSizeFilterSurf.setInputCloud(cloud);
    downSizeFilterSurf.filter(*cloud);
    *cloud_target += *cloud;
    tree->setInputCloud(cloud_target);
    LOG(INFO) << " target cloud size: " << cloud_target->points.size();
}

double fpfh_teaser::calc_matching_error(const Eigen::Matrix4f& transformation){
    int num_inliers = 0;
    double matching_error = 0.0;

    pcl::PointCloud<PointType> transformed;
    pcl::transformPointCloud(*cloud_source, transformed, transformation);

    std::vector<int> indices;
    std::vector<float> sq_dists;
    for (int i = 0; i < transformed.size(); i++) {
        tree->nearestKSearch(transformed[i], 1, indices, sq_dists);
        if (sq_dists[0] < MATCH_DISTANCE) {
            num_inliers ++;
            matching_error += sq_dists[0];
        }
    }

    return num_inliers ? matching_error / num_inliers : std::numeric_limits<double>::max();
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_teaser::extract_fpfh(const pcl::PointCloud<PointType>::Ptr& cloud){
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::copyPointCloud(*cloud, *tmp);
    pcl::NormalEstimationOMP<pcl::PointXYZI, pcl::Normal> nest;
    nest.setRadiusSearch(NORMAL_ESTIMATION_RADIUS);
    nest.setInputCloud(tmp);
    nest.compute(*normals);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr features(new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::FPFHEstimationOMP<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33> fest;
    fest.setInputCloud(tmp);
    fest.setRadiusSearch(FPFH_SEARCH_RADIUS);
    fest.setInputNormals(normals);
    fest.compute(*features);

    return features;
}

std::pair<double, Eigen::Matrix4f> fpfh_teaser::match(const bool& v){
    auto fpfh_source = extract_fpfh(cloud_source);
    auto fpfh_target = extract_fpfh(cloud_target);
    LOG(INFO) << " source fpfh size: " << fpfh_source->size();
    LOG(INFO) << " target fpfh size: " << fpfh_target->size();

    teaser::PointCloud target_teaser, source_teaser;
    teaser::FPFHCloud target_fpfh, source_fpfh;

    target_teaser.reserve(cloud_target->size());
    target_fpfh.reserve(cloud_target->size());
    for(int i = 0; i < cloud_target->size(); i++){
        target_teaser.push_back({cloud_target->at(i).x, cloud_target->at(i).y, cloud_target->at(i).z});
        target_fpfh.push_back(fpfh_target->at(i));
    }

    source_teaser.reserve(cloud_source->size());
    source_fpfh.reserve(cloud_source->size());
    for(int i = 0; i < cloud_source->size(); i++){
        source_teaser.push_back({cloud_source->at(i).x, cloud_source->at(i).y, cloud_source->at(i).z});
        source_fpfh.push_back(fpfh_source->at(i));
    }

    teaser::Matcher matcher;
    auto correspondences = matcher.calculateCorrespondences(
        source_teaser,
        target_teaser,
        source_fpfh,
        target_fpfh,
        USE_ABSOLUTE_SCALE,
        CROSS_CHECK,
        TUPLE_TEST,
        TUPLE_SCALE
    );

    teaser::RobustRegistrationSolver::Params params;
    params.noise_bound = NOISE_BOUND;
    params.cbar2 = CBAR2;
    params.estimate_scaling = false;
    params.rotation_max_iterations = ROTATION_MAX_ITERATIONS;
    params.rotation_gnc_factor = ROTATION_GNC_FACTOR;
    params.rotation_cost_threshold = ROTATION_COST_THRESHOLD;
    params.rotation_estimation_algorithm = teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;

    teaser::RobustRegistrationSolver solver(params);
    solver.solve(source_teaser, target_teaser, correspondences);
    auto solution = solver.getSolution();

    Eigen::Isometry3f transformation = Eigen::Isometry3f::Identity();
    transformation.linear() = solution.rotation.cast<float>();
    transformation.translation() = solution.translation.cast<float>();
    Eigen::Matrix4f transMatrix = transformation.matrix();

    double error;
    v == true ? error = calc_matching_error(transMatrix) : error = 0.;

    return std::make_pair(error, transMatrix);
}

std::pair<teaser::PointCloud, teaser::FPFHCloud> fpfh_teaser::getFeatureInfo(const pcl::PointCloud<PointType>::Ptr& cloud) {
    auto fpfh_source = extract_fpfh(cloud);
    teaser::PointCloud source_teaser;
    teaser::FPFHCloud source_fpfh;

    source_teaser.reserve(cloud->size());
    source_fpfh.reserve(cloud->size());
    for(int i = 0; i < cloud->size(); i++){
        source_teaser.push_back({cloud->at(i).x, cloud->at(i).y, cloud->at(i).z});
        source_fpfh.push_back(fpfh_source->at(i));
    }

    return std::make_pair(source_teaser, source_fpfh);
}

std::vector<std::map<int, int>> fpfh_teaser::getCorrespondence(teaser::PointCloud& pc1, teaser::PointCloud& pc2,
                                                                teaser::FPFHCloud& fc1, teaser::FPFHCloud& fc2,
                                                                const Eigen::Vector3f& trans1, const Eigen::Vector3f& trans2) {
    teaser::Matcher matcher;
    auto correspondences = matcher.calculateCorrespondences(
        pc1,
        pc2,
        fc1,
        fc2,
        USE_ABSOLUTE_SCALE,
        CROSS_CHECK,
        TUPLE_TEST,
        TUPLE_SCALE
    );

    std::vector<std::map<int, int>> map_corr;
    // float delta_x = std::abs(trans2(0) - trans1(0) + 0.02);
    // float delta_y = std::abs(trans2(1) - trans1(1) + 0.02);
    float delta_x = std::abs(0.05);
    float delta_y = std::abs(0.05);

    for (auto& corr : correspondences) {
        if (std::abs(pc2[corr.second].x - pc1[corr.first].x) >= delta_x || std::abs(pc2[corr.second].y - pc1[corr.first].y) >= delta_y) {
            continue;
        }
        std::map<int, int> map;
        map.insert(std::make_pair(corr.first, corr.second));
        map_corr.emplace_back(map);
    }

    return map_corr;
}