#ifndef _POINT_TO_POINT_G2O_H
#define _POINT_TO_POINT_G2O_H

#include "../Utility.h"

class PointBAusingG2O {
public:
    ~PointBAusingG2O() {}
    static void optimize(const std::map<int, std::map<int, Eigen::Vector3d>>& covisibility,
                         const std::map<int, Eigen::Matrix4d>& bf_poses,
                         std::map<int, Eigen::Vector3d>& opt_landmarks, 
                         std::map<int, Eigen::Matrix4d>& opt_poses) 
    {
        // initializa g2o optimizer
        g2o::SparseOptimizer optimizer;
        optimizer.setVerbose(true);
        std::unique_ptr<LinearSolverType> linearSolver = std::make_unique<LinearSolverType>();
        std::unique_ptr<BlockSolverType> blockSolver = std::make_unique<BlockSolverType>(std::move(linearSolver));
        g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));
        optimizer.setAlgorithm(algorithm);

        // add camera pose
        int pose_size = 0;
        for (const auto& pose : bf_poses) {
            int pose_id = pose.first;
            Eigen::Matrix4d pose_T(pose.second);
            Eigen::Matrix3d pose_rotation = pose_T.block(0, 0, 3, 3);
            Eigen::Vector3d pose_translation = pose_T.block(0, 3, 3, 1);

            g2o::VertexSE3Expmap* v_se3 = new g2o::VertexSE3Expmap();
            v_se3->setId(pose_id);

            if (pose_id == 0) {
                v_se3->setFixed(true);
            }

            v_se3->setEstimate(g2o::SE3Quat(pose_rotation, pose_translation));
            optimizer.addVertex(v_se3);
            pose_size ++;
        }
        LOG(INFO) << "[add vertex] pose size : " << pose_size;

        // add landmarks
        int lm_size = 0;
        for (const auto& iterm : covisibility) {
            int lm_id = iterm.first;
            auto obs = iterm.second;
            int anchor_id = obs.begin()->first;

            // TODO: transfer into world coordinate
            Eigen::Vector3d anchor_pos(obs.begin()->second);
            Eigen::Matrix4d anchor_pose(bf_poses.at(anchor_id));
            Eigen::Matrix3d anchor_rotation = anchor_pose.block(0, 0, 3, 3);
            Eigen::Vector3d anchor_translation = anchor_pose.block(0, 3, 3, 1);
            Eigen::Vector3d lm_assumption = anchor_rotation * anchor_pos + anchor_translation;

            g2o::VertexPointXYZ* v_p = new g2o::VertexPointXYZ();
            v_p->setId(lm_id + bf_poses.size());
            v_p->setEstimate(lm_assumption);
            optimizer.addVertex(v_p);
            lm_size ++;
        }
        LOG(INFO) << "[add vertex] landmarks size : " << lm_size;

        // add observation
        int obs_size = 0;
        for (const auto& iterm : covisibility) {
            for (const auto& cov : iterm.second) {
                g2o::EdgeSE3PointXYZ* edge = new g2o::EdgeSE3PointXYZ();
                // auto edge = std::make_shared<g2o::EdgeSE3PointXYZ>();
                edge->setId(obs_size);
                edge->setParameterId(0, 0);
                std::cout << 1 << std::endl;
                if (!optimizer.vertex(cov.first) || !optimizer.vertex(iterm.first + bf_poses.size())) {
                    std::cerr << "error" << std::endl;
                }
                edge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(cov.first)));
                edge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(iterm.first + bf_poses.size())));
                std::cout << 2 << std::endl;
                // edge->setMeasurement(cov.second);
                std::cout << 3 << std::endl;
                edge->setInformation(Eigen::Matrix3d::Identity());
                std::cout << 4 << std::endl;
                optimizer.addEdge(dynamic_cast<g2o::OptimizableGraph::Edge*>(edge));
                // optimizer.addEdge(edge.get());
                std::cout << 5 << std::endl;
                obs_size ++;
            }
        }
        LOG(INFO) << "[add edge] observation size : " << obs_size;

        // optimize g2o graph
        optimizer.initializeOptimization();
        optimizer.optimize(20);

        // get optimized poses
        for (const auto& pose : bf_poses) {
            int pose_id = pose.first;
            g2o::VertexSE3Expmap* v_se3 = dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pose_id));
            auto opt_pose = v_se3->estimate().to_homogeneous_matrix();
            opt_poses.insert(std::make_pair(pose_id, opt_pose));
        }
        LOG(INFO) << "[optimization results] optimized pose size: " << opt_poses.size();

        // get optimized landmarks
        for (const auto& iterm : covisibility) {
            int lm_id = iterm.first;
            g2o::VertexPointXYZ* v_p = dynamic_cast<g2o::VertexPointXYZ*>(optimizer.vertex(lm_id + bf_poses.size()));
            Eigen::Vector3d opt_lm = v_p->estimate().transpose();
            opt_landmarks.insert(std::make_pair(lm_id, opt_lm));
        }
        LOG(INFO) << "[optimization results] optimized landmark size: " << opt_landmarks.size();

        // output results
        LOG(INFO) << "[optimization results] done ... ";
    }

};

#endif _POINT_TO_POINT_G2O_H