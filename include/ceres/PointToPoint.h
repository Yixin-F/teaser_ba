#ifndef _POINT_TO_POINT_H_
#define _POINT_TO_POINT_H_

#include "pose_local_parameterization.h"

class pointBA {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
public:
    pointBA() {};
    static void optimize(const std::map<int, std::map<int, Eigen::Vector3f>>& covisibility,
                         const std::map<int, Eigen::Matrix4f>& bf_poses,
                         std::map<int, Eigen::Vector3d>& opt_landmarks, 
                         std::map<int, Eigen::Matrix4d>& opt_poses) 
    {
        // construct ceres optim parameters
        ceres::Problem problem;
        ceres::LossFunction *loss_function = new ceres::CauchyLoss(1.0);
        ceres::ParameterBlockOrdering *ordering = new ceres::ParameterBlockOrdering();

        // add assumption
        int assumption_count = 0;
        double Para_Point_Feature[covisibility.size()][3];
        for (auto mp = covisibility.begin(); mp != covisibility.end(); mp++) {
            int mp_id = mp->first;
            std::map<int, Eigen::Vector3f> obs = mp->second;
            if (obs.size() < 2) {
                std::cerr << "error occured, because there are no co-visibility for feature " << mp_id << std::endl;
                continue;
            }

            // FIXME: transefer into world coordinate
            int obs_frame_id = mp->second.begin()->first;
            Eigen::Matrix4f trans = bf_poses.at(obs_frame_id);
            Eigen::Matrix3f rotation_wc = trans.block(0, 0, 3, 3);
            Eigen::Vector3f translation_wc = trans.block(0, 3, 3, 1);
            Eigen::Vector3f assumption = rotation_wc * mp->second.begin()->second + translation_wc;
            if (assumption == Eigen::Vector3f::Zero()) {
                std::cerr << "feature " << mp_id << " is not valid ..." << std::endl;
                continue;
            }
    
            Para_Point_Feature[mp_id][0] = assumption(0);
            Para_Point_Feature[mp_id][1] = assumption(1);
            Para_Point_Feature[mp_id][2] = assumption(2);
            problem.AddParameterBlock(Para_Point_Feature[mp_id], 3);
            assumption_count ++;
        }
        LOG(INFO) << "[ceres point BA] add assumption size: " << assumption_count;

        // add camera pose
        int pose_count = 0;
        double Para_Pose[bf_poses.size()][7];
        for (auto kf = bf_poses.begin(); kf != bf_poses.end(); kf++) {
            int kf_id = kf->first;
            Eigen::Matrix4f Twc = kf->second;
            Eigen::Matrix3f Rot = Twc.block(0, 0, 3, 3);
            Eigen::Vector3f Tran = Twc.block(0, 3, 3, 1);
            Eigen::Quaternionf qua(Rot);

            Para_Pose[kf_id][0] = Tran(0);
            Para_Pose[kf_id][1] = Tran(1);
            Para_Pose[kf_id][2] = Tran(2);
            Para_Pose[kf_id][3] = qua.x();
            Para_Pose[kf_id][4] = qua.y();
            Para_Pose[kf_id][5] = qua.z();
            Para_Pose[kf_id][6] = qua.w();

            ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
            problem.AddParameterBlock(Para_Pose[kf_id], 7, local_parameterization);
            pose_count ++;
            if(kf_id == 0) {
                problem.SetParameterBlockConstant(Para_Pose[kf_id]);
            }        
        }
        LOG(INFO) << "[ceres point BA] add pose size: " << pose_count;

        // add residual
        for (auto asso_mp_mea = covisibility.begin(); asso_mp_mea !=  covisibility.end(); asso_mp_mea++) {
            int mp_id = asso_mp_mea->first;

            Eigen::Vector3f assumption = asso_mp_mea->second.begin()->second;
            if (assumption == Eigen::Vector3f::Zero()) {
                std::cerr << "feature " << mp_id << " is not valid ..." << std::endl;
                continue;
            }

            if(asso_mp_mea->second.size() < 2) {
                std::cerr << "feature " << mp_id << " is not valid ..." << std::endl;
                continue;
            }

            auto obs_group = asso_mp_mea->second;
            for (auto obs_index = obs_group.begin(); obs_index != obs_group.end(); obs_index++) {
                int kf_id = obs_index->first;
                Eigen::Vector3d mp_obs;
                mp_obs(0) = double(obs_index->second(0));
                mp_obs(1) = double(obs_index->second(1));
                mp_obs(2) = double(obs_index->second(2));
                PointProjectionFactor *f = new PointProjectionFactor(mp_obs);
                problem.AddResidualBlock(f, loss_function, Para_Pose[kf_id], Para_Point_Feature[mp_id]);
            }
        }

        // slove ceres problem
        ceres::Solver::Options options;
        options.minimizer_progress_to_stdout = true;
        // options.max_num_iterations = 15;
        // options.linear_solver_type = ceres::DENSE_SCHUR;
        options.linear_solver_type = ceres::ITERATIVE_SCHUR;
        options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        options.update_state_every_iteration = true;
        // options.gradient_tolerance = 1e-3;
        // options.function_tolerance = 1e-4;
        options.max_num_iterations = 100;

        ceres::Solver::Summary summary;
        ceres::Solve (options, &problem, & summary);

        // update ceres results
        for (auto kf =  bf_poses.begin(); kf !=  bf_poses.end(); kf++) {
            int kf_id = kf->first;
            Eigen::Vector3d tran(Para_Pose[kf_id][0], Para_Pose[kf_id][1], Para_Pose[kf_id][2]);
            Eigen::Quaterniond qua(Para_Pose[kf_id][6], Para_Pose[kf_id][3], Para_Pose[kf_id][4], Para_Pose[kf_id][5]);
            Eigen::Matrix3d Rot = qua.toRotationMatrix();
            Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
            Twc.block(0, 0, 3, 3) = Rot;
            Twc.block(0, 3, 3, 1) = tran;
            // kfs[kf_id] = Twc;
            opt_poses[kf_id] = Twc;
        }

        // update mappoints
        for (auto mp = covisibility.begin(); mp != covisibility.end(); mp++) {
            int mp_id = mp->first;

            Eigen::Vector3f assumption = mp->second.begin()->second;
            if (assumption == Eigen::Vector3f::Zero()) {
                std::cerr << "feature " << mp_id << " is not valid ..." << std::endl;
                continue;
            }

            Eigen::Vector3d optimized_mappoint;
            optimized_mappoint << Para_Point_Feature[mp_id][0],
                                  Para_Point_Feature[mp_id][1],         
                                  Para_Point_Feature[mp_id][2];                   
            opt_landmarks[mp_id] = optimized_mappoint;
        }

    }
};

#endif