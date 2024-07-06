#ifndef _POINT_TO_POINT_H_
#define _POINT_TO_POINT_H_

#include "../Utility.h"

class pointBA {
public:
    pointBA() {};
    void optimize(const std::map<int, std::map<int, Eigen::Vector3f>>& covisibility,
                  const std::map<int, Eigen::Matrix4f> &bf_poses,
                  std::map<int, Eigen::Vector3f>& opt_landmarks, 
                  std::map<int, Eigen::Matrix4f> &opt_poses);
    
    // construct ceres optim parameters
    ceres::Problem problem;
    ceres::LossFunction *loss_function = new ceres::CauchyLoss(1.0);
    ceres::ParameterBlockOrdering *ordering = new ceres::ParameterBlockOrdering();
};

#endif