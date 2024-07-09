#ifndef _POINT_TO_POINT_H_
#define _POINT_TO_POINT_H_

#include "../Utility.h"
#include "pose_local_parameterization.h"

class pointBA {
public:
    pointBA() {};
    void optimize(const std::map<int, std::map<int, Eigen::Vector3f>>& covisibility,
                  const std::map<int, Eigen::Matrix4f> &bf_poses,
                  std::map<int, Eigen::Vector3f>& opt_landmarks, 
                  std::map<int, Eigen::Matrix4f> &opt_poses);
};

#endif