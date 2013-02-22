#pragma once
#include <gtsam/geometry/Pose3.h>

gtsam::Pose3 transformToPose3(const Eigen::Affine3f& transform);

Eigen::Affine3f pose3ToTransform(const gtsam::Pose3& pose);
