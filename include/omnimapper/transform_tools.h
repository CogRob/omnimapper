#pragma once
#include <gtsam/geometry/Pose3.h>

gtsam::Pose3 transformToPose3(const Eigen::Affine3f& transform);

Eigen::Affine3f pose3ToTransform(const gtsam::Pose3& pose);

/* \brief Given two plane equations, this will generate a transform to align the
 * points on them. */
Eigen::Affine3d planarAlignmentTransform(const Eigen::Vector4d& target,
                                         const Eigen::Vector4d& to_align);
