#pragma once
#include <gtsam/geometry/Pose3.h>

gtsam::Pose3 TransformToPose3(const Eigen::Affine3f& transform);

Eigen::Affine3f Pose3ToTransform(const gtsam::Pose3& pose);

/* \brief Given two plane equations, this will generate a transform to align the
 * points on them. */
Eigen::Affine3d PlanarAlignmentTransform(const Eigen::Vector4d& target,
                                         const Eigen::Vector4d& to_align);
