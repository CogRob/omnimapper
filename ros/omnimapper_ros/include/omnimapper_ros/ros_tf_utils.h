#pragma once

#include <gtsam/geometry/Pose3.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

namespace omnimapper {
/** Converts a tf to a gtsam::Pose3 */
gtsam::Pose3 TfToPose3(tf::StampedTransform transform);
// {
//   tf::Vector3 axis = transform.getRotation().getAxis();
//   double len = sqrt(axis[0]*axis[0] +
//                     axis[1]*axis[1] +
//                     axis[2]*axis[2]);
//   assert(len != 0);
//   gtsam::Vector gtsam_axis =
//   gtsam::Vector_(3,axis[0]/len,axis[1]/len,axis[2]/len); double angle =
//   transform.getRotation().getAngle(); return
//   gtsam::Pose3(gtsam::Rot3::rodriguez(gtsam_axis,angle),
//                       gtsam::Point3(transform.getOrigin().x(),
//                                     transform.getOrigin().y(),
//                                     transform.getOrigin().z()));
// }

tf::Transform Pose3ToTf(gtsam::Pose3& pose);

}  // namespace omnimapper
