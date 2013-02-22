#include <omnimapper/transform_helpers.h>
#include <pcl/common/common.h>
#include <pcl/common/eigen.h>

gtsam::Pose3 transformToPose3(const Eigen::Affine3f& tform)
{
  //  return gtsam::Pose3(gtsam::Rot3(tform(0,0),tform(0,1),tform(0,2),
  //				  tform(1,0),tform(1,1),tform(1,2),
  //				  tform(2,0),tform(2,1),tform(2,2),
  //				  gtsam::Point3(tform());
  return gtsam::Pose3(tform.matrix().cast<double>());
}

Eigen::Affine3f pose3ToTransform(const gtsam::Pose3& pose)
{
  gtsam::Vector ypr = pose.rotation().ypr();
  Eigen::Affine3f t = pcl::getTransformation(pose.x(), pose.y(), pose.z(), ypr[2], ypr[1], ypr[0]);
  return t;
}

