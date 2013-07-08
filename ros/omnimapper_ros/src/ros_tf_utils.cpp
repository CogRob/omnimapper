#include <omnimapper_ros/ros_tf_utils.h>

gtsam::Pose3 
omnimapper::tf2pose3 (tf::StampedTransform transform)
{
  tf::Vector3 axis = transform.getRotation().getAxis();
  double len = sqrt(axis[0]*axis[0] + 
                    axis[1]*axis[1] + 
                    axis[2]*axis[2]);
  assert(len != 0);
  gtsam::Vector gtsam_axis = gtsam::Vector_(3,axis[0]/len,axis[1]/len,axis[2]/len);
  double angle = transform.getRotation().getAngle();
  return gtsam::Pose3(gtsam::Rot3::rodriguez(gtsam_axis,angle),
                      gtsam::Point3(transform.getOrigin().x(),
                                      transform.getOrigin().y(),
                                    transform.getOrigin().z()));
}

tf::Transform 
omnimapper::pose3totf (gtsam::Pose3& pose)
{
  tf::Transform t;
  t.setOrigin(tf::Vector3(pose.x(), pose.y(), pose.z()));
  tf::Quaternion q;
  gtsam::Vector ypr = pose.rotation().ypr();
  q.setRPY(ypr[2], ypr[1], ypr[0]);
  t.setRotation(q);
  return t;
}
