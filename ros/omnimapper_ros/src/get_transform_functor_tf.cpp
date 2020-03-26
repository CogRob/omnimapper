#include <omnimapper_ros/get_transform_functor_tf.h>
#include <omnimapper_ros/ros_time_utils.h>

omnimapper::GetTransformFunctorTF::GetTransformFunctorTF(
    std::string sensor_frame_name, std::string base_frame_name)
    : tf_listener_(ros::Duration(120.0)),
      sensor_frame_name_(sensor_frame_name),
      base_frame_name_(base_frame_name) {}

Eigen::Affine3d omnimapper::GetTransformFunctorTF::operator()(
    omnimapper::Time t) {
  ros::Time ros_time = PtimeToRosTime(t);
  tf::StampedTransform tf_transform;
  try {
    tf_listener_.waitForTransform(base_frame_name_, sensor_frame_name_,
                                  ros_time, ros::Duration(0.2));
    tf_listener_.lookupTransform(base_frame_name_, sensor_frame_name_, ros_time,
                                 tf_transform);
  } catch (tf::TransformException ex) {
    // printf ("GetTransformFunctorTF: Error looking up tf: %s\n", ex.what ());
    ROS_ERROR("GetTransformFunctorTF: Error looking up tf: %s\n", ex.what());
    return (Eigen::Affine3d::Identity());
  }

  // TODO: make a function for this
  gtsam::Pose3 transform_p3 = omnimapper::TfToPose3(tf_transform);
  Eigen::Affine3d transform(transform_p3.matrix().cast<double>());
  return (transform);
}
