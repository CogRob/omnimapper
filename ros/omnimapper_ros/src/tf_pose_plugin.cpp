#include <glog/logging.h>
#include <omnimapper_ros/tf_pose_plugin.h>

namespace omnimapper {

TFPosePlugin::TFPosePlugin(omnimapper::OmniMapperBase* mapper)
    : mapper_(mapper),
      nh_("~"),
      tf_listener_(ros::Duration(30.0)),
      odom_frame_name_("/odom"),
      base_frame_name_("/camera_depth_optical_frame"),
      rotation_noise_(1.0),
      translation_noise_(1.0) {}

gtsam::BetweenFactor<gtsam::Pose3>::shared_ptr TFPosePlugin::AddRelativePose(
    boost::posix_time::ptime t1, gtsam::Symbol sym1,
    boost::posix_time::ptime t2, gtsam::Symbol sym2) {
  // Convert the timestamps.
  ros::Time rt1 = PtimeToRosTime(t1);
  ros::Time rt2 = PtimeToRosTime(t2);

  // Get the poses
  tf::StampedTransform tf1;
  tf::StampedTransform tf2;

  // Add the factor
  bool got_tf = true;
  gtsam::Pose3 relative_pose = gtsam::Pose3::identity();
  try {
    tf_listener_.waitForTransform(odom_frame_name_, base_frame_name_, rt1,
                                  ros::Duration(0.2));
    tf_listener_.waitForTransform(odom_frame_name_, base_frame_name_, rt2,
                                  ros::Duration(0.2));
    tf_listener_.lookupTransform(odom_frame_name_, base_frame_name_, rt1, tf1);
    tf_listener_.lookupTransform(odom_frame_name_, base_frame_name_, rt2, tf2);
  } catch (tf::TransformException ex) {
    LOG(ERROR) << "Failed to get TF from " << odom_frame_name_ << " to "
               << base_frame_name_.c_str() << ", reason: " << ex.what();
    got_tf = false;
  }

  if (got_tf) {
    gtsam::Pose3 pose1 = TfToPose3(tf1);
    gtsam::Pose3 pose2 = TfToPose3(tf2);
    relative_pose = pose1.between(pose2);
  } else {
    LOG(INFO) << "Will use identity because couldn't get tf.";
  }

  LOG(INFO) << "TFPosePlugin: Adding factor between " << std::string(sym1)
            << " and " << std::string(sym2);
  LOG(INFO) << "TFPosePlugin, relative transform is ("
            << relative_pose.x() << ", " << relative_pose.y() << ", "
            << relative_pose.z() << ")";

  const double trans_noise = translation_noise_;
  const double rot_noise = rotation_noise_;
  gtsam::Vector noise_vector(6);
  noise_vector << roll_noise_, pitch_noise_, yaw_noise_,
                  trans_noise, trans_noise, trans_noise;
  gtsam::SharedDiagonal noise =
      gtsam::noiseModel::Diagonal::Sigmas(noise_vector);
  gtsam::BetweenFactor<gtsam::Pose3>::shared_ptr between(
      new gtsam::BetweenFactor<gtsam::Pose3>(sym1, sym2, relative_pose, noise));
  between->print("TF BetweenFactor:\n");
  return between;
}

bool TFPosePlugin::Ready() {
  return true;
}

}  // namespace omnimapper
