#include <omnimapper_ros/tum_data_error_plugin.h>

omnimapper::TUMDataErrorPlugin::TUMDataErrorPlugin(
    omnimapper::OmniMapperBase* mapper)
    : nh_("~"), tf_listener_(ros::Duration(180.0)), mapper_(mapper) {
  marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "/visualization_marker_array", 0);
}

void omnimapper::TUMDataErrorPlugin::Update(
    boost::shared_ptr<gtsam::Values>& vis_values,
    boost::shared_ptr<gtsam::NonlinearFactorGraph>& vis_graph) {
  printf("Updating  TUM ERROR Plugin!\n");
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker mapper_trajectory;
  mapper_trajectory.header.frame_id = "/world";
  mapper_trajectory.header.stamp = ros::Time();
  mapper_trajectory.ns = "mapper_trajectory";
  mapper_trajectory.id = 0;
  mapper_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
  mapper_trajectory.action = visualization_msgs::Marker::ADD;
  mapper_trajectory.color.a = 0.5;
  mapper_trajectory.color.r = 0.0;
  mapper_trajectory.color.g = 0.0;
  mapper_trajectory.color.b = 1.0;
  mapper_trajectory.scale.x = 0.01;

  visualization_msgs::Marker true_trajectory;
  true_trajectory.header.frame_id = "/world";
  true_trajectory.header.stamp = ros::Time();
  true_trajectory.ns = "true_trajectory";
  true_trajectory.id = 0;
  true_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
  true_trajectory.action = visualization_msgs::Marker::ADD;
  true_trajectory.color.a = 0.5;
  true_trajectory.color.r = 0.0;
  true_trajectory.color.g = 1.0;
  true_trajectory.color.b = 0.0;
  true_trajectory.scale.x = 0.01;

  visualization_msgs::Marker error_lines;
  error_lines.header.frame_id = "/world";
  error_lines.header.stamp = ros::Time();
  error_lines.ns = "error_lines";
  error_lines.id = 0;
  error_lines.type = visualization_msgs::Marker::LINE_LIST;
  error_lines.action = visualization_msgs::Marker::ADD;
  error_lines.color.a = 0.5;
  error_lines.color.r = 1.0;
  error_lines.color.g = 0.0;
  error_lines.color.b = 0.0;
  error_lines.scale.x = 0.01;

  // Draw the mapper trajectory, true trajectory, and error
  double total_error = 0.0;
  int count = 0;
  gtsam::Values::ConstFiltered<gtsam::Pose3> pose_filtered =
      vis_values->filter<gtsam::Pose3>();
  BOOST_FOREACH (
      const gtsam::Values::ConstFiltered<gtsam::Pose3>::KeyValuePair& key_value,
      pose_filtered) {
    // Skip the first, this seems to be out of order
    if (count == 0) {
      count++;
      continue;
    }

    geometry_msgs::Pose pose;

    gtsam::Symbol key_symbol(key_value.key);
    gtsam::Pose3 sam_pose = key_value.value;
    gtsam::Rot3 rot = sam_pose.rotation();
    // W X Y Z
    gtsam::Vector quat = rot.quaternion();

    geometry_msgs::Point point_msg;
    point_msg.x = sam_pose.x();
    point_msg.y = sam_pose.y();
    point_msg.z = sam_pose.z();
    mapper_trajectory.points.push_back(point_msg);
    printf("TUM Error plugin: Added pose!\n");

    // Get the true position at this timestamp
    boost::posix_time::ptime pose_ptime;
    tf::StampedTransform true_pose;
    printf("TUM Error plugin: Looking up time!\n");
    mapper_->GetTimeAtPoseSymbol(key_symbol, pose_ptime);
    printf("TUM Error plugin: got the time!\n");
    ros::Time pose_time = PtimeToRosTime(pose_ptime);
    tf_listener_.waitForTransform("/world", "/openni_rgb_optical_frame",
                                  pose_time, ros::Duration(0.2));
    tf_listener_.lookupTransform("/world", "/openni_rgb_optical_frame",
                                 pose_time, true_pose);

    geometry_msgs::Point true_point_msg;
    true_point_msg.x = true_pose.getOrigin().x();
    true_point_msg.y = true_pose.getOrigin().y();
    true_point_msg.z = true_pose.getOrigin().z();
    true_trajectory.points.push_back(true_point_msg);
    printf("adding point: %lf %lf %lf\n", true_point_msg.x, true_point_msg.y,
           true_point_msg.z);

    // Draw error line between these in red
    error_lines.points.push_back(point_msg);
    error_lines.points.push_back(true_point_msg);

    // Compute error
    double dx = sam_pose.x() - true_point_msg.x;
    double dy = sam_pose.y() - true_point_msg.y;
    double dz = sam_pose.z() - true_point_msg.z;
    double euclidean_error = sqrt(dx * dx + dy * dy + dz * dz);
    printf("euclidean error: %lf\n", euclidean_error);
    total_error += euclidean_error;

    count++;
  }

  double avg_euclidean_error = (total_error / ((double)count));
  printf("total euclidean error: %lf\n", avg_euclidean_error);

  marker_array.markers.push_back(mapper_trajectory);
  marker_array.markers.push_back(true_trajectory);
  marker_array.markers.push_back(error_lines);

  marker_array_pub_.publish(marker_array);
}
