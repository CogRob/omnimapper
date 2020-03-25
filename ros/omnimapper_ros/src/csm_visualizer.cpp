#include <geometry_msgs/PoseArray.h>
#include <omnimapper_ros/csm_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>

template <typename LScanT>
omnimapper::CSMVisualizerRViz<LScanT>::CSMVisualizerRViz(
    omnimapper::OmniMapperBase* mapper)
    : nh_("~"),
      mapper_(mapper),
      vis_values_(new gtsam::Values()),
      vis_graph_(new gtsam::NonlinearFactorGraph()),
      draw_graph_(true),
      draw_map_(true) {
  pose_array_pub_ = nh_.advertise<geometry_msgs::PoseArray>("trajectory", 0);

  marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "/visualization_marker_array", 0);

  map_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("csm_map_cloud", 0);

  draw_csm_map_srv_ = nh_.advertiseService(
      "draw_csm_map", &omnimapper::CSMVisualizerRViz<LScanT>::drawCSMMap, this);

  // draw_icp_clouds_srv_ = nh_.advertiseService ("draw_icp_clouds",
  // &omnimapper::OmniMapperVisualizerRViz<PointT>::drawICPCloudsCallback, this);
}

template <typename LScanT>
void omnimapper::CSMVisualizerRViz<LScanT>::update(
    boost::shared_ptr<gtsam::Values>& vis_values,
    boost::shared_ptr<gtsam::NonlinearFactorGraph>& vis_graph) {
  {
    boost::lock_guard<boost::mutex> lock(vis_mutex_);
    vis_values_ = vis_values;
    vis_graph_ = vis_graph;
  }

  gtsam::Values current_solution = *vis_values;
  gtsam::NonlinearFactorGraph current_graph = *vis_graph;

  geometry_msgs::PoseArray pose_array;
  pose_array.header.frame_id = "/world";
  pose_array.header.stamp = ros::Time::now();

  pcl::PointCloud<pcl::PointXYZ>::Ptr aggregate_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  aggregate_cloud->header.frame_id = "/world";
  // aggregate_cloud->header.stamp = ros::Time::now ();

  gtsam::Values::ConstFiltered<gtsam::Pose3> pose_filtered =
      current_solution.filter<gtsam::Pose3>();
  BOOST_FOREACH (
      const gtsam::Values::ConstFiltered<gtsam::Pose3>::KeyValuePair& key_value,
      pose_filtered) {
    geometry_msgs::Pose pose;

    gtsam::Symbol key_symbol(key_value.key);
    gtsam::Pose3 sam_pose = key_value.value;
    gtsam::Rot3 rot = sam_pose.rotation();
    // W X Y Z
    gtsam::Vector quat = rot.quaternion();

    // Eigen::Affine3d eigen_mat (pose.matrix ());
    // tf::Transform tf_pose;
    // tf::transformEigenToTF (eigen_mat, tf_pose);
    // X Y Z W
    tf::Quaternion orientation(quat[1], quat[2], quat[3], quat[0]);
    tf::quaternionTFToMsg(orientation, pose.orientation);
    pose.position.x = sam_pose.x();
    pose.position.y = sam_pose.y();
    pose.position.z = sam_pose.z();
    pose_array.poses.push_back(pose);

    if (draw_map_) {
      // Draw the scans too
      sensor_msgs::PointCloud2 cloud_msg = csm_plugin_->getPC2(key_symbol);
      if (cloud_msg.width > 0) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(cloud_msg, *cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(
            new pcl::PointCloud<pcl::PointXYZ>);
        Eigen::Matrix4f map_tform = sam_pose.matrix().cast<float>();
        pcl::transformPointCloud(*cloud, *map_cloud, map_tform);
        (*aggregate_cloud) += (*map_cloud);
      }
    }
  }

  pose_array_pub_.publish(pose_array);

  if (draw_map_) {
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*aggregate_cloud, cloud_msg);
    cloud_msg.header.frame_id = "world";
    cloud_msg.header.stamp = ros::Time::now();
    map_cloud_pub_.publish(cloud_msg);
    // draw_map_ = false;
  }

  // Draw the graph
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker mapper_graph;
  mapper_graph.header.frame_id = "/world";
  mapper_graph.header.stamp = ros::Time();
  mapper_graph.ns = "error_lines";
  mapper_graph.id = 0;
  mapper_graph.type = visualization_msgs::Marker::LINE_LIST;
  mapper_graph.action = visualization_msgs::Marker::ADD;
  mapper_graph.color.a = 0.5;
  mapper_graph.color.r = 1.0;
  mapper_graph.color.g = 0.0;
  mapper_graph.color.b = 0.0;
  mapper_graph.scale.x = 0.01;

  BOOST_FOREACH (const gtsam::NonlinearFactorGraph::sharedFactor& factor,
                 current_graph) {
    // check for poses
    const std::vector<gtsam::Key> keys = factor->keys();

    // skip if there aren't two pose keys
    if ((keys.size() == 2)) {
      if ((gtsam::symbolChr(keys[0]) == 'x') &&
          (gtsam::symbolChr(keys[1]) == 'x')) {
        gtsam::Pose3 p1 = current_solution.at<gtsam::Pose3>(keys[0]);
        gtsam::Pose3 p2 = current_solution.at<gtsam::Pose3>(keys[1]);

        geometry_msgs::Point p1_msg;
        p1_msg.x = p1.x();
        p1_msg.y = p1.y();
        p1_msg.z = p1.z();

        geometry_msgs::Point p2_msg;
        p2_msg.x = p2.x();
        p2_msg.y = p2.y();
        p2_msg.z = p2.z();

        mapper_graph.points.push_back(p1_msg);
        mapper_graph.points.push_back(p2_msg);
      }
    }
  }

  marker_array.markers.push_back(mapper_graph);
  marker_array_pub_.publish(marker_array);
}

template <typename LScanT>
bool omnimapper::CSMVisualizerRViz<LScanT>::drawCSMMap(
    omnimapper_ros::VisualizeFullCloud::Request& req,
    omnimapper_ros::VisualizeFullCloud::Response& res) {
  // gtsam::Values current_solution;
  // gtsam::NonlinearFactorGraph current_graph;

  // {
  //   boost::lock_guard<boost::mutex> lock (vis_mutex_);
  //   current_solution = vis_values_;
  //   current_graph = vis_graph_;
  // }

  // gtsam::Values::ConstFiltered<gtsam::Pose3> pose_filtered =
  // current_solution.filter<gtsam::Pose3>();

  // int pose_num = 0;
  // BOOST_FOREACH (const
  // gtsam::Values::ConstFiltered<gtsam::Pose3>::KeyValuePair& key_value,
  // pose_filtered)
  // {
  //   gtsam::Symbol key_symbol (key_value.key);
  //   gtsam::Pose3 sam_pose = key_value.value;
  // }
}

// template <typename LScanT> bool
// omnimapper::CSMVisualizerRViz<LScanT>::drawICPCloudsCallback
// (omnimapper_ros::VisualizeFullCloud::Request &req,
// omnimapper_ros::VisualizeFullCloud::Response &res)
// {
//   draw_icp_clouds_ = true;
//   return (true);
// }

template class omnimapper::CSMVisualizerRViz<sensor_msgs::LaserScan>;
