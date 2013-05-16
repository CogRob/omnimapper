#include <omnimapper_ros/omnimapper_visualizer_rviz.h>
#include <geometry_msgs/PoseArray.h>

template <typename PointT>
omnimapper::OmniMapperVisualizerRViz<PointT>::OmniMapperVisualizerRViz (omnimapper::OmniMapperBase* mapper)
  : nh_ ("~"),
    mapper_ (mapper)
{
  pose_array_pub_ = nh_.advertise<geometry_msgs::PoseArray>("trajectory", 0);
}

template <typename PointT> void
omnimapper::OmniMapperVisualizerRViz<PointT>::update (boost::shared_ptr<gtsam::Values>& vis_values)
{
  gtsam::Values current_solution = *vis_values;
  
  geometry_msgs::PoseArray pose_array;
  pose_array.header.frame_id = "/odom";
  pose_array.header.stamp = ros::Time::now ();
  
  gtsam::Values::ConstFiltered<gtsam::Pose3> pose_filtered = current_solution.filter<gtsam::Pose3>();
  BOOST_FOREACH (const gtsam::Values::ConstFiltered<gtsam::Pose3>::KeyValuePair& key_value, pose_filtered)
  {
    geometry_msgs::Pose pose;
    
    gtsam::Symbol key_symbol (key_value.key);
    gtsam::Pose3 sam_pose = key_value.value;
    gtsam::Rot3 rot = sam_pose.rotation ();
    // W X Y Z
    gtsam::Vector quat = rot.quaternion ();
    
    //Eigen::Affine3d eigen_mat (pose.matrix ());
    //tf::Transform tf_pose;
    //tf::transformEigenToTF (eigen_mat, tf_pose);
    // X Y Z W
    tf::Quaternion orientation (quat[1], quat[2], quat[3], quat[0]);
    tf::quaternionTFToMsg (orientation, pose.orientation);
    pose.position.x = sam_pose.x ();
    pose.position.y = sam_pose.y ();
    pose.position.z = sam_pose.z ();
    pose_array.poses.push_back (pose);
  }
  
  pose_array_pub_.publish (pose_array);
}

template class omnimapper::OmniMapperVisualizerRViz<pcl::PointXYZ>;
template class omnimapper::OmniMapperVisualizerRViz<pcl::PointXYZRGBA>;
