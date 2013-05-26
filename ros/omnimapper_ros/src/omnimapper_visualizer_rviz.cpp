#include <omnimapper_ros/omnimapper_visualizer_rviz.h>
#include <geometry_msgs/PoseArray.h>
#include <pcl/common/transforms.h>
#include <omnimapper/plane.h>

template <typename PointT>
omnimapper::OmniMapperVisualizerRViz<PointT>::OmniMapperVisualizerRViz (omnimapper::OmniMapperBase* mapper)
  : nh_ ("~"),
    mapper_ (mapper),
    draw_icp_clouds_ (false),
    draw_planar_landmarks_ (true)
{
  //tf::Transform

  pose_array_pub_ = nh_.advertise<geometry_msgs::PoseArray>("trajectory", 0);

  map_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("map_cloud", 0);

  planar_boundary_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("planar_boundaries", 0);

  draw_icp_clouds_srv_ = nh_.advertiseService ("draw_icp_clouds", &omnimapper::OmniMapperVisualizerRViz<PointT>::drawICPCloudsCallback, this);
}

template <typename PointT> void
omnimapper::OmniMapperVisualizerRViz<PointT>::update (boost::shared_ptr<gtsam::Values>& vis_values)
{
  gtsam::Values current_solution = *vis_values;
  
  // Draw the cloud
  CloudPtr aggregate_cloud (new Cloud ());
  aggregate_cloud->header.frame_id = "/world";
  aggregate_cloud->header.stamp = ros::Time::now ();
  
  geometry_msgs::PoseArray pose_array;
  pose_array.header.frame_id = "/world";
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

    // Optionally Draw clouds too
    if (draw_icp_clouds_)
    {
      CloudConstPtr frame_cloud = icp_plugin_->getCloudPtr (key_symbol);
      char frame_name[1024];
      CloudPtr map_cloud (new Cloud ());
      Eigen::Matrix4f map_tform = sam_pose.matrix ().cast<float>();
      //pose.print ("SAM Pose: ");
      std::cout << "Map Tform: " << map_tform << std::endl;
      pcl::transformPointCloud (*frame_cloud, *map_cloud, map_tform);
      sprintf (frame_name, "x_%d", key_symbol.index ());
      printf ("name: x_%d\n",key_symbol.index ());
      (*aggregate_cloud) += (*map_cloud);
    }
  }
  
  // Publish the poses
  pose_array_pub_.publish (pose_array);

  // Optionally publish the ICP Clouds
  if (draw_icp_clouds_)
  {
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg (*aggregate_cloud, cloud_msg);
    cloud_msg.header.frame_id = "world";
    cloud_msg.header.stamp = ros::Time::now ();
    map_cloud_pub_.publish (cloud_msg);
    draw_icp_clouds_ = false;
  }

  // Draw Planar Landmarks
  if (draw_planar_landmarks_)
  {
    CloudPtr plane_boundary_cloud (new Cloud ());
    gtsam::Values::ConstFiltered<gtsam::Plane<PointT> > plane_filtered = current_solution.filter<gtsam::Plane<PointT> >();
    BOOST_FOREACH (const typename gtsam::Values::ConstFiltered<gtsam::Plane<PointT> >::KeyValuePair& key_value, plane_filtered)
    {
      Cloud lm_cloud = key_value.value.hull ();
      
      (*plane_boundary_cloud) += lm_cloud;
    }
    
    if (plane_boundary_cloud->points.size () > 0)
    {
      sensor_msgs::PointCloud2 cloud_msg;
      pcl::toROSMsg (*plane_boundary_cloud, cloud_msg);
      cloud_msg.header.frame_id = "world";
      cloud_msg.header.stamp = ros::Time::now ();
      planar_boundary_pub_.publish (cloud_msg);
    }
    
  }
  
}

template <typename PointT> bool 
omnimapper::OmniMapperVisualizerRViz<PointT>::drawICPCloudsCallback (omnimapper_ros::VisualizeFullCloud::Request &req, omnimapper_ros::VisualizeFullCloud::Response &res)
{
  draw_icp_clouds_ = true;
  return (true);
}


template class omnimapper::OmniMapperVisualizerRViz<pcl::PointXYZ>;
template class omnimapper::OmniMapperVisualizerRViz<pcl::PointXYZRGBA>;
