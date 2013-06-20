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
  
  marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray> ("/visualization_marker_array", 0);

  draw_icp_clouds_srv_ = nh_.advertiseService ("draw_icp_clouds", &omnimapper::OmniMapperVisualizerRViz<PointT>::drawICPCloudsCallback, this);
}

template <typename PointT> void
omnimapper::OmniMapperVisualizerRViz<PointT>::update (boost::shared_ptr<gtsam::Values>& vis_values, boost::shared_ptr<gtsam::NonlinearFactorGraph>& vis_graph)
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
      //CloudConstPtr frame_cloud = icp_plugin_->getFullResCloudPtr (key_symbol);
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
    visualization_msgs::MarkerArray marker_array;
    CloudPtr plane_boundary_cloud (new Cloud ());
    gtsam::Values::ConstFiltered<gtsam::Plane<PointT> > plane_filtered = current_solution.filter<gtsam::Plane<PointT> >();
    int id = 0;
    BOOST_FOREACH (const typename gtsam::Values::ConstFiltered<gtsam::Plane<PointT> >::KeyValuePair& key_value, plane_filtered)
    {
      // Draw the boundary
      Cloud lm_cloud = key_value.value.hull ();
      (*plane_boundary_cloud) += lm_cloud;
      
      for (int i = 0; i < lm_cloud.points.size (); i++)
      {
        if (!pcl::isFinite (lm_cloud.points[i]))
          printf ("Error!  Point is not finite!\n");
      }

      Eigen::Vector4f centroid;
      pcl::compute3DCentroid (lm_cloud, centroid);
      
      printf ("RViz Plugin: Cloud had %d points\n", lm_cloud.points.size ());
      printf ("RViz Plugin Centroid: %lf %lf %lf\n", centroid[0], centroid[1], centroid[2]);

      geometry_msgs::Point start;
      start.x = centroid[0];
      start.y = centroid[1];
      start.z = centroid[2];
      geometry_msgs::Point end;
      end.x = centroid[0] + key_value.value.a ();
      end.y = centroid[1] + key_value.value.b ();
      end.z = centroid[2] + key_value.value.c ();
      
      // Draw the normal
      visualization_msgs::Marker normal_marker;
      normal_marker.header.frame_id = "/world";
      normal_marker.header.stamp = ros::Time();
      normal_marker.ns = "planar_normals";
      normal_marker.id = ++id;//key_value.key.index ();
      normal_marker.type = visualization_msgs::Marker::ARROW;
      normal_marker.action = visualization_msgs::Marker::ADD;
      normal_marker.points.push_back (start);
      normal_marker.points.push_back (end);
      normal_marker.pose.position.x = 0.0;
      normal_marker.pose.position.y = 0.0;
      normal_marker.pose.position.z = 0.0;
      normal_marker.pose.orientation.x = 0.0;
      normal_marker.pose.orientation.y = 0.0;
      normal_marker.pose.orientation.z = 0.0;
      normal_marker.pose.orientation.w = 1.0;
      normal_marker.scale.x = 0.025;
      normal_marker.scale.y = 0.05;
      normal_marker.scale.z = 0.1;
      normal_marker.color.a = 0.5;
      normal_marker.color.r = 1.0;
      normal_marker.color.g = 0.0;
      normal_marker.color.b = 0.0;
      marker_array.markers.push_back (normal_marker);
    }
    
    //marker_array.header.stamp = ros::Time::now ();
    //marker_array.header.frame_id = "/odom";
    marker_array_pub_.publish (marker_array);
    
    if (plane_boundary_cloud->points.size () > 0)
    {
      sensor_msgs::PointCloud2 cloud_msg;
      pcl::toROSMsg (*plane_boundary_cloud, cloud_msg);
      cloud_msg.header.frame_id = "/world";
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
