#include <omnimapper_ros/omnimapper_visualizer_rviz.h>
#include <omnimapper_ros/ros_time_utils.h>
#include <geometry_msgs/PoseArray.h>
#include <pcl/common/transforms.h>
#include <omnimapper/plane.h>
#include <pcl_conversions/pcl_conversions.h>

template <typename PointT>
omnimapper::OmniMapperVisualizerRViz<PointT>::OmniMapperVisualizerRViz (omnimapper::OmniMapperBase* mapper)
  : nh_ ("~"),
    mapper_ (mapper),
    draw_icp_clouds_ (false),
    draw_planar_landmarks_ (true),
    draw_pose_array_ (true),
    draw_object_observation_cloud_ (true),
    draw_object_observation_bboxes_ (true)
{
  //tf::Transform

  pose_array_pub_ = nh_.advertise<geometry_msgs::PoseArray>("trajectory", 0);

  map_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("map_cloud", 0);

  planar_boundary_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("planar_boundaries", 0);
  
  marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray> ("/visualization_marker_array", 0);

  segmented_plane_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("segmented_planes", 0);

  segmented_label_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("segmented_label_cloud", 0);

  segmented_clusters_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("segmented_clusters", 0);

  object_observation_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("object_observations", 0);

  draw_icp_clouds_srv_ = nh_.advertiseService ("draw_icp_clouds", &omnimapper::OmniMapperVisualizerRViz<PointT>::drawICPCloudsCallback, this);

  draw_object_observation_cloud_srv_ = nh_.advertiseService ("draw_object_observations", &omnimapper::OmniMapperVisualizerRViz<PointT>::drawObjectObservationCloud, this);

}

template <typename PointT> void
omnimapper::OmniMapperVisualizerRViz<PointT>::drawBBox (pcl::PointCloud<pcl::PointXYZRGB>& cloud, ros::Publisher& marker_pub_, int obj_idx)
{
  // Get bbox points
  Eigen::Vector4f min_pt;
  Eigen::Vector4f max_pt;
  pcl::getMinMax3D (cloud, min_pt, max_pt);
  
  geometry_msgs::Point p1;
  p1.x = min_pt[0];
  p1.y = min_pt[1];
  p1.z = min_pt[2];
  geometry_msgs::Point p2;
  p2.x = max_pt[0];
  p2.y = min_pt[1];
  p2.z = min_pt[2];
  geometry_msgs::Point p3;
  p3.x = min_pt[0];
  p3.y = max_pt[1];
  p3.z = min_pt[2];
  geometry_msgs::Point p4;
  p4.x = max_pt[0];
  p4.y = max_pt[1];
  p4.z = min_pt[2];

  geometry_msgs::Point p5;
  p5.x = min_pt[0];
  p5.y = min_pt[1];
  p5.z = max_pt[2];
  geometry_msgs::Point p6;
  p6.x = max_pt[0];
  p6.y = min_pt[1];
  p6.z = max_pt[2];
  geometry_msgs::Point p7;
  p7.x = min_pt[0];
  p7.y = max_pt[1];
  p7.z = max_pt[2];
  geometry_msgs::Point p8;
  p8.x = max_pt[0];
  p8.y = max_pt[1];
  p8.z = max_pt[2];
  

  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker bbox_marker;
  bbox_marker.header.frame_id = "/world";
  bbox_marker.header.stamp = ros::Time::now ();
  bbox_marker.ns = "object_bboxes";
  bbox_marker.id = obj_idx;
  bbox_marker.type = visualization_msgs::Marker::LINE_LIST;
  bbox_marker.action = visualization_msgs::Marker::ADD;

  bbox_marker.points.push_back (p1);
  bbox_marker.points.push_back (p2);
  bbox_marker.points.push_back (p1);
  bbox_marker.points.push_back (p3);
  bbox_marker.points.push_back (p2);
  bbox_marker.points.push_back (p4);
  bbox_marker.points.push_back (p3);
  bbox_marker.points.push_back (p4);

  bbox_marker.points.push_back (p5);
  bbox_marker.points.push_back (p6);
  bbox_marker.points.push_back (p5);
  bbox_marker.points.push_back (p7);
  bbox_marker.points.push_back (p6);
  bbox_marker.points.push_back (p8);
  bbox_marker.points.push_back (p7);
  bbox_marker.points.push_back (p8);

  bbox_marker.points.push_back (p1);
  bbox_marker.points.push_back (p5);
  bbox_marker.points.push_back (p2);
  bbox_marker.points.push_back (p6);
  bbox_marker.points.push_back (p3);
  bbox_marker.points.push_back (p7);
  bbox_marker.points.push_back (p4);
  bbox_marker.points.push_back (p8);

  bbox_marker.scale.x = 0.01;
  bbox_marker.color.a = 0.2;
  bbox_marker.color.r = 1.0;
  bbox_marker.color.g = 1.0;
  bbox_marker.color.b = 1.0;
  marker_array.markers.push_back (bbox_marker);
  marker_pub_.publish (marker_array);
  
}


template <typename PointT> void
omnimapper::OmniMapperVisualizerRViz<PointT>::update (boost::shared_ptr<gtsam::Values>& vis_values, boost::shared_ptr<gtsam::NonlinearFactorGraph>& vis_graph)
{
  gtsam::Values current_solution = *vis_values;
  
  // Draw the cloud
  CloudPtr aggregate_cloud (new Cloud ());
  //aggregate_cloud->header.frame_id = "/world";
  //aggregate_cloud->header.stamp = ros::Time::now ();
  
  // Draw object cloud
  pcl::PointCloud<pcl::PointXYZRGB> aggregate_object_observation_cloud;// (new pcl::PointCloud<pcl::PointXYZRGB> ());
  //aggregate_object_observation_cloud.header.frame_id = "/world";
  //aggregate_object_observation_cloud.header.stamp = ros::Time::now ();

  geometry_msgs::PoseArray pose_array;
  pose_array.header.frame_id = "/world";
  pose_array.header.stamp = ros::Time::now ();
  
  unsigned char red [6] = {255,   0,   0, 255, 255,   0};
  unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
  unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};
  int obj_id = 0;
  
  gtsam::Values::ConstFiltered<gtsam::Pose3> pose_filtered = current_solution.filter<gtsam::Pose3>();
  BOOST_FOREACH (const gtsam::Values::ConstFiltered<gtsam::Pose3>::KeyValuePair& key_value, pose_filtered)
  {
    geometry_msgs::Pose pose;
    
    gtsam::Symbol key_symbol (key_value.key);
    gtsam::Pose3 sam_pose = key_value.value;
    gtsam::Rot3 rot = sam_pose.rotation ();
    int pose_idx = key_symbol.index ();
    // W X Y Z
    gtsam::Vector quat = rot.quaternion ();
    Eigen::Matrix4f map_tform = sam_pose.matrix ().cast<float>();

    //Eigen::Affine3d eigen_mat (pose.matrix ());
    //tf::Transform tf_pose;
    //tf::transformEigenToTF (eigen_mat, tf_pose);
    // X Y Z W
    tf::Quaternion orientation (quat[1], quat[2], quat[3], quat[0]);
    tf::quaternionTFToMsg (orientation, pose.orientation);
    pose.position.x = sam_pose.x ();
    pose.position.y = sam_pose.y ();
    pose.position.z = sam_pose.z ();
    if (draw_pose_array_)
      pose_array.poses.push_back (pose);

    // Optionally Draw clouds too
    if (draw_icp_clouds_)
    {
      CloudConstPtr frame_cloud = icp_plugin_->getCloudPtr (key_symbol);
      //CloudConstPtr frame_cloud = icp_plugin_->getFullResCloudPtr (key_symbol);
      char frame_name[1024];
      CloudPtr map_cloud (new Cloud ());
      //pose.print ("SAM Pose: ");
      std::cout << "Map Tform: " << map_tform << std::endl;
      pcl::transformPointCloud (*frame_cloud, *map_cloud, map_tform);
      sprintf (frame_name, "x_%d", key_symbol.index ());
      printf ("name: x_%d\n",key_symbol.index ());
      (*aggregate_cloud) += (*map_cloud);
    }
    
    // Optionally draw object observations
    if (draw_object_observation_cloud_)
    {
      // Get the observations from this location
      std::vector<CloudPtr> obs_clouds = object_plugin_->getObservations (key_symbol);

      pcl::PointCloud<pcl::PointXYZRGB> cluster;
      
      for (int i = 0; i < obs_clouds.size (); i++)
      {
        // Get the cluster
        pcl::copyPointCloud ((*(obs_clouds[i])), cluster);

        for (int j = 0; j < cluster.points.size (); j++)
        {
          cluster.points[j].r = (cluster.points[j].r + red[i%6]) / 2;
          cluster.points[j].g = (cluster.points[j].g + grn[i%6]) / 2;
          cluster.points[j].b = (cluster.points[j].b + blu[i%6]) / 2;
        }

        // Move it to the map frame
        pcl::transformPointCloud (cluster, cluster, map_tform);

        // Optionally draw a bbox too
        if (draw_object_observation_bboxes_)
        {
          drawBBox (cluster, marker_array_pub_, ++obj_id);
        }

        aggregate_object_observation_cloud += cluster;
      }

    }

  }
  
  // Publish the poses
  if (draw_pose_array_)
    pose_array_pub_.publish (pose_array);

  // Optionally publish the ICP Clouds
  if (draw_icp_clouds_)
  {
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg (*aggregate_cloud, cloud_msg);
    //pcl_conversions::moveFromPCL (*aggregate_cloud, cloud_msg);
    cloud_msg.header.frame_id = "world";
    cloud_msg.header.stamp = ros::Time::now ();
    map_cloud_pub_.publish (cloud_msg);
    draw_icp_clouds_ = false;
  }

  // Draw object observations
  if (draw_object_observation_cloud_)
  {
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg (aggregate_object_observation_cloud, cloud_msg);
    //pcl_conversions::moveFromPCL (aggregate_object_observation_cloud, cloud_msg);
    cloud_msg.header.frame_id = "world";
    cloud_msg.header.stamp = ros::Time::now ();
    object_observation_pub_.publish (cloud_msg);
    draw_object_observation_cloud_ = false;
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
      //pcl_conversions::moveFromPCL (*plane_boundary_cloud, cloud_msg);
      cloud_msg.header.frame_id = "/world";
      cloud_msg.header.stamp = ros::Time::now ();
      planar_boundary_pub_.publish (cloud_msg);
    }
    
  }
  
}

template <typename PointT> void
omnimapper::OmniMapperVisualizerRViz<PointT>::planarRegionCallback (std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions, omnimapper::Time t)
{
  // Display the segmented planar regions
  pcl::PointCloud<PointT> aggregate_cloud;
  for (int i = 0; i < regions.size (); i++)
  {
    pcl::PointCloud<PointT> border_cloud;
    std::vector<PointT, Eigen::aligned_allocator<PointT> > border = regions[i].getContour ();
    border_cloud.points = border;
    aggregate_cloud += border_cloud;
  }
  
  if (aggregate_cloud.points.size () > 0)
  {
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg (aggregate_cloud, cloud_msg);
    //pcl_conversions::moveFromPCL (aggregate_cloud, cloud_msg);
    cloud_msg.header.frame_id = "/camera_rgb_optical_frame";
    cloud_msg.header.stamp = ptime2rostime (t);
    segmented_plane_pub_.publish (cloud_msg);
  }
  
}

template <typename PointT> void
omnimapper::OmniMapperVisualizerRViz<PointT>::labelCloudCallback (const CloudConstPtr& cloud, const LabelCloudConstPtr& labels)
{
  // Create a colored label cloud
  pcl::PointCloud<pcl::PointXYZRGB> labeled_cloud;
  //labeled_cloud = (*cloud);
  pcl::copyPointCloud (*cloud, labeled_cloud);

  unsigned char red [6] = {255,   0,   0, 255, 255,   0};
  unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
  unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};
  
  for (int i = 0; i < labeled_cloud.points.size (); i++)
  {
    labeled_cloud.points[i].r = (labeled_cloud.points[i].r + red[labels->points[i].label%6]) / 2;
    labeled_cloud.points[i].g = (labeled_cloud.points[i].g + grn[labels->points[i].label%6]) / 2;
    labeled_cloud.points[i].b = (labeled_cloud.points[i].b + blu[labels->points[i].label%6]) / 2;
  }
  
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg (labeled_cloud, cloud_msg);
  //pcl_conversions::moveFromPCL (labeled_cloud, cloud_msg);
  cloud_msg.header.frame_id = cloud->header.frame_id;
  cloud_msg.header.stamp = ptime2rostime (stamp2ptime (cloud->header.stamp));
  segmented_label_cloud_pub_.publish (cloud_msg);
}

template <typename PointT> void
omnimapper::OmniMapperVisualizerRViz<PointT>::clusterCloudCallback (std::vector<CloudPtr> clusters, omnimapper::Time t)
{
  printf ("Omnimappervisualizerrviz: Got %d clusters\n", clusters.size ());
  
  if (clusters.size () == 0)
    return;

  pcl::PointCloud<pcl::PointXYZRGB> aggregate_cloud;
  
  unsigned char red [6] = {255,   0,   0, 255, 255,   0};
  unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
  unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};
  
  
  // color the clouds
  pcl::PointCloud<pcl::PointXYZRGB> color_cluster;
  for (int i = 0; i < clusters.size (); i++)
  {
    //if (clusters[i]->points.size () > 0)
    //aggregate_cloud += ((*(clusters[i])));
    

    if (clusters[i]->points.size () > 0)
    {
      printf ("Cluster %d has %d points\n", i, clusters[i]->points.size ());
      color_cluster.resize (clusters[i]->points.size ());
      pcl::copyPointCloud ((*(clusters[i])), color_cluster);
      for (int j = 0; j < color_cluster.points.size (); j++)
      {
        color_cluster.points[j].r = (color_cluster.points[j].r + red[i%6]) / 2;
        color_cluster.points[j].g = (color_cluster.points[j].g + grn[i%6]) / 2;
        color_cluster.points[j].b = (color_cluster.points[j].b + blu[i%6]) / 2;
      }
    
       try
       {
         aggregate_cloud += color_cluster;
       }
       catch (std::bad_alloc& ba)
       {
         std::cerr << "bad_alloc caught in omnimapper_rviz: " << ba.what () << std::endl;
       }
     }
  }

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg (aggregate_cloud, cloud_msg);
  //moveFromPCL (aggregate_cloud, cloud_msg);
  cloud_msg.header.frame_id = "/camera_rgb_optical_frame";
  cloud_msg.header.stamp = ptime2rostime (t);
  segmented_clusters_pub_.publish (cloud_msg);
}


// template <typename PointT> void
// clusterLabelsCallback (const CloudConstPtr& cloud, const LabelCloudConstPtr& labels)

template <typename PointT> bool 
omnimapper::OmniMapperVisualizerRViz<PointT>::drawICPCloudsCallback (omnimapper_ros::VisualizeFullCloud::Request &req, omnimapper_ros::VisualizeFullCloud::Response &res)
{
  draw_icp_clouds_ = true;
  return (true);
}

template <typename PointT> bool
omnimapper::OmniMapperVisualizerRViz<PointT>::drawObjectObservationCloud (omnimapper_ros::VisualizeFullCloud::Request &req, omnimapper_ros::VisualizeFullCloud::Response &res)
{
  draw_object_observation_cloud_ = true;
  return (true);
}


//template class omnimapper::OmniMapperVisualizerRViz<pcl::PointXYZ>;
template class omnimapper::OmniMapperVisualizerRViz<pcl::PointXYZRGBA>;
