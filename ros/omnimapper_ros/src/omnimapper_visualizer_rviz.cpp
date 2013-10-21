#include <omnimapper_ros/omnimapper_visualizer_rviz.h>
#include <omnimapper_ros/ros_time_utils.h>
#include <geometry_msgs/PoseArray.h>
#include <pcl/common/transforms.h>
#include <omnimapper/plane.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/btMatrix3x3.h>

template <typename PointT>
omnimapper::OmniMapperVisualizerRViz<PointT>::OmniMapperVisualizerRViz (omnimapper::OmniMapperBase* mapper)
  : nh_ ("~"),
    mapper_ (mapper),
    marker_server_ (new interactive_markers::InteractiveMarkerServer ("OmniMapper", "", false)),
    menu_handler_ (new interactive_markers::MenuHandler ()),
    draw_icp_clouds_ (false),
    draw_planar_landmarks_ (true),
    draw_pose_array_ (true),
    draw_pose_graph_ (true),
    draw_object_observation_cloud_ (true),
    draw_object_observation_bboxes_ (true),
    draw_pose_marginals_ (true)
{
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

  pose_covariances_pub_ = nh_.advertise<visualization_msgs::MarkerArray> ("/pose_covariances", 0);

  object_modeled_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("modelled_objects", 0);

  initMenu ();
}

template <typename PointT> void
omnimapper::OmniMapperVisualizerRViz<PointT>::initMenu ()
{
  // Create a control marker at the map origin for map level controls
  visualization_msgs::InteractiveMarker origin_int_marker;
  origin_int_marker.header.frame_id = "/world";
  origin_int_marker.name = "OmniMapper";

  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::CUBE;
  box_marker.scale.x = 0.1;
  box_marker.scale.y = 0.1;
  box_marker.scale.z = 0.1;
  box_marker.color.r = 0.5;
  box_marker.color.g = 0.5;
  box_marker.color.b = 0.5;
  box_marker.color.a = 1.0;
  
  visualization_msgs::InteractiveMarkerControl control;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  control.always_visible = true;
  control.markers.push_back (box_marker);
  origin_int_marker.controls.push_back (control);  
  marker_server_->insert (origin_int_marker);

  // Generate a playback control menu
  playback_menu_ = menu_handler_->insert ("Playback Control");
  interactive_markers::MenuHandler::EntryHandle play_pause = menu_handler_->insert (playback_menu_, "Play / Pause", boost::bind (&omnimapper::OmniMapperVisualizerRViz<PointT>::playPauseCb, this, _1));

  // Generate an output menu
  visualization_menu_ = menu_handler_->insert ("Visualization");
  interactive_markers::MenuHandler::EntryHandle map_cloud = menu_handler_->insert (visualization_menu_, "Draw Map Cloud", boost::bind (&omnimapper::OmniMapperVisualizerRViz<PointT>::drawMapCloudCb, this, _1));

  menu_handler_->apply (*marker_server_, "OmniMapper");
  marker_server_->applyChanges ();
}

template <typename PointT> void
omnimapper::OmniMapperVisualizerRViz<PointT>::playPauseCb (const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  printf ("Toggling playback!");
  
}

template <typename PointT> void
omnimapper::OmniMapperVisualizerRViz<PointT>::drawMapCloudCb (const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  printf ("Drawing map cloud!\n");
  draw_icp_clouds_ = true;
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
  gtsam::NonlinearFactorGraph current_graph = *vis_graph;

  // Draw the cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr aggregate_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
  //CloudPtr aggregate_cloud (new Cloud ());
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
  
  gtsam::Values::ConstFiltered<gtsam::Point3> object_filtered = current_solution.filter<gtsam::Point3>();
  BOOST_FOREACH (const gtsam::Values::ConstFiltered<gtsam::Point3>::KeyValuePair& key_value, object_filtered)
   {

   }


  gtsam::Values::ConstFiltered<gtsam::Pose3> pose_filtered = current_solution.filter<gtsam::Pose3>();
  gtsam::Values::ConstFiltered<gtsam::Point3> point_filtered = current_solution.filter<gtsam::Point3>();

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

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_map_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
      copyPointCloud (*map_cloud, *rgb_map_cloud);

      //(*aggregate_cloud) += (*map_cloud);
      (*aggregate_cloud) += (*rgb_map_cloud);
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

        // for (int j = 0; j < cluster.points.size (); j++)
        // {
        //   cluster.points[j].r = (cluster.points[j].r + red[i%6]) / 2;
        //   cluster.points[j].g = (cluster.points[j].g + grn[i%6]) / 2;
        //   cluster.points[j].b = (cluster.points[j].b + blu[i%6]) / 2;
        // }

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

  // Draw the pose graph
  if (draw_pose_graph_)
  {
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker mapper_graph;
    mapper_graph.header.frame_id = "/world";
    mapper_graph.header.stamp = ros::Time ();
    mapper_graph.ns = "error_lines";
    mapper_graph.id = 0;
    mapper_graph.type = visualization_msgs::Marker::LINE_LIST;
    mapper_graph.action = visualization_msgs::Marker::ADD;
    mapper_graph.color.a = 0.5;
    mapper_graph.color.r = 1.0;
    mapper_graph.color.g = 0.0;
    mapper_graph.color.b = 0.0;
    mapper_graph.scale.x = 0.01;
    
    visualization_msgs::Marker object_graph;
    object_graph.header.frame_id = "/world";
    object_graph.header.stamp = ros::Time ();
    object_graph.ns = "object_lines";
    object_graph.id = 0;
    object_graph.type = visualization_msgs::Marker::LINE_LIST;
    object_graph.action = visualization_msgs::Marker::ADD;
    object_graph.color.a = 0.5;
    object_graph.color.r = 0.0;
    object_graph.color.g = 1.0;
    object_graph.color.b = 0.0;
    object_graph.scale.x = 0.01;

    visualization_msgs::Marker object_object_graph;
    object_object_graph.header.frame_id = "/world";
    object_object_graph.header.stamp = ros::Time ();
    object_object_graph.ns = "object_object_lines";
    object_object_graph.id = 0;
    object_object_graph.type = visualization_msgs::Marker::LINE_LIST;
    object_object_graph.action = visualization_msgs::Marker::ADD;
    object_object_graph.color.a = 1.0;
    object_object_graph.color.r = 1.0;
    object_object_graph.color.g = 0.0;
    object_object_graph.color.b = 0.0;
    object_object_graph.scale.x = 0.01;


    BOOST_FOREACH (const gtsam::NonlinearFactorGraph::sharedFactor& factor, current_graph)
    {
      // check for poses
      const std::vector<gtsam::Key> keys = factor->keys ();
      
      // skip if there aren't two pose keys
      if ((keys.size () == 2))
      {
        if ((gtsam::symbolChr (keys[0]) == 'x') && (gtsam::symbolChr (keys[1]) == 'x'))
        {
          gtsam::Pose3 p1 = current_solution.at<gtsam::Pose3>(keys[0]);
          gtsam::Pose3 p2 = current_solution.at<gtsam::Pose3>(keys[1]);
          
          geometry_msgs::Point p1_msg;
          p1_msg.x = p1.x ();
          p1_msg.y = p1.y ();
          p1_msg.z = p1.z ();
          
          geometry_msgs::Point p2_msg;
          p2_msg.x = p2.x ();
          p2_msg.y = p2.y ();
          p2_msg.z = p2.z ();
          
          mapper_graph.points.push_back (p1_msg);
          mapper_graph.points.push_back (p2_msg);
        }


        if ((gtsam::symbolChr (keys[0]) == 'x') && (gtsam::symbolChr (keys[1]) == 'o'))
        {
          gtsam::Pose3 p1 = current_solution.at<gtsam::Pose3>(keys[0]);
          gtsam::Point3 p2 = current_solution.at<gtsam::Point3>(keys[1]);

          p1.print("Current Pose:\n");
          p2.print("Current Object:\n");
          geometry_msgs::Point p1_msg;
          p1_msg.x = p1.x ();
          p1_msg.y = p1.y ();
          p1_msg.z = p1.z ();

          geometry_msgs::Point p2_msg;
          p2_msg.x = p2.x ();
          p2_msg.y = p2.y ();
          p2_msg.z = p2.z ();

          object_graph.points.push_back (p1_msg);
          object_graph.points.push_back (p2_msg);
        }

        if ((gtsam::symbolChr (keys[0]) == 'o') && (gtsam::symbolChr (keys[1]) == 'o'))
            {
              gtsam::Point3 p1 = current_solution.at<gtsam::Point3>(keys[0]);
              gtsam::Point3 p2 = current_solution.at<gtsam::Point3>(keys[1]);

              p1.print("Object1:\n");
              p2.print("Object2:\n");
              geometry_msgs::Point p1_msg;
              p1_msg.x = p1.x ();
              p1_msg.y = p1.y ();
              p1_msg.z = p1.z ();

              geometry_msgs::Point p2_msg;
              p2_msg.x = p2.x ();
              p2_msg.y = p2.y ();
              p2_msg.z = p2.z ();

              object_object_graph.points.push_back (p1_msg);
              object_object_graph.points.push_back (p2_msg);
            }
      }
    }
    
    marker_array.markers.push_back (mapper_graph);
    marker_array.markers.push_back (object_graph);
   // marker_array.markers.push_back (object_object_graph);
    marker_array_pub_.publish (marker_array);
  }

  // Optionally draw the pose marginals
   if (draw_pose_marginals_)
   {
     gtsam::Marginals marginals (*vis_graph, *vis_values);

     visualization_msgs::MarkerArray pose_cov_markers;

		BOOST_FOREACH (const gtsam::Values::ConstFiltered<gtsam::Pose3>::KeyValuePair& key_value, pose_filtered)
		{
			try
			{
				geometry_msgs::Pose pose;

				gtsam::Symbol key_symbol (key_value.key);
				gtsam::Pose3 sam_pose = key_value.value;
				gtsam::Rot3 rot = sam_pose.rotation ();

				gtsam::Matrix pose_cov = marginals.marginalCovariance (key_symbol);
				gtsam::Matrix u, v;
				gtsam::Vector s;
				gtsam::svd (pose_cov, u, s, v);

				btMatrix3x3 btm (u (0,0), u (0,1), u (0,2),
						u (1,0), u (1,1), u (1,2),
						u (2,0), u (2,1), u (2,2));
				btQuaternion btq;
				btm.getRotation (btq);

				visualization_msgs::Marker pose_cov_marker;
				pose_cov_marker.header.frame_id = "/world";
				pose_cov_marker.header.stamp = ros::Time::now ();
				pose_cov_marker.type = visualization_msgs::Marker::SPHERE;
				pose_cov_marker.action = visualization_msgs::Marker::ADD;
				pose_cov_marker.ns = "pose_covariances";
				pose_cov_marker.id = key_symbol.index ();
				pose_cov_marker.color.r = 0.0f;
				pose_cov_marker.color.g = 0.0f;
				pose_cov_marker.color.b = 1.0f;
				pose_cov_marker.color.a = 0.05;
				pose_cov_marker.pose.position.x = sam_pose.x ();
				pose_cov_marker.pose.position.y = sam_pose.y ();
				pose_cov_marker.pose.position.z = sam_pose.z ();
				pose_cov_marker.pose.orientation.x = btq.x ();
				pose_cov_marker.pose.orientation.y = btq.y ();
				pose_cov_marker.pose.orientation.z = btq.z ();
				pose_cov_marker.pose.orientation.w = btq.w ();
				pose_cov_marker.scale.x = sqrt (s[0]);
				pose_cov_marker.scale.y = sqrt (s[1]);
				pose_cov_marker.scale.z = sqrt (s[2]);
				//pose_cov_markers.markers.push_back (pose_cov_marker);
			}
			catch(std::out_of_range) {

			}

		}

		BOOST_FOREACH (const gtsam::Values::ConstFiltered<gtsam::Point3>::KeyValuePair& key_value, point_filtered)
     {
    	 try{
       geometry_msgs::Pose pose;

       gtsam::Symbol key_symbol (key_value.key);
       gtsam::Point3 sam_point = key_value.value;
       //gtsam::Rot3 rot = sam_pose.rotation ();

       gtsam::Matrix pose_cov = marginals.marginalCovariance (key_symbol);
       gtsam::Matrix u, v;
       gtsam::Vector s;
       gtsam::svd (pose_cov, u, s, v);

       btMatrix3x3 btm (u (0,0), u (0,1), u (0,2),
                        u (1,0), u (1,1), u (1,2),
                        u (2,0), u (2,1), u (2,2));
       btQuaternion btq;
       btm.getRotation (btq);

       visualization_msgs::Marker pose_cov_marker;
       pose_cov_marker.header.frame_id = "/world";
       pose_cov_marker.header.stamp = ros::Time::now ();
       pose_cov_marker.type = visualization_msgs::Marker::SPHERE;
       pose_cov_marker.action = visualization_msgs::Marker::ADD;
       pose_cov_marker.ns = "pose_covariances";
       pose_cov_marker.id = key_symbol.index ();
       pose_cov_marker.color.r = 0.0f;
       pose_cov_marker.color.g = 0.0f;
       pose_cov_marker.color.b = 1.0f;
       pose_cov_marker.pose.position.x = sam_point.x ();
       pose_cov_marker.color.a = 0.05;
       pose_cov_marker.pose.position.y = sam_point.y ();
       pose_cov_marker.pose.position.z = sam_point.z ();
       pose_cov_marker.pose.orientation.x = 0.0f;
       pose_cov_marker.pose.orientation.y = 0.0f;
       pose_cov_marker.pose.orientation.z = 0.0f;
       pose_cov_marker.pose.orientation.w = 0.0f;
       pose_cov_marker.scale.x = sqrt (s[0]);
       pose_cov_marker.scale.y = sqrt (s[1]);
       pose_cov_marker.scale.z = sqrt (s[2]);
       pose_cov_markers.markers.push_back (pose_cov_marker);
    	 }
    	 catch(std::out_of_range){

    	 }

     }


     pose_covariances_pub_.publish(pose_cov_markers);

   }

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
omnimapper::OmniMapperVisualizerRViz<PointT>::clusterCloudCallback (std::vector<CloudPtr> clusters, omnimapper::Time t,  boost::optional<std::vector<pcl::PointIndices> > indices)
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


template<typename PointT> void omnimapper::OmniMapperVisualizerRViz<PointT>::objectCallback (
    std::map<gtsam::Symbol, Object<PointT> > object_map, gtsam::Point3 view_center, gtsam::Point3 view_direction)
{

  typename std::map<gtsam::Symbol, Object<PointT> >::iterator it;
  typename std::map<gtsam::Symbol, CloudPtr>::iterator it_cluster;
  typename std::multimap<int, gtsam::Symbol> top_objects;
  typename std::multimap<int, gtsam::Symbol>::iterator obj_iterator;
  pcl::PointCloud<pcl::PointXYZRGB> aggregate_cloud;
  pcl::PointCloud<pcl::PointXYZRGB> truncated_map_cloud;

  for (it = object_map.begin (); it != object_map.end (); it++)
  {
    gtsam::Symbol sym = it->first;
    top_objects.insert(std::pair<int, gtsam::Symbol>(it->second.clusters_.size(), sym));
  }


  int object_count = 0;
  for (obj_iterator = top_objects.begin (); obj_iterator != top_objects.end ();
      obj_iterator++)
  {
    if(object_count == 20)break;
    gtsam::Symbol sym = obj_iterator->second;
   Object<PointT>& object = object_map.at(sym);

    object.object_mutex_.lock();
   CloudPtr optimal_cloud = object.optimal_cloud_;
   object.object_mutex_.unlock();
    pcl::copyPointCloud(*optimal_cloud, truncated_map_cloud);
    aggregate_cloud = aggregate_cloud + truncated_map_cloud;

    object_count++; //counter to process only top K objects
  }

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(aggregate_cloud, cloud_msg);
  cloud_msg.header.frame_id = "/world"; ///camera_rgb_optical_frame";
  cloud_msg.header.stamp = ros::Time::now ();
  object_modeled_pub_.publish(cloud_msg);


  view_direction.print("[rviz] view direction: ");
  view_center.print("[rviz] view center");
  gtsam::Point3 transformed_view_center (view_center.x (), view_center.y (),
      view_center.z ());
  /* publish the camera frustum */
int depth_limit = 3; // frustum culling at 3m
double vertical_angle = (49/2)*M_PI/180; // kinect vertical FOV=49 degrees
double horizontal_angle = (57/2)*M_PI/180; // kinect horizontal FOV = 57
gtsam::Point3 frame_center = transformed_view_center + depth_limit*view_direction;
frame_center.print("[rviz] frame center");



geometry_msgs::Point p1;
  p1.x = 0;
  p1.y = 0;
  p1.z = 0;

  geometry_msgs::Point frame_c;
  frame_c.x = 0;
  frame_c.y = 0;
  frame_c.z = depth_limit;

geometry_msgs::Point p2;
p2.x = depth_limit*tan(horizontal_angle);
p2.y = depth_limit*tan(vertical_angle);
p2.z = depth_limit;

geometry_msgs::Point p3;
p3.x = -p2.x                                                                                                                                                                                                               ;
p3.y = p2.y;
p3.z = p2.z;

geometry_msgs::Point p4;
p4.x = p2.x;
p4.y = -p2.y;
p4.z = p2.z;

geometry_msgs::Point p5;
p5.x = -p2.x;
p5.y = -p2.y;
p5.z = p2.z;

  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker frustum_marker;
  frustum_marker.header.frame_id = "/camera_rgb_optical_frame";
  frustum_marker.header.stamp = ros::Time::now ();
  frustum_marker.ns = "camera_frustum";
  frustum_marker.id = 0;
  frustum_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  frustum_marker.action = visualization_msgs::Marker::ADD;

  frustum_marker.points.push_back (p1);
  frustum_marker.points.push_back (p2);
  frustum_marker.points.push_back (p3);


  frustum_marker.points.push_back (p1);
  frustum_marker.points.push_back (p2);
  frustum_marker.points.push_back (p4);



  frustum_marker.points.push_back (p1);
  frustum_marker.points.push_back (p4);
  frustum_marker.points.push_back (p5);


  frustum_marker.points.push_back (p1);
  frustum_marker.points.push_back (p3);
  frustum_marker.points.push_back (p5);



  frustum_marker.scale.x = 1.0;
  frustum_marker.scale.y = 1.0;
  frustum_marker.scale.z = 1.0;
  frustum_marker.color.a = 0.5;
  frustum_marker.color.r = 1.0;
  frustum_marker.color.g = 1.0;
  frustum_marker.color.b = 1.0;
  marker_array.markers.push_back (frustum_marker);
  marker_array_pub_.publish (marker_array);



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
