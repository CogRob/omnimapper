/*
 * Software License Agreement (BSD License)
 *
 *  OmniMapper
 *  Copyright (c) 2012-, Georgia Tech Research Corporation,
 *  Atlanta, Georgia 30332-0415
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <omnimapper_ros/omnimapper_ros.h>

template<typename PointT>
OmniMapperROS<PointT>::OmniMapperROS (ros::NodeHandle nh)
  : n_ ("~"),
    omb_ (),
    tf_plugin_ (&omb_),
    no_motion_plugin_ (&omb_),
    icp_plugin_ (&omb_),
    edge_icp_plugin_ (&omb_),
    //bounded_plane_plugin_ (&omb_), // Comning soon...
    csm_plugin_ (&omb_),
    vis_plugin_ (&omb_),
    csm_vis_plugin_ (&omb_),
    //tsdf_plugin_ (&omb_), //
    organized_segmentation_ (),
    tf_listener_ (ros::Duration (500.0))
{
  if (debug_)
    ROS_INFO ("OmniMapperROS: Constructing... Loading ROS Params...");
  loadROSParams ();
  
  // Use ROS Time instead of system clock
  omnimapper::GetTimeFunctorPtr time_functor_ptr (new omnimapper::GetROSTimeFunctor ());
  omb_.setTimeFunctor (time_functor_ptr);
  omb_.setSuppressCommitWindow (suppress_commit_window_);

  // Optionally specify an alternate initial pose
  if (use_init_pose_)
  {
    gtsam::Pose3 init_pose (gtsam::Rot3::quaternion (init_qw_, init_qx_, init_qy_, init_qz_),
                            gtsam::Point3 (init_x_, init_y_, init_z_));
    omb_.setInitialPose (init_pose);
  }
  
  // Optionally get initial pose from TF
  if (init_pose_from_tf_)
  {
    bool got_tf = false;
    tf::StampedTransform init_transform;
    
    while (!got_tf)
    {
      got_tf = true;
      try
      {
        ROS_INFO("Waiting for initial pose from %s to %s", odom_frame_name_.c_str (), base_frame_name_.c_str ());
        ros::Time current_time = ros::Time::now ();
        tf_listener_.waitForTransform (odom_frame_name_, base_frame_name_,
                                       current_time, ros::Duration (1.0));
        tf_listener_.lookupTransform (odom_frame_name_, base_frame_name_,
                                      current_time, init_transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_INFO("Transform not yet available!");
        got_tf = false;
      }
    }
    
    gtsam::Pose3 init_pose = omnimapper::tf2pose3 (init_transform);
    gtsam::Pose3 init_pose_inv = init_pose.inverse ();
    omb_.setInitialPose (init_pose);
  }
  
  // Add the TF Pose Plugin
  tf_plugin_.setOdomFrameName (odom_frame_name_);
  tf_plugin_.setBaseFrameName (base_frame_name_);
  tf_plugin_.setRotationNoise (tf_trans_noise_);
  tf_plugin_.setRollNoise (tf_roll_noise_);
  tf_plugin_.setPitchNoise (tf_pitch_noise_);
  tf_plugin_.setYawNoise (tf_yaw_noise_);
  tf_plugin_.setTranslationNoise (tf_rot_noise_);
  if (use_tf_)
  {
    boost::shared_ptr<omnimapper::PosePlugin> tf_plugin_ptr (&tf_plugin_);
    omb_.addPosePlugin (tf_plugin_ptr);
  }
  
  // Add the No Motion Plugin (null motion model)
  if (use_no_motion_)
  {
    boost::shared_ptr<omnimapper::PosePlugin> no_motion_ptr (&no_motion_plugin_);
    omb_.addPosePlugin (no_motion_ptr);
  }
  
  // Set up a sensor_to_base functor, for plugins to use
  omnimapper::GetTransformFunctorPtr rgbd_to_base_ptr (new omnimapper::GetTransformFunctorTF (rgbd_frame_name_, base_frame_name_));
  // Optionally disable this, if we don't have TF available
  if (!use_rgbd_sensor_base_tf_functor_)
  {
    rgbd_to_base_ptr = omnimapper::GetTransformFunctorPtr (new omnimapper::GetTransformFunctorIdentity ());
  }

  // Set up an ICP Plugin
  icp_plugin_.setUseGICP (true);
  icp_plugin_.setOverwriteTimestamps (false);
  icp_plugin_.setAddIdentityOnFailure (icp_add_identity_on_fail_);
  icp_plugin_.setShouldDownsample (true);
  icp_plugin_.setLeafSize (icp_leaf_size_);      //0.02
  icp_plugin_.setMaxCorrespondenceDistance (icp_max_correspondence_distance_);
  icp_plugin_.setScoreThreshold (icp_score_thresh_);
  icp_plugin_.setTransNoise (icp_trans_noise_);//10.1
  icp_plugin_.setRotNoise (icp_rot_noise_);//10.1
  icp_plugin_.setAddLoopClosures (icp_add_loop_closures_);
  icp_plugin_.setTransNoise (icp_trans_noise_);      //10.1
  icp_plugin_.setRotNoise (icp_rot_noise_);      //10.1
  icp_plugin_.setLoopClosureDistanceThreshold (icp_loop_closure_distance_threshold_);
  icp_plugin_.setSaveFullResClouds (icp_save_full_res_clouds_);
  icp_plugin_.setSensorToBaseFunctor (rgbd_to_base_ptr);

  // Set up edge ICP plugin
  edge_icp_plugin_.setUseGICP (false);
  edge_icp_plugin_.setOverwriteTimestamps (false);
  edge_icp_plugin_.setAddIdentityOnFailure (occ_edge_add_identity_on_fail_);
  edge_icp_plugin_.setShouldDownsample (false);
  edge_icp_plugin_.setMaxCorrespondenceDistance (occ_edge_max_correspondence_dist_);
  edge_icp_plugin_.setScoreThreshold (occ_edge_score_thresh_);
  edge_icp_plugin_.setTransNoise (occ_edge_trans_noise_);      //10.1
  edge_icp_plugin_.setRotNoise (occ_edge_rot_noise_);      //10.1
  edge_icp_plugin_.setAddLoopClosures (false);
  edge_icp_plugin_.setLoopClosureDistanceThreshold (0.15);
  edge_icp_plugin_.setSaveFullResClouds (false);
  edge_icp_plugin_.setSensorToBaseFunctor (rgbd_to_base_ptr);

  /*
  // Set up the Bounded Plane Plugin
  bounded_plane_plugin_.setRangeThreshold (plane_range_threshold_);
  bounded_plane_plugin_.setAngularThreshold (plane_angular_threshold_);
  bounded_plane_plugin_.setAngularNoise (plane_angular_noise_);
  bounded_plane_plugin_.setRangeNoise (plane_range_noise_);
  bounded_plane_plugin_.setSensorToBaseFunctor (rgbd_to_base_ptr);
  */

  // Set up the feature extraction
  if (use_occ_edge_icp_)
  {
    boost::function<void (const CloudConstPtr&)> edge_icp_cloud_cb =
    boost::bind (&omnimapper::ICPPoseMeasurementPlugin<PointT>::cloudCallback, &edge_icp_plugin_, _1);
    organized_segmentation_.setOccludingEdgeCallback (edge_icp_cloud_cb);
  }

  /*
  if (use_bounded_planes_)
  {
    ROS_INFO ("OmniMapperROS: Installing BoundedPlanePlugin callback.");
    boost::function<void (std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >, omnimapper::Time)> plane_cb = boost::bind (&omnimapper::BoundedPlanePlugin<PointT>::planarRegionCallback, &bounded_plane_plugin_, _1, _2);
    organized_segmentation_.setPlanarRegionStampedCallback (plane_cb);
  }
  */
  
  // Optionally use planes in the visualizer
  if (ar_mode_)
  {
    boost::function<void (std::vector<pcl::PlanarRegion<PointT>,  Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >, omnimapper::Time)> plane_vis_cb = boost::bind (&omnimapper::OmniMapperVisualizerRViz<PointT>::planarRegionCallback, &vis_plugin_, _1, _2);
    organized_segmentation_.setPlanarRegionStampedCallback (plane_vis_cb);
  }

  // Optionally draw label cloud
  if (draw_label_cloud_)
  {
    boost::function<void (const CloudConstPtr&, const LabelCloudConstPtr&)> label_vis_callback = boost::bind (&omnimapper::OmniMapperVisualizerRViz<PointT>::labelCloudCallback, &vis_plugin_, _1, _2);
    organized_segmentation_.setClusterLabelsCallback (label_vis_callback);
  }

// Canonical Scan Matcher
if (use_csm_)
 {
   // Subscribe to laser scan
   laserScan_sub_ = n_.subscribe ("/scan", 1, &OmniMapperROS::laserScanCallback, this);

  // Make a Trigger Functor: TODO: param this
  //omnimapper::TriggerFunctorPtr trigger_functor_ptr (new omnimapper::TriggerPeriodic (time_functor_ptr, 2.0));
  omnimapper::TriggerFunctorPtr trigger_functor_ptr (new omnimapper::TriggerAlways ());
  csm_plugin_.setTriggerFunctor(trigger_functor_ptr);

  boost::shared_ptr<omnimapper::CanonicalScanMatcherPlugin<sensor_msgs::LaserScan> > csm_ptr (&csm_plugin_);
  csm_vis_plugin_.setCSMPlugin (csm_ptr);
  
  // Install the visualizer
  boost::shared_ptr<omnimapper::OutputPlugin> csm_vis_ptr (&csm_vis_plugin_);
  omb_.addOutputPlugin (csm_vis_ptr);
   
  boost::thread csm_thread (&omnimapper::CanonicalScanMatcherPlugin<sensor_msgs::LaserScan>::spin, &csm_plugin_);
 }
 
// Set the ICP Plugin on the visualizer
 boost::shared_ptr<omnimapper::ICPPoseMeasurementPlugin<PointT> > icp_ptr (&icp_plugin_);
 vis_plugin_.setICPPlugin (icp_ptr);
 
 // Subscribe to Point Clouds
 pointcloud_sub_ = n_.subscribe (cloud_topic_name_, 1, &OmniMapperROS::cloudCallback, this);  //("/camera/depth/points", 1, &OmniMapperHandheldNode::cloudCallback, this);
 
 // Install the visualizer
 if (use_rviz_plugin_)
 {
   vis_plugin_.setDrawPoseArray (draw_pose_array_);
   vis_plugin_.setDrawPoseGraph (draw_pose_graph_);
   vis_plugin_.setDrawICPCloudsAlways (draw_icp_clouds_always_);
   boost::shared_ptr<omnimapper::OutputPlugin> vis_ptr (&vis_plugin_);
   omb_.addOutputPlugin (vis_ptr);
 }

 // Set up the TSDF Plugin
 // if (use_tsdf_plugin_)
 // {
 //   tsdf_plugin_.setICPPlugin (icp_ptr);
 //   boost::shared_ptr<omnimapper::OutputPlugin> tsdf_ptr (&tsdf_plugin_);
 //   omb_.addOutputPlugin (tsdf_ptr);
 // }

 generate_tsdf_srv_ = n_.advertiseService ("generate_map_tsdf",
                                           &OmniMapperROS::generateMapTSDFCallback, this);

 //OmniMapper thread
 omb_.setDebug (false);
 boost::thread omb_thread (&omnimapper::OmniMapperBase::spin, &omb_);
 if (use_icp_)
   boost::thread icp_thread (&omnimapper::ICPPoseMeasurementPlugin<PointT>::spin, &icp_plugin_);
 if (use_occ_edge_icp_)
   boost::thread edge_icp_thread (&omnimapper::ICPPoseMeasurementPlugin<PointT>::spin, &edge_icp_plugin_);
 if (use_rviz_plugin_)
   boost::thread rviz_plugin_thread (&omnimapper::OmniMapperVisualizerRViz<PointT>::spin, &vis_plugin_);

 if (use_organized_segmentation_)
   organized_segmentation_.spin ();

 if (debug_)
   ROS_INFO ("OmniMapperROS: Constructor complete.");
}

template<typename PointT> void
OmniMapperROS<PointT>::loadROSParams ()
{
  // Load some params
  n_.param ("use_planes", use_planes_, true);
  n_.param ("use_bounded_planes", use_bounded_planes_, true);
  n_.param ("use_objects", use_objects_, true);
  n_.param ("use_csm", use_csm_, true);
  n_.param ("use_icp", use_icp_, true);
  n_.param ("use_occ_edge_icp", use_occ_edge_icp_, false);
  n_.param ("use_tf", use_tf_, true);
  n_.param ("use_tsdf_plugin", use_tsdf_plugin_, true);
  n_.param ("use_error_plugin", use_error_plugin_, false);
  n_.param ("use_error_eval_plugin", use_error_eval_plugin_, false);
  n_.param ("use_no_motion", use_no_motion_, false);
  n_.param ("odom_frame_name", odom_frame_name_, std::string ("/odom"));
  n_.param ("base_frame_name", base_frame_name_, std::string ("/camera_depth_optical_frame"));
  n_.param ("cloud_topic_name", cloud_topic_name_, std::string ("/throttled_points"));
  n_.param ("rgbd_frame_name", rgbd_frame_name_, std::string ("/camera_rgb_optical_frame"));
  n_.param ("icp_leaf_size", icp_leaf_size_, 0.05);
  n_.param ("icp_max_correspondence_distance", icp_max_correspondence_distance_, 0.5);
  n_.param ("icp_score_thresh", icp_score_thresh_, 0.8);
  n_.param ("icp_trans_noise", icp_trans_noise_, 0.1);
  n_.param ("icp_rot_noise", icp_rot_noise_, 0.1);
  n_.param ("icp_add_identity_on_fail", icp_add_identity_on_fail_, false);
  n_.param ("icp_add_loop_closures", icp_add_loop_closures_, true);
  n_.param ("icp_loop_closure_distance_threshold", icp_loop_closure_distance_threshold_, 1.0);
  n_.param ("icp_loop_closure_score_threshold", icp_loop_closure_score_threshold_, 0.8);
  n_.param ("icp_loop_closure_pose_index_threshold", icp_loop_closure_pose_index_threshold_, 20);
  n_.param ("icp_save_full_res_clouds", icp_save_full_res_clouds_, false);
  n_.param ("occ_edge_trans_noise", occ_edge_trans_noise_, 0.1);
  n_.param ("occ_edge_rot_noise", occ_edge_rot_noise_, 0.1);
  n_.param ("occ_edge_score_thresh", occ_edge_score_thresh_, 0.1);
  n_.param ("occ_edge_max_correspondence_dist",
            occ_edge_max_correspondence_dist_, 0.1);
  n_.param ("occ_edge_add_identity_on_fail", occ_edge_add_identity_on_fail_,
            false);
  n_.param ("plane_range_threshold", plane_range_threshold_, 0.6);
  n_.param ("plane_angular_threshold", plane_angular_threshold_,
            pcl::deg2rad (10.0));
  n_.param ("plane_range_noise", plane_range_noise_, 0.2);
  n_.param ("plane_angular_noise", plane_angular_noise_, 0.26);
  n_.param ("tf_trans_noise", tf_trans_noise_, 0.05);
  n_.param ("tf_rot_noise", tf_rot_noise_, pcl::deg2rad (10.0));
  n_.param ("tf_roll_noise", tf_roll_noise_, tf_rot_noise_);
  n_.param ("tf_pitch_noise", tf_pitch_noise_, tf_rot_noise_);
  n_.param ("tf_yaw_noise", tf_yaw_noise_, tf_rot_noise_);
  n_.param ("use_init_pose", use_init_pose_, false);
  n_.param ("suppress_commit_window", suppress_commit_window_, false);
  n_.param ("init_pose_from_tf", init_pose_from_tf_, false);
  n_.param ("init_x", init_x_, 0.0);
  n_.param ("init_y", init_y_, 0.0);
  n_.param ("init_z", init_z_, 0.0);
  n_.param ("init_qx", init_qx_, 0.0);
  n_.param ("init_qy", init_qy_, 0.0);
  n_.param ("init_qz", init_qz_, 0.0);
  n_.param ("init_qw", init_qw_, 1.0);
  n_.param ("use_rviz_plugin", use_rviz_plugin_, true);
  n_.param ("draw_pose_array", draw_pose_array_, true);
  n_.param ("draw_pose_graph", draw_pose_graph_, true);
  n_.param ("draw_label_cloud", draw_label_cloud_, true);
  n_.param ("draw_clusters", draw_clusters_, false);
  n_.param ("draw_icp_clouds_always", draw_icp_clouds_always_, false);
  n_.param ("use_label_cloud", use_label_cloud_, true);
  n_.param ("add_pose_per_cloud", add_pose_per_cloud_, true);
  n_.param ("broadcast_map_to_odom", broadcast_map_to_odom_, false);
  n_.param ("broadcast_current_pose", broadcast_current_pose_, false);
  n_.param ("use_distortion_model", use_distortion_model_, false);
  n_.param ("use_rgbd_sensor_base_tf_functor",
            use_rgbd_sensor_base_tf_functor_, true);
  n_.param ("evaluation_mode", evaluation_mode_, false);
  n_.param ("evaluation_pcd_path", evaluation_pcd_path_, std::string (""));
  n_.param ("evaluation_associated_txt_path",
            evaluation_associated_txt_path_, std::string (""));
  n_.param ("evaluation_ground_truth_txt_path",
            evaluation_ground_truth_txt_path_, std::string (""));
  n_.param ("evaluation_output_trajectory_txt_path",
            evaluation_output_trajectory_txt_path_, std::string (""));
  n_.param ("evaluation_mode_write_trajectory",
            evaluation_mode_write_trajectory_, true);
  n_.param ("evaluation_mode_write_tsdf", evaluation_mode_write_tsdf_,
            false);
  n_.param ("evaluation_mode_paused", evaluation_mode_paused_, false);
  n_.param ("evaluation_show_frames", evaluation_show_frames_, true);
  n_.param ("object_database_location", object_database_location_,
            std::string ("/home/siddharth/kinect/"));
  n_.param ("object_loop_closures", object_loop_closures_, true);
  n_.param ("object_landmarks", object_landmarks_, true);
  n_.param ("save_object_models", save_object_models_, true);
  n_.param ("object_min_height", object_min_height_, 0.3);
  n_.param ("use_organized_segmentation", use_organized_segmentation_, true);
  n_.param ("debug", debug_, false);
  n_.param ("ar_mode", ar_mode_, false);
}

template<typename PointT> void 
OmniMapperROS<PointT>::cloudCallback (const sensor_msgs::PointCloud2ConstPtr& msg)
{
  if (debug_)
    ROS_INFO("OmniMapperROS got a cloud.");
  double start_cb = pcl::getTime ();
  double start_copy = pcl::getTime ();
  CloudPtr cloud (new Cloud ());
  pcl::fromROSMsg<PointT> (*msg, *cloud);
  double end_copy = pcl::getTime ();
  omnimapper::Time cloud_stamp = omnimapper::stamp2ptime (
                                                          cloud->header.stamp);
  if (debug_)
  {
    std::cout << "OmniMapperRos: Got cloud from: " << cloud_stamp
              << std::endl;
  }

  if (debug_)
  {
    std::cout << "cloudCallback: conversion took "
              << double (end_copy - start_copy) << std::endl;
  }

  if (use_icp_)
  {
    if (debug_)
    {
      std::cout << "Calling ICP Plugin with stamp: "
                << omnimapper::stamp2ptime (cloud->header.stamp) << std::endl;
    }
        
    icp_plugin_.cloudCallback (cloud);
  }
      
  if (use_organized_segmentation_)
  {
    double start_ofe = pcl::getTime ();
    organized_segmentation_.cloudCallback (cloud);
    double end_ofe = pcl::getTime ();
    if (debug_)
      std::cout << "cloudCallback: ofe_cb took " << double(end_ofe - start_ofe) << std::endl;
  }

  if (add_pose_per_cloud_)
  {
    double start_getpose = pcl::getTime ();
    gtsam::Symbol sym;
    boost::posix_time::ptime header_time = omnimapper::stamp2ptime (
                                                                    cloud->header.stamp);      //msg->header.stamp.toBoost ();
    if (debug_)
      std::cout << "header time: " << header_time << std::endl;
    omb_.getPoseSymbolAtTime (header_time, sym);
    double end_getpose = pcl::getTime ();
    if (debug_)
      std::cout << "cloudCallback: get_pose took " << double (end_getpose - start_getpose) << std::endl;
  }
  if (broadcast_map_to_odom_)
  {
    double start_pub = pcl::getTime ();
    publishMapToOdom ();
    double end_pub = pcl::getTime ();
    if (debug_)
      std::cout << "cloudCallback: pub took " << double (end_pub - start_pub) << std::endl;
  }
  if (broadcast_current_pose_)
  {
    publishCurrentPose ();
  }
  double end_cb = pcl::getTime ();
  if (debug_)
    std::cout << "cloudCallback: cb took: " << double (end_cb - start_cb) << std::endl;
}

template <typename PointT> void
OmniMapperROS<PointT>::laserScanCallback (const sensor_msgs::LaserScanConstPtr& msg)
{
  if (debug_)
    ROS_INFO("OmniMapperROS: got a scan.");

  gtsam::Symbol sym;

  boost::shared_ptr<sensor_msgs::LaserScan> lscanPtr1 (new sensor_msgs::LaserScan (*msg));
  csm_plugin_.laserScanCallback (lscanPtr1);
  publishMapToOdom ();
}

template <typename PointT> bool
OmniMapperROS<PointT>::generateMapTSDFCallback (omnimapper_ros::OutputMapTSDF::Request& req,
                                                omnimapper_ros::OutputMapTSDF::Response &res)
{
  printf ("TSDF Service call, calling tsdf plugin\n");
  //tsdf_plugin_.generateTSDF (req.grid_size, req.resolution);
  return (true);
}

template <typename PointT> void
OmniMapperROS<PointT>::publishMapToOdom ()
{
  gtsam::Pose3 current_pose;
  boost::posix_time::ptime current_time;
  omb_.getLatestPose (current_pose, current_time);
  ros::Time current_time_ros = omnimapper::ptime2rostime (current_time);
  tf::Transform current_pose_ros = omnimapper::pose3totf (current_pose);

  tf::Stamped<tf::Pose> odom_to_map;
  try
  {
//        tf_listener_.transformPose ("/odom", tf::Stamped<tf::Pose> (tf::btTransform (tf::btQuaternion (current_quat[1], current_quat[2], current_quat[3], current_quat[0]), btVector3 (current_pose.x (), current_pose.y (), current_pose.z ())).inverse (), current_time_ros, "/base_link"), odom_to_map);
    tf_listener_.waitForTransform (odom_frame_name_, base_frame_name_,
                                   current_time_ros, ros::Duration (0.05));
    tf_listener_.transformPose (odom_frame_name_,
                                tf::Stamped<tf::Pose> (current_pose_ros.inverse (),
                                                       current_time_ros, base_frame_name_), odom_to_map);
  }
  catch (tf::TransformException e)
  {
    ROS_ERROR(
              "OmniMapperROS: Error: could not immediately get odom to base transform\n");
    odom_to_map.setIdentity ();
    return;
  }
  tf::Transform map_to_odom = odom_to_map.inverse ();  //tf::Transform (tf::Quaternion (odom_to_map.getRotation ()), tf::Point (odom_to_map.getOrigin ())
  tf_broadcaster_.sendTransform (
                                 tf::StampedTransform (map_to_odom, ros::Time::now (), "/world",
                                                       odom_frame_name_));
}

template <typename PointT> void
OmniMapperROS<PointT>::publishCurrentPose ()
{
  gtsam::Pose3 current_pose;
  boost::posix_time::ptime current_time;
  omb_.getLatestPose (current_pose, current_time);
  ros::Time current_time_ros = omnimapper::ptime2rostime (current_time);
  tf::Transform current_pose_ros = omnimapper::pose3totf (current_pose);
  
  tf_broadcaster_.sendTransform (tf::StampedTransform (current_pose_ros, ros::Time::now (), "/world", "/current_pose"));
  
}

template <typename PointT> void
OmniMapperROS<PointT>::resetEvaluation ()
{
  evaluation_pcd_files_.clear ();

  // ICP Plugin
  icp_plugin_.setUseGICP (true);
  icp_plugin_.setOverwriteTimestamps (false);
  icp_plugin_.setAddIdentityOnFailure (icp_add_identity_on_fail_);
  icp_plugin_.setShouldDownsample (true);
  icp_plugin_.setLeafSize (icp_leaf_size_);      //0.02
  icp_plugin_.setMaxCorrespondenceDistance (icp_max_correspondence_distance_);
  icp_plugin_.setScoreThreshold (icp_score_thresh_);
  icp_plugin_.setTransNoise (icp_trans_noise_);//10.1
  icp_plugin_.setRotNoise (icp_rot_noise_);//10.1
  icp_plugin_.setAddLoopClosures (icp_add_loop_closures_);
  icp_plugin_.setTransNoise (icp_trans_noise_);      //10.1
  icp_plugin_.setRotNoise (icp_rot_noise_);      //10.1
  icp_plugin_.setLoopClosureDistanceThreshold (icp_loop_closure_distance_threshold_);
  icp_plugin_.setSaveFullResClouds (true);
}

template class OmniMapperROS<pcl::PointXYZRGBA>;




