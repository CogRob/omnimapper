#include <glog/logging.h>
#include <omnimapper_ros/omnimapper_ros.h>

template <typename PointT>
OmniMapperROS<PointT>::OmniMapperROS(ros::NodeHandle nh)
    : n_("~"),
      omb_(),
      tf_plugin_(&omb_),
      no_motion_plugin_(&omb_),
      icp_plugin_(&omb_),
      edge_icp_plugin_(&omb_),
      plane_plugin_(&omb_),
      bounded_plane_plugin_(&omb_),
      object_plugin_(&omb_),
      csm_plugin_(&omb_),
      ar_marker_plugin_(&omb_),
      vis_plugin_(&omb_),
      csm_vis_plugin_(&omb_),
      tsdf_plugin_(&omb_),
      bag_error_plugin_(&omb_),
      eval_plugin_(&omb_),
      fake_files_({"/tmp/FAKE_FILE_PATH"}),
      fake_grabber_(fake_files_, 1.0, false),
      organized_feature_extraction_(fake_grabber_),
      tf_listener_(ros::Duration(500.0)) {
  LOG(INFO) << "OmniMapperROS constructing.";

  LoadROSParams();

  // Use ROS Time instead of system clock.
  omnimapper::GetTimeFunctorPtr time_functor_ptr(
      new omnimapper::GetROSTimeFunctor());
  omb_.SetTimeFunctor(time_functor_ptr);
  omb_.SetSuppressCommitWindow(suppress_commit_window_);

  // Optionally specify an alternate initial pose.
  if (use_init_pose_) {
    CHECK(!init_pose_from_tf_)
        << "Use either use_init_pose or init_pose_from_tf.";
    gtsam::Pose3 init_pose(
        gtsam::Rot3::quaternion(init_qw_, init_qx_, init_qy_, init_qz_),
        gtsam::Point3(init_x_, init_y_, init_z_));
    omb_.SetInitialPose(init_pose);
  }

  // Optionally get initial pose from TF.
  if (init_pose_from_tf_) {
    CHECK(!use_init_pose_) << "Use either use_init_pose or init_pose_from_tf.";
    bool got_tf = false;
    tf::StampedTransform init_transform;

    while (!got_tf) {
      try {
        LOG(INFO) << "Waiting for initial pose from " << odom_frame_name_
                  << " to " << base_frame_name_;
        ros::Time current_time = ros::Time::now();
        tf_listener_.waitForTransform(odom_frame_name_, base_frame_name_,
                                      current_time, ros::Duration(1.0));
        tf_listener_.lookupTransform(odom_frame_name_, base_frame_name_,
                                     current_time, init_transform);
        got_tf = true;
      } catch (tf::TransformException ex) {
        LOG(INFO) << "Transform not yet available!";
        got_tf = false;
      }
    }
    gtsam::Pose3 init_pose = omnimapper::TfToPose3(init_transform);
    gtsam::Pose3 init_pose_inv = init_pose.inverse();
    omb_.SetInitialPose(init_pose);
  }

  // Optionally use distortion model.
  if (use_distortion_model_) {
    LOG(INFO) << "Loading distortion model: " << distortion_model_path_;
    distortion_model_.load(distortion_model_path_);
  }

  // Add the TF Pose Plugin
  tf_plugin_.SetOdomFrameName(odom_frame_name_);
  tf_plugin_.SetBaseFrameName(base_frame_name_);
  tf_plugin_.SetTranslationNoise(tf_trans_noise_);
  tf_plugin_.SetRollNoise(tf_roll_noise_);
  tf_plugin_.SetPitchNoise(tf_pitch_noise_);
  tf_plugin_.SetYawNoise(tf_yaw_noise_);
  if (use_tf_) {
    LOG(INFO) << "Using ROS TF plugin.";
    omb_.AddPosePlugin(&tf_plugin_);
  }

  // Add the No Motion Plugin (null motion model)
  if (use_no_motion_) {
    LOG(ERROR) << "Using NoMotionPlugin, "
               << "are you sure you don't want to use tf?";
    omb_.AddPosePlugin(&no_motion_plugin_);
  }

  // Set up a sensor_to_base functor, for plugins to use.
  // Optionally disable this, if we don't have TF available.
  omnimapper::GetTransformFunctorPtr rgbd_to_base_ptr;
  if (use_rgbd_sensor_base_tf_functor_) {
    LOG(INFO) << "RGBD Sensor to Base transform will be provided by TF.";
    rgbd_to_base_ptr = omnimapper::GetTransformFunctorPtr(
        new omnimapper::GetTransformFunctorTF(rgbd_frame_name_,
                                              base_frame_name_));
  } else {
    LOG(ERROR) << "RGBD Sensor to Base transform will be identity.";
    rgbd_to_base_ptr = omnimapper::GetTransformFunctorPtr(
        new omnimapper::GetTransformFunctorIdentity());
  }

  // Set up an ICP plugin.
  icp_plugin_.SetUseGICP(true);
  icp_plugin_.SetOverwriteTimestamps(false);
  icp_plugin_.SetAddIdentityOnFailure(icp_add_identity_on_fail_);
  icp_plugin_.SetShouldDownsample(true);
  icp_plugin_.SetLeafSize(icp_leaf_size_);
  icp_plugin_.SetMaxCorrespondenceDistance(icp_max_correspondence_distance_);
  icp_plugin_.SetScoreThreshold(icp_score_thresh_);
  icp_plugin_.SetTransNoise(icp_trans_noise_);
  icp_plugin_.SetRotNoise(icp_rot_noise_);
  icp_plugin_.SetAddLoopClosures(icp_add_loop_closures_);
  icp_plugin_.SetLoopClosureDistanceThreshold(
      icp_loop_closure_distance_threshold_);
  icp_plugin_.SetSaveFullResClouds(false);
  icp_plugin_.SetSensorToBaseFunctor(rgbd_to_base_ptr);

  // Set up edge ICP plugin.
  edge_icp_plugin_.SetUseGICP(false);
  edge_icp_plugin_.SetOverwriteTimestamps(false);
  edge_icp_plugin_.SetAddIdentityOnFailure(occ_edge_add_identity_on_fail_);
  edge_icp_plugin_.SetShouldDownsample(false);
  edge_icp_plugin_.SetMaxCorrespondenceDistance(
      occ_edge_max_correspondence_dist_);
  edge_icp_plugin_.SetScoreThreshold(occ_edge_score_thresh_);
  edge_icp_plugin_.SetTransNoise(occ_edge_trans_noise_);  // 10.1
  edge_icp_plugin_.SetRotNoise(occ_edge_rot_noise_);      // 10.1
  edge_icp_plugin_.SetAddLoopClosures(false);
  edge_icp_plugin_.SetLoopClosureDistanceThreshold(0.15);
  edge_icp_plugin_.SetSaveFullResClouds(false);
  edge_icp_plugin_.SetSensorToBaseFunctor(rgbd_to_base_ptr);

  // Set up the Plane plugin.
  plane_plugin_.SetOverwriteTimestamps(false);
  plane_plugin_.SetDisableDataAssociation(false);
  plane_plugin_.SetRangeThreshold(plane_range_threshold_);
  plane_plugin_.SetAngularThreshold(plane_angular_threshold_);
  plane_plugin_.SetAngularNoise(plane_angular_noise_);
  plane_plugin_.SetRangeNoise(plane_range_noise_);
  plane_plugin_.SetSensorToBaseFunctor(rgbd_to_base_ptr);

  // Set up the Bounded Plane plugin.
  bounded_plane_plugin_.SetRangeThreshold(plane_range_threshold_);
  bounded_plane_plugin_.SetAngularThreshold(plane_angular_threshold_);
  bounded_plane_plugin_.SetAngularNoise(plane_angular_noise_);
  bounded_plane_plugin_.SetRangeNoise(plane_range_noise_);
  bounded_plane_plugin_.SetSensorToBaseFunctor(rgbd_to_base_ptr);

  // Set up the object plugin.
  if (use_objects_) {
    LOG(INFO) << "Use objects plugin.";
    object_plugin_.SetSensorToBaseFunctor(rgbd_to_base_ptr);
    object_plugin_.SetAndLoadObjectDatabaseLocation(object_database_location_);
    object_plugin_.UseObjectLoopClosures(object_loop_closures_);
    object_plugin_.UseObjectLandmarks(object_landmarks_);
    object_plugin_.SaveObjectModels(save_object_models_);
    // Min height of the object above floor.
    object_plugin_.SetMinimumClusterHeight(object_min_height_);
  }

  // Set up the feature extraction.
  if (use_occ_edge_icp_) {
    LOG(INFO) << "Use occ edge icp plugin.";
    boost::function<void(const CloudConstPtr&)> edge_icp_cloud_cb = boost::bind(
        &omnimapper::ICPPoseMeasurementPlugin<PointT>::CloudCallback,
        &edge_icp_plugin_, _1);
    organized_feature_extraction_.SetOccludingEdgeCallback(edge_icp_cloud_cb);
  }

  if (use_planes_) {
    LOG(INFO) << "Use planes plugin.";
    boost::function<void(
        std::vector<pcl::PlanarRegion<PointT>,
                    Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >,
        omnimapper::Time)>
        plane_cb = boost::bind(
            &omnimapper::PlaneMeasurementPlugin<PointT>::PlanarRegionCallback,
            &plane_plugin_, _1, _2);
    organized_feature_extraction_.SetPlanarRegionStampedCallback(plane_cb);
    boost::function<void(
        std::vector<pcl::PlanarRegion<PointT>,
                    Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >,
        omnimapper::Time)>
        plane_vis_cb = boost::bind(
            &omnimapper::OmniMapperVisualizerRViz<PointT>::PlanarRegionCallback,
            &vis_plugin_, _1, _2);
    organized_feature_extraction_.SetPlanarRegionStampedCallback(plane_vis_cb);
  }

  if (use_bounded_planes_) {
    LOG(INFO) << "Use bounded planes plugin.";
    LOG(INFO) << "Installing BoundedPlanePlugin callback.";
    boost::function<void(
        std::vector<pcl::PlanarRegion<PointT>,
                    Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >,
        omnimapper::Time)>
        plane_cb = boost::bind(
            &omnimapper::BoundedPlanePlugin<PointT>::PlanarRegionCallback,
            &bounded_plane_plugin_, _1, _2);
    organized_feature_extraction_.SetPlanarRegionStampedCallback(plane_cb);
  }

  // Optionally use planes in the visualizer.
  if (ar_mode_) {
    LOG(INFO) << "AR mode on, show planes in the visualizer";
    boost::function<void(
        std::vector<pcl::PlanarRegion<PointT>,
                    Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >,
        omnimapper::Time)>
        plane_vis_cb = boost::bind(
            &omnimapper::OmniMapperVisualizerRViz<PointT>::PlanarRegionCallback,
            &vis_plugin_, _1, _2);
    organized_feature_extraction_.SetPlanarRegionStampedCallback(plane_vis_cb);
  }

  // Optionally draw label cloud.
  if (draw_label_cloud_) {
    LOG(INFO) << "Draw label cloud in the visualizer.";
    boost::function<void(const CloudConstPtr&, const LabelCloudConstPtr&)>
        label_vis_callback = boost::bind(
            &omnimapper::OmniMapperVisualizerRViz<PointT>::LabelCloudCallback,
            &vis_plugin_, _1, _2);
    organized_feature_extraction_.SetClusterLabelsCallback(label_vis_callback);
  }

  // Optionally draw clusters
  if (draw_clusters_) {
    LOG(INFO) << "Draw clusters in the visualizer.";
    boost::function<void(std::vector<CloudPtr>, omnimapper::Time t,
                         boost::optional<std::vector<pcl::PointIndices> >)>
        cluster_vis_callback = boost::bind(
            &omnimapper::OmniMapperVisualizerRViz<PointT>::ClusterCloudCallback,
            &vis_plugin_, _1, _2, _3);
    organized_feature_extraction_.SetClusterCloudCallback(cluster_vis_callback);
  }

  // Optionally use labels
  if (use_objects_) {
    LOG(INFO) << "Use objects plugin.";
    typename boost::function<void(
        std::vector<CloudPtr>, omnimapper::Time t,
        boost::optional<std::vector<pcl::PointIndices> >)>
        object_cluster_callback =
            boost::bind(&omnimapper::ObjectPlugin<PointT>::ClusterCloudCallback,
                        &object_plugin_, _1, _2, _3);
    organized_feature_extraction_.SetClusterCloudCallback(
        object_cluster_callback);

    boost::function<void(
        std::map<gtsam::Symbol, omnimapper::Object<PointT> > object_map,
        gtsam::Point3, gtsam::Point3)>
        object_vis_callback = boost::bind(
            &omnimapper::OmniMapperVisualizerRViz<PointT>::ObjectCallback,
            &vis_plugin_, _1, _2, _3);
    object_plugin_.SetObjectCallback(object_vis_callback);
  }

  // Canonical Scan Matcher
  if (use_csm_) {
    LOG(INFO) << "Use Canonical Scan Matcher plugin.";
    // Subscribe to laser scan
    laserScan_sub_ =
        n_.subscribe("/scan", 1, &OmniMapperROS::LaserScanCallback, this);

    csm_vis_plugin_.SetCSMPlugin(&csm_plugin_);

    // Install the visualizer
    omb_.AddOutputPlugin(&csm_vis_plugin_);

    boost::thread csm_thread(
        &omnimapper::CanonicalScanMatcherPlugin<sensor_msgs::LaserScan>::Spin,
        &csm_plugin_);
  }

  // Set the ICP Plugin on the visualizer
  // TODO(shengye): This is very dangerous -- if the vis_plugin_ ever abandons
  // the shared pointer, it will try to free icp_plugin_, but icp_plugin_ can
  // not be freed.
  vis_plugin_.SetICPPlugin(&icp_plugin_);

  // Set up the Object Plugin with the visualizer.
  // TODO(shengye): obj_ptr took the ownership of object_plugin_ but it
  // shouldn't have done it.
  vis_plugin_.SetObjectPlugin(&object_plugin_);

  // Subscribe to Point Clouds
  pointcloud_sub_ =
      n_.subscribe(cloud_topic_name_, 1, &OmniMapperROS::CloudCallback, this);

  // Install the visualizer.
  if (use_rviz_plugin_) {
    LOG(INFO) << "Use rviz output plugin.";
    vis_plugin_.SetDrawPoseArray(draw_pose_array_);
    vis_plugin_.SetDrawPoseGraph(draw_pose_graph_);
    vis_plugin_.SetDrawICPCloudsAlways(draw_icp_clouds_always_);
    omb_.AddOutputPlugin(&vis_plugin_);
  }

  // Set up the TSDF plugin.
  if (use_tsdf_plugin_) {
    LOG(INFO) << "Use tsdf output plugin.";
    tsdf_plugin_.SetICPPlugin(&icp_plugin_);
    omb_.AddOutputPlugin(&tsdf_plugin_);
  }

  // Set up the error plugin.
  if (use_error_plugin_) {
    LOG(INFO) << "Use error output plugin.";
    omb_.AddOutputPlugin(&bag_error_plugin_);
  }

  generate_tsdf_srv_ = n_.advertiseService(
      "generate_map_tsdf", &OmniMapperROS::GenerateMapTSDFCallback, this);

  // OmniMapper thread
  omb_.SetDebug(debug_);
  boost::thread omb_thread(&omnimapper::OmniMapperBase::Spin, &omb_);
  if (use_icp_) {
    LOG(INFO) << "Staring a thread for ICP";
    boost::thread icp_thread(
        &omnimapper::ICPPoseMeasurementPlugin<PointT>::Spin, &icp_plugin_);
  }
  if (use_occ_edge_icp_) {
    LOG(INFO) << "Staring a thread for Edge ICP";
    boost::thread edge_icp_thread(
        &omnimapper::ICPPoseMeasurementPlugin<PointT>::Spin, &edge_icp_plugin_);
  }
  if (use_rviz_plugin_) {
    LOG(INFO) << "Staring a thread for Rviz";
    boost::thread rviz_plugin_thread(
        &omnimapper::OmniMapperVisualizerRViz<PointT>::Spin, &vis_plugin_);
  }

  // TODO(shengye): This was inconsistent between TBB and non-TBB. Now they
  // all spawn a new thread.
  if (use_organized_feature_extraction_) organized_feature_extraction_.Spin();

  // If evaluation mode, start a timer to check on things, and load the files
  if (evaluation_mode_ || use_error_eval_plugin_) {
    // TODO(shengye): Again, claimed the ownership when it should not and could
    // not.
    omb_.AddOutputPlugin(&eval_plugin_);  // Calls the update function

    // Set up Visualization and Interactive Markers
    LOG(INFO) << "Getting interactive ptrs.";
    interactive_markers::InteractiveMarkerServer* ims_ptr =
        vis_plugin_.GetInteractiveMarkerServerPtr();
    interactive_markers::MenuHandler* mh_ptr = vis_plugin_.GetMenuHandlerPtr();
    LOG(INFO) << "Setting ptrs";
    eval_plugin_.SetInteractiveMarkerServerPtr(ims_ptr);
    eval_plugin_.SetMenuHandlerPtr(mh_ptr);
    eval_plugin_.InitMenu();
    LOG(INFO) << "Done with that.";
  }

  if (evaluation_mode_) {
    // Set up the error evaluation plugin
    eval_plugin_.LoadAssociatedFile(evaluation_associated_txt_path_);
    eval_plugin_.LoadGroundTruthFile(evaluation_ground_truth_txt_path_);

    // Initialize pose to start in the same coord system as the ground truth.
    gtsam::Pose3 gt_init = eval_plugin_.GetInitialPose();
    omb_.SetInitialPose(gt_init);

    LOG(INFO) << "Loading PCDs";
    evaluation_file_idx_ = 0;
    boost::filesystem::directory_iterator end_itr;
    for (boost::filesystem::directory_iterator itr(evaluation_pcd_path_);
         itr != end_itr; ++itr) {
      if (itr->path().extension() == ".pcd")
        evaluation_pcd_files_.push_back(itr->path().string());
    }
    sort(evaluation_pcd_files_.begin(), evaluation_pcd_files_.end());
    LOG(INFO) << "OmniMapper: Loaded " << evaluation_pcd_files_.size()
              << " files for evaluation.";
    eval_timer_ = n_.createTimer(ros::Duration(0.01),
                                 &OmniMapperROS::EvalTimerCallback, this);
  }

  LOG(INFO) << "OmniMapperROS: Constructor complete.";
}

template <typename PointT>
void OmniMapperROS<PointT>::RunEvaluation(
    std::string& associated_filename, std::string& groundtruth_filename,
    std::string& pcd_path, std::string& output_trajectory_filename,
    std::string& output_timing_filename) {
  // Clear old state.
  omb_.Reset();
  icp_plugin_.Reset();
  eval_plugin_.Reset();
  LoadROSParams();
  ResetEvaluation();

  // Load timestamps and ground truth files
  eval_plugin_.LoadAssociatedFile(associated_filename);
  eval_plugin_.LoadGroundTruthFile(groundtruth_filename);

  // Initialize pose to start in the same coord system as the ground truth
  gtsam::Pose3 gt_init = eval_plugin_.GetInitialPose();
  omb_.SetInitialPose(gt_init);

  LOG(INFO) << "Loading PCDs for evaluation.";
  evaluation_file_idx_ = 0;
  boost::filesystem::directory_iterator end_itr;
  for (boost::filesystem::directory_iterator itr(pcd_path); itr != end_itr;
       ++itr) {
    if (itr->path().extension() == ".pcd")
      evaluation_pcd_files_.push_back(itr->path().string());
  }
  sort(evaluation_pcd_files_.begin(), evaluation_pcd_files_.end());
  LOG(INFO) << "OmniMapper: Loaded " << evaluation_pcd_files_.size()
            << " files for evaluation.";

  // Set up timing information
  const omnimapper::Time exp_start_time =
      boost::posix_time::microsec_clock::local_time();
  omnimapper::Time exp_end_time =
      boost::posix_time::microsec_clock::local_time();
  const omnimapper::Time frame_start_time =
      boost::posix_time::microsec_clock::local_time();
  omnimapper::Time frame_end_time =
      boost::posix_time::microsec_clock::local_time();
  std::vector<omnimapper::Time> frame_start_times;
  std::vector<omnimapper::Time> frame_end_times;

  // Process all files
  bool done = false;
  bool first = true;
  while (!done) {
    bool ready = true;

    // Check if plugins are still processing previous frames
    if (use_icp_ && !icp_plugin_.Ready()) ready = false;

    if (use_occ_edge_icp_ && !edge_icp_plugin_.Ready()) ready = false;

    if (!organized_feature_extraction_.Ready()) ready = false;

    if (ready && (evaluation_file_idx_ ==
                  static_cast<int>(evaluation_pcd_files_.size()))) {
      frame_end_times.push_back(
          (boost::posix_time::microsec_clock::local_time()));
      exp_end_time = boost::posix_time::microsec_clock::local_time();

      // If we're totally done
      ROS_INFO("Completed evaluation, writing output file.");
      gtsam::Values solution = omb_.GetSolution();

      // Write trajectory
      if (evaluation_mode_write_trajectory_) {
        eval_plugin_.WriteMapperTrajectoryFile(output_trajectory_filename,
                                               solution);
        ROS_INFO("Evaluation log written as: %s",
                 output_trajectory_filename.c_str());
      }

      // Write Timing
      std::ofstream timing_file(output_timing_filename.c_str());
      for (std::size_t i = 0; i < frame_start_times.size(); i++) {
        timing_file
            << boost::posix_time::to_iso_extended_string(frame_start_times[i])
            << " "
            << boost::posix_time::to_iso_extended_string(frame_end_times[i])
            << std::endl;
      }
      timing_file.close();

      // Write Git commit hash, ros parameter dump

      // Write TSDF
      if (evaluation_mode_write_tsdf_) {
        tsdf_plugin_.GenerateTSDF(10.0, 1024);
      }

      done = true;
      return;
    } else if (ready && (evaluation_file_idx_ <
                         static_cast<int>(evaluation_pcd_files_.size()))) {
      if (!first)
        frame_end_times.push_back(
            boost::posix_time::microsec_clock::local_time());

      // If we need to process a new frame
      CloudPtr cloud(new Cloud());
      double load_start = pcl::getTime();
      pcl::io::loadPCDFile(evaluation_pcd_files_[evaluation_file_idx_], *cloud);
      double load_end = pcl::getTime();
      if (debug_) {
        std::cout << "Loading took: " << double(load_end - load_start)
                  << std::endl;
        ROS_INFO("Processing cloud %d with %zu points\n", evaluation_file_idx_,
                 cloud->points.size());
      }

      // Convert it
      sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2());
      // We have to get the timestamp from associated.txt (parsed by eval
      // plugin) since PCD doesn't include timestamps...
      cloud->header.stamp =
          eval_plugin_.GetStampFromIndex(evaluation_file_idx_);
      pcl::toROSMsg(*cloud, *cloud_msg);
      evaluation_file_idx_++;

      sensor_msgs::PointCloud2ConstPtr cloud_msg_ptr(cloud_msg);
      first = false;
      frame_start_times.push_back(
          boost::posix_time::microsec_clock::local_time());
      CloudCallback(cloud_msg_ptr);
    } else {
      // If we're still processing, sleep this thread for a moment
      boost::this_thread::sleep(boost::posix_time::milliseconds(10));
    }
    ready = false;
  }
}

template <typename PointT>
void OmniMapperROS<PointT>::LoadROSParams() {
  // Load some params
  LOG(INFO) << "Loading ROS parameters.";
  n_.param("use_planes", use_planes_, true);
  n_.param("use_bounded_planes", use_bounded_planes_, true);
  n_.param("use_objects", use_objects_, true);
  n_.param("use_csm", use_csm_, true);
  n_.param("use_icp", use_icp_, true);
  n_.param("use_occ_edge_icp", use_occ_edge_icp_, false);
  n_.param("use_tf", use_tf_, true);
  n_.param("use_tsdf_plugin", use_tsdf_plugin_, true);
  n_.param("use_error_plugin", use_error_plugin_, false);
  n_.param("use_error_eval_plugin", use_error_eval_plugin_, false);
  n_.param("use_no_motion", use_no_motion_, false);
  n_.param("odom_frame_name", odom_frame_name_, std::string("/odom"));
  n_.param("base_frame_name", base_frame_name_,
           std::string("/camera_depth_optical_frame"));
  n_.param("cloud_topic_name", cloud_topic_name_,
           std::string("/throttled_points"));
  n_.param("rgbd_frame_name", rgbd_frame_name_,
           std::string("/camera_rgb_optical_frame"));
  n_.param("icp_leaf_size", icp_leaf_size_, 0.05);
  n_.param("icp_max_correspondence_distance", icp_max_correspondence_distance_,
           0.5);
  n_.param("icp_score_thresh", icp_score_thresh_, 0.8);
  n_.param("icp_trans_noise", icp_trans_noise_, 0.1);
  n_.param("icp_rot_noise", icp_rot_noise_, 0.1);
  n_.param("icp_add_identity_on_fail", icp_add_identity_on_fail_, false);
  n_.param("icp_add_loop_closures", icp_add_loop_closures_, true);
  n_.param("icp_loop_closure_distance_threshold",
           icp_loop_closure_distance_threshold_, 1.0);
  n_.param("occ_edge_trans_noise", occ_edge_trans_noise_, 0.1);
  n_.param("occ_edge_rot_noise", occ_edge_rot_noise_, 0.1);
  n_.param("occ_edge_score_thresh", occ_edge_score_thresh_, 0.1);
  n_.param("occ_edge_max_correspondence_dist",
           occ_edge_max_correspondence_dist_, 0.1);
  n_.param("occ_edge_add_identity_on_fail", occ_edge_add_identity_on_fail_,
           false);
  n_.param("plane_range_threshold", plane_range_threshold_, 0.6);
  n_.param("plane_angular_threshold", plane_angular_threshold_,
           pcl::deg2rad(10.0));
  n_.param("plane_range_noise", plane_range_noise_, 0.2);
  n_.param("plane_angular_noise", plane_angular_noise_, 0.26);
  n_.param("tf_trans_noise", tf_trans_noise_, 0.05);
  n_.param("tf_roll_noise", tf_roll_noise_, 0.174533);
  n_.param("tf_pitch_noise", tf_pitch_noise_, 0.174533);
  n_.param("tf_yaw_noise", tf_yaw_noise_, 0.174533);
  n_.param("use_init_pose", use_init_pose_, false);
  n_.param("suppress_commit_window", suppress_commit_window_, false);
  n_.param("init_pose_from_tf", init_pose_from_tf_, false);
  n_.param("init_x", init_x_, 0.0);
  n_.param("init_y", init_y_, 0.0);
  n_.param("init_z", init_z_, 0.0);
  n_.param("init_qx", init_qx_, 0.0);
  n_.param("init_qy", init_qy_, 0.0);
  n_.param("init_qz", init_qz_, 0.0);
  n_.param("init_qw", init_qw_, 1.0);
  n_.param("use_rviz_plugin", use_rviz_plugin_, true);
  n_.param("draw_pose_array", draw_pose_array_, true);
  n_.param("draw_pose_graph", draw_pose_graph_, true);
  n_.param("draw_label_cloud", draw_label_cloud_, true);
  n_.param("draw_clusters", draw_clusters_, false);
  n_.param("draw_icp_clouds_always", draw_icp_clouds_always_, false);
  n_.param("use_label_cloud", use_label_cloud_, true);
  n_.param("add_pose_per_cloud", add_pose_per_cloud_, true);
  n_.param("broadcast_map_to_odom", broadcast_map_to_odom_, false);
  n_.param("broadcast_current_pose", broadcast_current_pose_, false);
  n_.param("use_distortion_model", use_distortion_model_, false);
  n_.param("use_rgbd_sensor_base_tf_functor", use_rgbd_sensor_base_tf_functor_,
           true);
  n_.param("distortion_model_path", distortion_model_path_,
           std::string("/home/atrevor/github/atrevor_sandbox/"
                       "sdmiller_calibration/new_distortion_model"));
  n_.param("evaluation_mode", evaluation_mode_, false);
  n_.param("evaluation_pcd_path", evaluation_pcd_path_, std::string(""));
  n_.param("evaluation_associated_txt_path", evaluation_associated_txt_path_,
           std::string(""));
  n_.param("evaluation_ground_truth_txt_path",
           evaluation_ground_truth_txt_path_, std::string(""));
  n_.param("evaluation_output_trajectory_txt_path",
           evaluation_output_trajectory_txt_path_, std::string(""));
  n_.param("evaluation_mode_write_trajectory",
           evaluation_mode_write_trajectory_, true);
  n_.param("evaluation_mode_write_tsdf", evaluation_mode_write_tsdf_, false);
  n_.param("evaluation_mode_paused", evaluation_mode_paused_, false);
  n_.param("evaluation_show_frames", evaluation_show_frames_, true);
  n_.param("object_database_location", object_database_location_,
           std::string("/home/siddharth/kinect/"));
  n_.param("object_loop_closures", object_loop_closures_, true);
  n_.param("object_landmarks", object_landmarks_, true);
  n_.param("save_object_models", save_object_models_, true);
  n_.param("object_min_height", object_min_height_, 0.3);
  n_.param("use_organized_feature_extraction",
           use_organized_feature_extraction_, true);
  n_.param("debug", debug_, false);
  n_.param("ar_mode", ar_mode_, false);
}

template <typename PointT>
void OmniMapperROS<PointT>::CloudCallback(
    const sensor_msgs::PointCloud2ConstPtr& msg) {
  LOG(INFO) << "OmniMapperROS got a cloud.";

  const double start_cb = pcl::getTime();
  const double start_copy = pcl::getTime();
  CloudPtr cloud(new Cloud());
  pcl::fromROSMsg<PointT>(*msg, *cloud);
  const double end_copy = pcl::getTime();
  omnimapper::Time cloud_stamp = omnimapper::StampToPtime(cloud->header.stamp);
  LOG_IF(INFO, debug_) << "OmniMapperRos: Got cloud from: " << cloud_stamp;
  LOG_IF(INFO, debug_) << "CloudCallback: conversion took "
                       << double(end_copy - start_copy) << " seconds.";

  if (use_distortion_model_) {
    const double start_undistort = pcl::getTime();
    distortion_model_.undistort(*cloud);
    const double end_undistort = pcl::getTime();
    LOG_IF(INFO, debug_) << "CloudCallback: undistortion took "
                         << double(end_undistort - start_undistort)
                         << " seconds.";
  }

  if (use_icp_) {
    LOG_IF(INFO, debug_) << "Calling ICP Plugin with stamp: "
                         << omnimapper::StampToPtime(cloud->header.stamp);
    icp_plugin_.CloudCallback(cloud);
  }

  if (use_organized_feature_extraction_) {
    const double start_ofe = pcl::getTime();
    organized_feature_extraction_.CloudCallback(cloud);
    const double end_ofe = pcl::getTime();
    LOG_IF(INFO, debug_) << "CloudCallback: ofe_cb took "
                         << double(end_ofe - start_ofe) << " seconds.";
  }

  if (add_pose_per_cloud_) {
    const double start_getpose = pcl::getTime();
    gtsam::Symbol sym;
    boost::posix_time::ptime header_time =
        omnimapper::StampToPtime(cloud->header.stamp);
    LOG_IF(INFO, debug_) << "Header time: " << header_time << std::endl;
    omb_.GetPoseSymbolAtTime(header_time, &sym);
    const double end_getpose = pcl::getTime();
    LOG_IF(INFO, debug_) << "CloudCallback: get_pose took "
                         << double(end_getpose - start_getpose) << " seconds.";
  }

  if (broadcast_map_to_odom_) {
    double start_pub = pcl::getTime();
    PublishMapToOdom();
    double end_pub = pcl::getTime();
    if (debug_)
      std::cout << "cloudCallback: pub took " << double(end_pub - start_pub)
                << std::endl;
  }

  if (broadcast_current_pose_) {
    PublishCurrentPose();
  }

  const double end_cb = pcl::getTime();
  LOG_IF(INFO, debug_) << "CloudCallback: entire call back took "
                       << double(end_cb - start_cb) << " seconds";
}

template <typename PointT>
void OmniMapperROS<PointT>::LaserScanCallback(
    const sensor_msgs::LaserScanConstPtr& msg) {
  LOG(INFO) << "OmniMapperROS received a LaserScan.";
  gtsam::Symbol sym;
  boost::shared_ptr<sensor_msgs::LaserScan> lscan_ptr(
      new sensor_msgs::LaserScan(*msg));
  csm_plugin_.LaserScanCallback(lscan_ptr);
  PublishMapToOdom();
}

template <typename PointT>
bool OmniMapperROS<PointT>::GenerateMapTSDFCallback(
    omnimapper_ros::OutputMapTSDF::Request& req,
    omnimapper_ros::OutputMapTSDF::Response& res) {
  LOG(INFO) << "TSDF service called, calling tsdf plugin.";
  tsdf_plugin_.GenerateTSDF(req.grid_size, req.resolution);
  return true;
}

template <typename PointT>
void OmniMapperROS<PointT>::EvalTimerCallback(const ros::TimerEvent& e) {
  const double cb_start = pcl::getTime();

  LOG(INFO) << "Evaluation timer call back at " << cb_start;

  // Check if everything is done processing
  bool ready = true;
  if (use_icp_ && !icp_plugin_.Ready()) ready = false;
  if (use_occ_edge_icp_ && !edge_icp_plugin_.Ready()) ready = false;
  if (evaluation_mode_paused_) ready = false;
  if (use_organized_feature_extraction_ &&
      !organized_feature_extraction_.Ready()) {
    ready = false;
  }

  if (ready &&
      (evaluation_file_idx_ < static_cast<int>(evaluation_pcd_files_.size()))) {
    // Load next file
    CloudPtr cloud(new Cloud());
    const double load_start = pcl::getTime();
    pcl::io::loadPCDFile(evaluation_pcd_files_[evaluation_file_idx_], *cloud);
    const double load_end = pcl::getTime();

    LOG_IF(INFO, debug_) << "Loading took: " << double(load_end - load_start)
                         << " seconds";
    LOG_IF(INFO, debug_) << "Processing cloud " << evaluation_file_idx_
                         << " with " << cloud->points.size() << "points.";

    // Convert it.
    sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2());
    // We have to get the timestamp from associated.txt (parsed by eval plugin)
    // since PCD doesn't include timestamps...

    cloud->header.stamp = eval_plugin_.GetStampFromIndex(evaluation_file_idx_);
    pcl::toROSMsg(*cloud, *cloud_msg);

    evaluation_file_idx_++;

    sensor_msgs::PointCloud2ConstPtr cloud_msg_ptr(cloud_msg);
    CloudCallback(cloud_msg_ptr);

    // TODO: Visualization adds a lag  each time it is called, delays if other
    // plugins are ready or not.
    if (evaluation_show_frames_) {
      eval_plugin_.VisualizeEachFrame(cloud);  // visualize each frame
      eval_plugin_.VisualizeStats();
    }

    ready = false;
  } else {
    LOG(ERROR) << "Plugins not yet ready.";
  }

  // Write the trajectory if we're finished
  if (ready && (evaluation_file_idx_ ==
                static_cast<int>(evaluation_pcd_files_.size()))) {
    LOG(INFO) << "Completed evaluation, writing output file.";
    gtsam::Values solution = omb_.GetSolution();

    if (evaluation_mode_write_trajectory_) {
      eval_plugin_.WriteMapperTrajectoryFile(
          evaluation_output_trajectory_txt_path_, solution);
      LOG(INFO) << "Evaluation log written as: "
                << evaluation_output_trajectory_txt_path_;
    }

    if (evaluation_mode_write_tsdf_) {
      tsdf_plugin_.GenerateTSDF(10.0, 1024);
    }

    exit(0);
  }

  const double cb_end = pcl::getTime();
  LOG(INFO) << "Timer callback took: " << double(cb_end - cb_start)
            << " seconds";
}

template <typename PointT>
void OmniMapperROS<PointT>::PublishMapToOdom() {
  gtsam::Pose3 current_pose;
  boost::posix_time::ptime current_time;
  omb_.GetLatestPose(&current_pose, &current_time);
  ros::Time current_time_ros = omnimapper::PtimeToRosTime(current_time);
  tf::Transform current_pose_ros = omnimapper::Pose3ToTf(current_pose);

  tf::Stamped<tf::Pose> odom_to_map;
  try {
    tf_listener_.waitForTransform(odom_frame_name_, base_frame_name_,
                                  current_time_ros, ros::Duration(0.05));
    tf_listener_.transformPose(
        odom_frame_name_,
        tf::Stamped<tf::Pose>(current_pose_ros.inverse(), current_time_ros,
                              base_frame_name_),
        odom_to_map);
  } catch (tf::TransformException e) {
    LOG(ERROR) << "Could nort immediately get odom to base transform.";
    odom_to_map.setIdentity();
    return;
  }
  tf::Transform map_to_odom = odom_to_map.inverse();
  tf_broadcaster_.sendTransform(tf::StampedTransform(
      map_to_odom, ros::Time::now(), "/world", odom_frame_name_));
}

template <typename PointT>
void OmniMapperROS<PointT>::PublishCurrentPose() {
  gtsam::Pose3 current_pose;
  boost::posix_time::ptime current_time;
  omb_.GetLatestPose(&current_pose, &current_time);
  ros::Time current_time_ros = omnimapper::PtimeToRosTime(current_time);
  tf::Transform current_pose_ros = omnimapper::Pose3ToTf(current_pose);
  tf_broadcaster_.sendTransform(tf::StampedTransform(
      current_pose_ros, ros::Time::now(), "/world", "/current_pose"));
}

template <typename PointT>
void OmniMapperROS<PointT>::ResetEvaluation() {
  evaluation_pcd_files_.clear();

  // ICP Plugin
  icp_plugin_.SetUseGICP(true);
  icp_plugin_.SetOverwriteTimestamps(false);
  icp_plugin_.SetAddIdentityOnFailure(icp_add_identity_on_fail_);
  icp_plugin_.SetShouldDownsample(true);
  icp_plugin_.SetLeafSize(icp_leaf_size_);
  icp_plugin_.SetMaxCorrespondenceDistance(icp_max_correspondence_distance_);
  icp_plugin_.SetScoreThreshold(icp_score_thresh_);
  icp_plugin_.SetTransNoise(icp_trans_noise_);
  icp_plugin_.SetRotNoise(icp_rot_noise_);
  icp_plugin_.SetAddLoopClosures(icp_add_loop_closures_);
  icp_plugin_.SetTransNoise(icp_trans_noise_);
  icp_plugin_.SetRotNoise(icp_rot_noise_);
  icp_plugin_.SetLoopClosureDistanceThreshold(
      icp_loop_closure_distance_threshold_);
  icp_plugin_.SetSaveFullResClouds(true);
}

template class OmniMapperROS<pcl::PointXYZRGBA>;
