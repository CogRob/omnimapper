#include <omnimapper_ros/canonical_scan_matcher_plugin.h>
#include <omnimapper_ros/csm_math_functions.h>
#include <omnimapper/omnimapper_base.h>
#include <omnimapper/time.h>

gtsam::Pose3 doCSM_impl ( const sensor_msgs::LaserScan& from_scan,
                          const sensor_msgs::LaserScan& to_scan,
                          const gtsam::Pose3& odometry_relative_pose,
                          bool& worked_,
                          gtsam::noiseModel::Gaussian::shared_ptr& noise_model,
                          scan_tools::CanonicalScan& canonicalScan,
                          tf::Transform& base_to_laser_tf,
                          bool laser_mode)
{
    gtsam::Pose3 result_pose = odometry_relative_pose;
    LDP from_ldp, to_ldp;

    canonicalScan.laserScanToLDP ( from_scan,
                                   from_ldp);
    canonicalScan.laserScanToLDP ( to_scan,
                                   to_ldp);

    gtsam::Pose2 outp_rel_pose_laser;

    tf::Transform base_to_laser_ = base_to_laser_tf;
    tf::Transform laser_to_base_ = base_to_laser_.inverse();
    tf::Transform pr_ch = Pose3ToTransform(odometry_relative_pose);
    tf::Transform pr_ch_l = laser_to_base_ * pr_ch * base_to_laser_;
    gtsam::Pose3 pr_ch_l_pose3 = TransformToPose3(pr_ch_l);

    worked_ =   canonicalScan.processScan(from_ldp, to_ldp,
                                          Pose3ToPose2(pr_ch_l_pose3),
                                          outp_rel_pose_laser,
                                          noise_model);
    
    tf::Transform corr_ch_l = Pose3ToTransform(Pose2ToPose3(outp_rel_pose_laser));
    tf::Transform corr_ch = base_to_laser_ * corr_ch_l * laser_to_base_;
    outp_rel_pose_laser = Pose3ToPose2(TransformToPose3(corr_ch));

    if (worked_ )
    {
      gtsam::Pose3 p3 = Pose2ToPose3(outp_rel_pose_laser);
      result_pose = p3;
    }
    else
    {
        result_pose = odometry_relative_pose;
    }
    return (result_pose);
}


sensor_msgs::LaserScan SmoothScan (const sensor_msgs::LaserScanConstPtr& msg_in)
{
    return *msg_in;
    sensor_msgs::LaserScan msg_out = *msg_in;

    if (msg_out.ranges.size() < 3) return msg_out;

    std::vector<float> new_ranges = msg_out.ranges;
    new_ranges[0] = (msg_out.ranges[0] + msg_out.ranges[1]) /2.0;

    for (unsigned int i = 1; i < msg_out.ranges.size();i++)
    {
        if ( msg_out.ranges[i-1] < msg_out.range_max &&
                msg_out.ranges[i-1] > msg_out.range_min &&
                msg_out.ranges[i] < msg_out.range_max &&
                msg_out.ranges[i] > msg_out.range_min &&
                msg_out.ranges[i+1] < msg_out.range_max &&
                msg_out.ranges[i+1] > msg_out.range_min)
        {
            new_ranges[i] = (msg_out.ranges[i-1]+
                             msg_out.ranges[i] +
                             msg_out.ranges[i+1])/3.0;

            if (fabs(new_ranges[i] - msg_out.ranges[i]) > 0.1)
                new_ranges[i] = msg_out.ranges[i];
        }
    }

    new_ranges[msg_out.ranges.size()-1] = (msg_out.ranges[msg_out.ranges.size()-1] + msg_out.ranges[msg_out.ranges.size()-2]) /2.0;
    msg_out.ranges = new_ranges;
    return msg_out;
}

gtsam::Point3 GetMeanLaserPoint(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > cloud)
{
    double mean_x = 0.0;
    double mean_y = 0.0;
    double mean_z = 0.0;
    for (unsigned int i = 0;i<cloud->points.size();i++)
    {
        mean_x += cloud->points[i].x;
        mean_y += cloud->points[i].y;
        mean_z += cloud->points[i].z;
    }

    return gtsam::Point3 (mean_x / (double)(cloud->points.size()),
                          mean_y / (double)(cloud->points.size()),
                          mean_z / (double)(cloud->points.size()));
}



namespace omnimapper
{
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template <typename LScanT>
  CanonicalScanMatcherPlugin<LScanT>::CanonicalScanMatcherPlugin (omnimapper::OmniMapperBase* mapper) :
    nh_ ("~"),
    mapper_ (mapper),
    initialized_ (false),
    have_new_lscan_ (false),
    first_ (true),
    downsample_ (true),
    leaf_size_ (0.05f),
    score_threshold_ (0.5),
    trans_noise_ (1.0),
    rot_noise_ (1.0),
    debug_ (false),
    overwrite_timestamps_ (true),
    base_frame_name_ (std::string ("/base_link")),
    previous_sym_ (gtsam::Symbol ('x', 0)),
    previous2_sym_ (gtsam::Symbol ('x', 0)),
    previous3_sym_ (gtsam::Symbol ('x', 0)),
    add_identity_on_failure_ (false),
    add_multiple_links_ (false),
    add_loop_closures_ (true),
    paused_ (false),
    count_(0),
    trigger_ (new omnimapper::TriggerAlways()),
  triggered_time_ (ros::Time::now ().toBoost()),
  tf_listener_ (ros::Duration (100.0))
    {
      seq_canonical_scan_.initParams (nh_);
      lc_canonical_scan_.initParams (nh_);
      have_new_lscan_ = false;
      first_ = true;

      laser_scan_msg_pub1_ = nh_.advertise<sensor_msgs::LaserScan>("scan1", 0);
      laser_scan_msg_pub2_ = nh_.advertise<sensor_msgs::LaserScan>("scan2", 0);
      visualization_marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 0);
    }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template <typename LScanT> CanonicalScanMatcherPlugin<LScanT>::~CanonicalScanMatcherPlugin ()
    {

    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template <typename LScanT> void CanonicalScanMatcherPlugin<LScanT>::laserScanCallback (const LaserScanPtr& lscan)
    {
      if (debug_)
        std::cout << "CSMPlugin: laser callback: " << ros::Time::now () << std::endl;

      Time measurement_time = lscan->header.stamp.toBoost();
      bool use_measurement = (*trigger_)(measurement_time);
      if (!use_measurement)
        return;

      boost::mutex::scoped_lock lock (current_lscan_mutex_);

      current_lscan_ = lscan;
      have_new_lscan_ = true;
      scan_cv_.notify_one();
      if (debug_)
        printf ("CSMPlugin: stored new scan!\n");
    }

 //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template <typename LScanT> void CanonicalScanMatcherPlugin<LScanT>::spin()
    {
        while (true)
        {
            spinOnce ();
            boost::this_thread::sleep (boost::posix_time::milliseconds (10));
        }
    }


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template <typename LScanT> bool CanonicalScanMatcherPlugin<LScanT>:: spinOnce()
  {
    ros::Time start_time = ros::Time::now ();
    if (debug_)
      std::cout << "CSMPlugin: spin start: " << start_time << std::endl;

    LaserScanPtr current_lscan;

    {
      boost::mutex::scoped_lock lock(current_lscan_mutex_);
      while (!have_new_lscan_)
      {
        scan_cv_.wait(lock);
      }
      current_lscan = current_lscan_;
      have_new_lscan_ = false;
    }

    gtsam::Symbol current_sym;
    boost::posix_time::ptime current_time = current_lscan->header.stamp.toBoost();//omnimapper::stamp2ptime(current_lscan->header.stamp);

    mapper_->getPoseSymbolAtTime (current_time, current_sym);

    if (debug_)
      std::cout << "Pose symbol for current scan: " << current_sym.index() << std::endl;
    lscans_.insert(std::pair<gtsam::Symbol, LaserScanPtr>(current_sym, current_lscan));
    
    try
    {
      sensor_msgs::PointCloud2 scan_cloud;
      projector_.transformLaserScanToPointCloud (base_frame_name_, *current_lscan, scan_cloud, tf_listener_);
      clouds_.insert (std::pair<gtsam::Symbol, sensor_msgs::PointCloud2>(current_sym, scan_cloud));

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg (scan_cloud, *cloud);

      gtsam::Point3 laser_centroid = GetMeanLaserPoint (cloud);
      lscan_centroids_.insert (std::pair<gtsam::Symbol, gtsam::Point3>(current_sym, laser_centroid));
    }
     catch (tf::TransformException ex)
     {
       ROS_WARN ("No transform yet!\n");
       return false;
     }


    if (first_)
    {
      getBaseToLaserTf (current_lscan->header.frame_id);
      previous_sym_ = current_sym;
      first_ = false;
      return (false);
    }

    // current pose to previous pose
    boost::thread latest_icp_thread (&CanonicalScanMatcherPlugin<LScanT>::addConstraint, this, previous_sym_, current_sym, seq_canonical_scan_, true);
    // The library is not thread safe, so we must join here -- cannot parallelize loop closure w/ sequential
    latest_icp_thread.join ();
    if (add_loop_closures_)
    {
      if (lscans_.size () > 20)
      {
        boost::thread loop_closure_thread (&CanonicalScanMatcherPlugin<LScanT>::tryLoopClosure, this, previous_sym_);
        loop_closure_thread.join ();
      }
      
    }
    //latest_icp_thread.join ();

    if (debug_)
    {
      ros::Time end_time = ros::Time::now ();
      ros::Duration spin_time = end_time - start_time;
      std::cout << "CSMPlugin: spin end: " << spin_time << std::endl;
    }

    {
      previous_sym_ = current_sym;
      return (true);
    }

  }

  template <typename LScanT> bool
  CanonicalScanMatcherPlugin<LScanT>::getBaseToLaserTf (const std::string& frame_id)
  {
    ros::Time t = ros::Time::now();
    
    tf::StampedTransform base_to_laser_tf;
    try
    {
    tf_listener_.waitForTransform(
                                  base_frame_name_, frame_id, t, ros::Duration(1.0));
    tf_listener_.lookupTransform (
                                  base_frame_name_, frame_id, t, base_to_laser_tf);
    }
    catch (tf::TransformException ex)
    {
      ROS_WARN("Could not get initial transform from base to laser frame, %s", ex.what());
      return false;
    }
    base_to_laser_ = base_to_laser_tf;
    laser_to_base_ = base_to_laser_.inverse();
    
    return true;
  }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename LScanT> bool
CanonicalScanMatcherPlugin<LScanT>::addConstraint(gtsam::Symbol sym1, gtsam::Symbol sym2, scan_tools::CanonicalScan& cscan, bool always_add)
  {
    LaserScanPtr scan1 = lscans_.at(sym1);
    LaserScanPtr scan2 = lscans_.at(sym2);
    if (!(scan1 && scan2))
    {
        printf("CSMPlugin: Don't have laser scans for these poses!\n");
        return(false);
    }

    boost::optional<gtsam::Pose3> scan1_pose = mapper_->predictPose (sym1);
    boost::optional<gtsam::Pose3> scan2_pose = mapper_->predictPose (sym2);

    Eigen::Matrix4f cloud_tform;
    gtsam::Pose3 initial_guess;
    gtsam::Pose3 initial_guess_inv;
    if ((scan1_pose) && (scan2_pose))
    {
      initial_guess = scan1_pose->between (*scan2_pose);
      cloud_tform = initial_guess.matrix ().cast<float>();
      initial_guess_inv = scan2_pose->between (*scan1_pose);
    }
    else
    {
      cloud_tform = Eigen::Matrix4f::Identity ();
      initial_guess = gtsam::Pose3::identity ();
    }

    bool worked = false;
    gtsam::noiseModel::Gaussian::shared_ptr noise;//(new gtsam::noiseModel::Gaussian());
 
    gtsam::Pose3 relative_pose = doCSM_impl(*scan1, *scan2, initial_guess, worked, noise, cscan, base_to_laser_, true);

    //debug
    if (debug_)
    {
      LaserScanPtr vis_scan1 (new LScan (*scan1));
      vis_scan1->header.frame_id = "/odom";
      LaserScanPtr vis_scan2 (new LScan (*scan2));
      vis_scan2->header.frame_id = "/odom";
      laser_scan_msg_pub1_.publish (*vis_scan1);
      laser_scan_msg_pub2_.publish (*vis_scan2);
      visualization_msgs::MarkerArray marker_array;

      gtsam::Vector csm_quat = relative_pose.rotation ().quaternion ();
      visualization_msgs::Marker csm_arrow;
      csm_arrow.header.frame_id = "/odom";
      csm_arrow.header.stamp = ros::Time ();
      csm_arrow.ns = "csm_estimate";
      csm_arrow.id = 0;
      csm_arrow.type = visualization_msgs::Marker::ARROW;
      csm_arrow.action = visualization_msgs::Marker::ADD;
      csm_arrow.color.a = 0.5;
      csm_arrow.color.r = 1.0;
      csm_arrow.color.g = 0.0;
      csm_arrow.color.b = 0.0;
      csm_arrow.scale.x = 1.0;
      csm_arrow.scale.y = 0.1;
      csm_arrow.scale.z = 0.1;
      csm_arrow.pose.position.x = 0.0;//relative_pose.x ();
      csm_arrow.pose.position.y = 0.0;//relative_pose.y ();
      csm_arrow.pose.position.z = 0.0;//relative_pose.z ();
      csm_arrow.pose.orientation.w = csm_quat[0];
      csm_arrow.pose.orientation.x = csm_quat[1];
      csm_arrow.pose.orientation.y = csm_quat[2];
      csm_arrow.pose.orientation.z = csm_quat[3];
      marker_array.markers.push_back (csm_arrow);

      visualization_msgs::Marker csm_sphere;
      csm_sphere.header.frame_id = "/odom";
      csm_sphere.header.stamp = ros::Time ();
      csm_sphere.ns = "csm_pose";
      csm_sphere.id = 0;
      csm_sphere.type = visualization_msgs::Marker::SPHERE;
      csm_sphere.action = visualization_msgs::Marker::ADD;
      csm_sphere.color.a = 0.5;
      csm_sphere.color.r = 1.0;
      csm_sphere.color.g = 0.0;
      csm_sphere.color.b = 0.0;
      csm_sphere.scale.x = 0.1;
      csm_sphere.scale.y = 0.1;
      csm_sphere.scale.z = 0.1;
      csm_sphere.pose.position.x = relative_pose.x ();
      csm_sphere.pose.position.y = relative_pose.y ();
      csm_sphere.pose.position.z = relative_pose.z ();
      marker_array.markers.push_back (csm_sphere);

      gtsam::Vector odom_quat = initial_guess.rotation ().quaternion ();
      visualization_msgs::Marker odom_arrow;
      odom_arrow.header.frame_id = "/odom";
      odom_arrow.header.stamp = ros::Time ();
      odom_arrow.ns = "odom_estimate";
      odom_arrow.id = 0;
      odom_arrow.type = visualization_msgs::Marker::ARROW;
      odom_arrow.action = visualization_msgs::Marker::ADD;
      odom_arrow.color.a = 0.5;
      odom_arrow.color.r = 0.0;
      odom_arrow.color.g = 1.0;
      odom_arrow.color.b = 0.0;
      odom_arrow.scale.x = 1.0;
      odom_arrow.scale.y = 0.1;
      odom_arrow.scale.z = 0.1;
      odom_arrow.pose.position.x = 0.0;//initial_guess.x ();
      odom_arrow.pose.position.y = 0.0;//initial_guess.y ();
      odom_arrow.pose.position.z = 0.0;//initial_guess.z ();
      odom_arrow.pose.orientation.w = odom_quat[0];
      odom_arrow.pose.orientation.x = odom_quat[1];
      odom_arrow.pose.orientation.y = odom_quat[2];
      odom_arrow.pose.orientation.z = odom_quat[3];
      marker_array.markers.push_back (odom_arrow);

      visualization_msgs::Marker odom_sphere;
      odom_sphere.header.frame_id = "/odom";
      odom_sphere.header.stamp = ros::Time ();
      odom_sphere.ns = "odom_position";
      odom_sphere.id = 0;
      odom_sphere.type = visualization_msgs::Marker::SPHERE;
      odom_sphere.action = visualization_msgs::Marker::ADD;
      odom_sphere.color.a = 0.5;
      odom_sphere.color.r = 0.0;
      odom_sphere.color.g = 1.0;
      odom_sphere.color.b = 0.0;
      odom_sphere.scale.x = 0.1;
      odom_sphere.scale.y = 0.1;
      odom_sphere.scale.z = 0.1;
      odom_sphere.pose.position.x = initial_guess.x ();
      odom_sphere.pose.position.y = initial_guess.y ();
      odom_sphere.pose.position.z = initial_guess.z ();
      marker_array.markers.push_back (odom_sphere);

      visualization_marker_array_pub_.publish (marker_array);
    }

    if (!always_add)
      noise = gtsam::noiseModel::Diagonal::Sigmas ((gtsam::Vector (6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1));

    if (debug_)
      printf("CSM Relative Pose: %lf %lf %lf", relative_pose.x(), relative_pose.y(), relative_pose.z());

    omnimapper::OmniMapperBase::NonlinearFactorPtr between (new gtsam::BetweenFactor<gtsam::Pose3>(sym1, sym2, relative_pose, noise));
    if (debug_)
      between->print ("CSM BetweenFactor:\n");
    if (worked || always_add)
      mapper_->addFactor(between);
    return (true);
  }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template <typename LScanT> bool CanonicalScanMatcherPlugin<LScanT>::tryLoopClosure(gtsam::Symbol sym)
  {
    double loop_closure_dist_thresh_ = 3.0;
    int pose_index_thresh_ = 40;

    // Get the latest solution from the mapper
    gtsam::Values solution = mapper_->getSolution ();

    boost::optional<gtsam::Pose3> current_pose = mapper_->predictPose(sym);
    
    if (!current_pose)
      {
	printf ("Could not predict pose in Laser Scan loop closure!\n");
	exit(1);
      }

    gtsam::Point3 current_centroid = lscan_centroids_[sym];
    gtsam::Point3 current_centroid_map = current_pose->transform_from (current_centroid);

    // Find the closest pose
    gtsam::Values::ConstFiltered<gtsam::Pose3> pose_filtered = solution.filter<gtsam::Pose3>();
    double min_dist = std::numeric_limits<double>::max ();
    gtsam::Symbol closest_sym;
    BOOST_FOREACH (const gtsam::Values::ConstFiltered<gtsam::Pose3>::KeyValuePair& key_value, pose_filtered)
    {
      gtsam::Symbol test_sym (key_value.key);
      int sym_dist = sym.index () - test_sym.index ();

      if (sym_dist > pose_index_thresh_)
      {
        gtsam::Pose3 test_pose (key_value.value);
        double test_dist = current_pose->range (test_pose);

        // Compute distance from scan_centroids
        gtsam::Point3 test_centroid = lscan_centroids_[test_sym];
        gtsam::Point3 test_centroid_map = test_pose.transform_from (test_centroid);
        
        double centroid_dist = fabs (test_centroid_map.distance (current_centroid_map));

        std::map<gtsam::Symbol, LaserScanPtr>::iterator itr1;
        itr1 = lscans_.find (test_sym);

        if ((centroid_dist < min_dist) && (itr1 != lscans_.end ()))
        {
          min_dist = test_dist;
          closest_sym = key_value.key;
        }
      }
    }

    // If we found something, try to add a link
    if (min_dist < loop_closure_dist_thresh_)
    {
       //todo cn:check
      addConstraint (closest_sym, sym, lc_canonical_scan_, false);

      if (debug_)
        printf ("ADDED LOOP CLOSURE BETWEEN %d and %d!\n", sym.index (), closest_sym.index ());
      return (true);
    }
    else
    {
      return (false);
    }
  }


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template <typename LScanT> bool CanonicalScanMatcherPlugin<LScanT>::ready ()
  {
    boost::mutex::scoped_lock (current_lscan_mutex_);
    return (have_new_lscan_);
  }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template <typename LScanT> typename omnimapper::CanonicalScanMatcherPlugin<LScanT>::LaserScanPConstPtr CanonicalScanMatcherPlugin<LScanT>::getLaserScanPtr (gtsam::Symbol sym)
  {
    if (lscans_.count (sym) > 0)
      return (lscans_.at (sym));
    else
    {
      if (debug_)
        printf ("ERROR: REQUESTED SYMBOL WITH NO POINTS!\n");
      LaserScanPConstPtr empty (new LScanT ());
      return (empty);
    }
  }

  template<typename LScanT> sensor_msgs::PointCloud2
  CanonicalScanMatcherPlugin<LScanT>::getPC2 (gtsam::Symbol sym)
  {
    if (clouds_.count (sym) > 0)
      return (clouds_.at (sym));
    else
    {
      sensor_msgs::PointCloud2 empty;
      return (empty);
    }
  }

}

template class omnimapper::CanonicalScanMatcherPlugin<sensor_msgs::LaserScan>;
