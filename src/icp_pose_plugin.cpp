#include <omnimapper/icp_pose_plugin.h>
#include <omnimapper/omnimapper_base.h>
#include <omnimapper/time.h>

#include <pcl/common/centroid.h>
#include <pcl/common/time.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

namespace omnimapper {
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
ICPPoseMeasurementPlugin<PointT>::ICPPoseMeasurementPlugin(
    omnimapper::OmniMapperBase* mapper)
    : mapper_(mapper),
      get_sensor_to_base_(GetTransformFunctorPtr()),
      last_processed_time_(),
      initialized_(false),
      have_new_cloud_(false),
      first_(true),
      downsample_(true),
      leaf_size_(0.05f),
      score_threshold_(0.5),
      trans_noise_(1.0),
      rot_noise_(1.0),
      debug_(true),
      overwrite_timestamps_(true),
      icp_max_correspondence_distance_(3.5),
      previous_sym_(gtsam::Symbol('x', 0)),
      previous2_sym_(gtsam::Symbol('x', 0)),
      previous3_sym_(gtsam::Symbol('x', 0)),
      use_gicp_(true),
      add_identity_on_failure_(false),
      add_multiple_links_(false),
      add_loop_closures_(false),
      loop_closure_distance_threshold_(0.1),
      paused_(false),
      save_full_res_clouds_(false) {
  have_new_cloud_ = false;
  first_ = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
ICPPoseMeasurementPlugin<PointT>::~ICPPoseMeasurementPlugin() {}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void ICPPoseMeasurementPlugin<PointT>::cloudCallback(
    const CloudConstPtr& cloud) {
  if (debug_) printf("cloud callback\n");
  // Store this as the previous cloud
  boost::mutex::scoped_lock(current_cloud_mutex_);
  current_cloud_ = cloud;

  //
  if (have_new_cloud_) {
    if (debug_) printf("got new cloud before done processing the old one!\n");
    // assert (false);
  }

  have_new_cloud_ = true;
  if (debug_) printf("ICPTest: stored new cloud!\n");
}

/*
template <typename PointT> bool
ICPPoseMeasurementPlugin<PointT>::addInitialPose ()
{
  // Do nothing if there isn't a new cloud
  if (!have_new_cloud_)
  {
    printf ("no new cloud!\n");
    return false;
  }

  gtsam::Symbol current_symbol = mapper_->currentPoseSymbol ();
  printf ("current symbol: %zu\n", current_symbol.index ());
  {
    boost::mutex::scoped_lock (current_cloud_mutex_);
    clouds_.insert (std::pair<gtsam::Symbol, CloudConstPtr> (current_symbol,
current_cloud_));

    // If this is the first cloud, just add it and we're done -- no motion has
occured
    // Note that initialization of the first pose is done by the OmniMapperBase
    if (clouds_.size () == 1)
    {
      initialized_ = true;
      have_new_cloud_ = false;
      return (true);
    }
  }
}
*/

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void ICPPoseMeasurementPlugin<PointT>::spin() {
  // while (grabber_.isRunning ())
  while (true) {
    spinOnce();
    boost::this_thread::sleep(boost::posix_time::milliseconds(10));
  }
}

template <typename PointT>
bool ICPPoseMeasurementPlugin<PointT>::spinOnce() {
  // Do nothing if there isn't a new cloud
  if (!have_new_cloud_) {
    // printf ("ICPPoseMeasurementPlugin: no new cloud!\n");
    return false;
  }

  double spin_start = pcl::getTime();

  // Get a shared_ptr to the latest cloud
  CloudConstPtr current_cloud;  // (new Cloud ());
  {
    boost::mutex::scoped_lock(current_cloud_mutex_);

    // Strict input checking
    if (false) {
      if (current_cloud_->points.size() < 200) {
        printf("ICPPoseMeasurementPlugin: Not enough points!\n");
        return (false);
      }

      // for (int i = 0; i < current_cloud_->points.size (); i++)
      // {
      //   if (!pcl::isFinite (current_cloud_->points[i]))
      //   {
      //     printf ("NAN FOUND IN INPUT!");
      //     return (false);
      //   }
      // }
    }

    current_cloud = current_cloud_;
  }

  if (debug_)
    printf("current cloud points: %zu\n", current_cloud->points.size());

  // Downsample, if needed
  CloudPtr current_cloud_filtered(new Cloud());
  if (downsample_) {
    pcl::VoxelGrid<PointT> grid;
    grid.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    grid.setInputCloud(current_cloud);
    grid.filter(*current_cloud_filtered);
  } else {
    *current_cloud_filtered = *current_cloud;
  }

  // Get the previous pose and cloud
  // gtsam::Symbol current_sym = mapper_->currentPoseSymbol ();
  gtsam::Symbol current_sym;
  boost::posix_time::ptime current_time;
  if (overwrite_timestamps_) {
    current_time = boost::posix_time::ptime(
        boost::posix_time::microsec_clock::local_time());
  } else {
    // #ifdef USE_ROS
    //       current_time = current_cloud->header.stamp.toBoost
    //       ();//current_cloud_->header.stamp.toBoost ();
    // #else
    //       current_time = boost::posix_time::ptime( omnimapper::stamp2ptime
    //       (current_cloud->header.stamp));//(current_cloud_->header.stamp) );
    // #endif //USE_ROS
    current_time = omnimapper::stamp2ptime(current_cloud->header.stamp);
  }

  // Apply sensor to base transform, if we have one
  CloudPtr current_cloud_base(new Cloud());
  if (get_sensor_to_base_) {
    if (debug_) printf("ICPPosePlugin: Applying sensor to base transform\n");
    std::cout << "Timestamp: " << current_time << std::endl;
    Eigen::Affine3d sensor_to_base = (*get_sensor_to_base_)(current_time);
    pcl::transformPointCloud(*current_cloud_filtered, *current_cloud_base,
                             sensor_to_base);
  } else {
    if (debug_) printf("ICPPosePlugin: No sensor to base transform exists!\n");
  }

  if (debug_)
    std::cout << "ICP Plugin: Getting symbol for current time: " << current_time
              << std::endl;
  mapper_->getPoseSymbolAtTime(current_time, current_sym);
  // std::cout << "stamp time: " << current_cloud_->header.stamp << " converted
  // time: " << current_time << std::endl;
  if (debug_)
    printf("ICP Plugin: current symbol: %zu, inserting cloud\n",
           current_sym.index());
  //{
  //  boost::mutex::scoped_lock (current_cloud_mutex_);
  clouds_.insert(std::pair<gtsam::Symbol, CloudConstPtr>(
      current_sym, current_cloud_base));  // current_cloud_));

  // Compute and save the cloud centroid, for use in loop closure detection
  Eigen::Vector4f cloud_centroid;
  pcl::compute3DCentroid(*current_cloud_base, cloud_centroid);
  gtsam::Point3 centroid_pt(cloud_centroid[0], cloud_centroid[1],
                            cloud_centroid[2]);
  cloud_centroids_.insert(
      std::pair<gtsam::Symbol, gtsam::Point3>(current_sym, centroid_pt));

  //}

  if (save_full_res_clouds_) {
    if (debug_)
      printf("ICPPlugin: Saving full res cloud with %zu\n",
             current_cloud->points.size());
    // full_res_clouds_.insert (std::pair<gtsam::Symbol, CloudConstPtr>
    // (current_sym, current_cloud)); CloudPtr full_res_cloud_base (new Cloud
    // ());
    Eigen::Affine3d sensor_to_base = (*get_sensor_to_base_)(current_time);
    // pcl::transformPointCloud (*current_cloud, *full_res_cloud_base,
    // sensor_to_base);

    std::string out_file = "/tmp/" + std::string(current_sym) + ".pcd";
    full_res_clouds_.insert(
        std::pair<gtsam::Symbol, std::string>(current_sym, out_file));
    pcl::io::savePCDFileBinaryCompressed(out_file, *current_cloud);
    sensor_to_base_transforms_.insert(
        std::pair<gtsam::Symbol, Eigen::Affine3d>(current_sym, sensor_to_base));
    // pcl::io::savePCDFileBinaryCompressed (out_file, *full_res_cloud_base);
  }

  // We're done if that was the first cloud
  // printf ("first: %d\n", first_);
  if (first_) {
    boost::mutex::scoped_lock(current_cloud_mutex_);
    if (debug_) printf("ICPTest: done with first, returning\n");
    have_new_cloud_ = false;
    previous_sym_ = current_sym;
    first_ = false;
    return (false);
  }

  // Add constraints
  if (debug_)
    printf("ICP SYMS: prev3: x%zu, prev2: x%zu  prev: x%zu, curr: x%zu\n",
           previous3_sym_.index(), previous2_sym_.index(),
           previous_sym_.index(), current_sym.index());

  // boost::thread latest_icp_thread
  // (&ICPPoseMeasurementPlugin<PointT>::addConstraint, this, current_sym,
  // previous_sym_, score_threshold_);
  boost::thread latest_icp_thread(
      &ICPPoseMeasurementPlugin<PointT>::addConstraint, this, previous_sym_,
      current_sym, score_threshold_);
  // Try previous too
  if (add_multiple_links_) {
    if (clouds_.size() >= 3) {
      boost::thread prev2_icp_thread(
          &ICPPoseMeasurementPlugin<PointT>::addConstraint, this, previous_sym_,
          previous3_sym_, true);
      prev2_icp_thread.join();
      if (debug_) printf("PREV 2 COMPLETE!\n");
    }
    if (clouds_.size() >= 4) {
      boost::thread prev3_icp_thread(
          &ICPPoseMeasurementPlugin<PointT>::addConstraint, this, previous_sym_,
          previous3_sym_, score_threshold_);
      prev3_icp_thread.join();
      if (debug_) printf("PREV 3 COMPLETE!\n");
    }
  }

  if (add_loop_closures_) {
    if (clouds_.size() > 20) {
      boost::thread loop_closure_thread(
          &ICPPoseMeasurementPlugin<PointT>::tryLoopClosure, this,
          previous3_sym_);
      loop_closure_thread.join();
    }
  }

  // Wait for latest one to complete, at least
  latest_icp_thread.join();

  // Note that we're done
  {
    boost::mutex::scoped_lock(current_cloud_mutex_);
    if (debug_) printf("ICPTest: done with cloud!\n");
    previous3_sym_ = previous2_sym_;
    previous2_sym_ = previous_sym_;
    previous_sym_ = current_sym;
    last_processed_time_ = current_time;
    have_new_cloud_ = false;
  }

  double spin_end = pcl::getTime();
  if (debug_)
    std::cout << "ICP Plugin took " << double(spin_end - spin_start)
              << std::endl;

  if (debug_) printf("ICPPoseMeasurementPlugin: Added a pose!\n");
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool ICPPoseMeasurementPlugin<PointT>::addConstraint(
    gtsam::Symbol sym1, gtsam::Symbol sym2, double icp_score_threshold) {
  // Look up clouds
  CloudConstPtr cloud1 = clouds_.at(sym1);
  CloudConstPtr cloud2 = clouds_.at(sym2);
  if (!(cloud1 && cloud2)) {
    printf("Don't have clouds for these poses!\n");
    return (false);
  }

  // Look up initial guess, if applicable
  // boost::optional<gtsam::Pose3> cloud1_pose = mapper_->getPose
  // (sym1);//mapper_->predictPose (sym1); boost::optional<gtsam::Pose3>
  // cloud2_pose = mapper_->getPose (sym2);//mapper_->predictPose (sym2);
  boost::optional<gtsam::Pose3> cloud1_pose = mapper_->predictPose(sym1);
  boost::optional<gtsam::Pose3> cloud2_pose = mapper_->predictPose(sym2);

  // If we have an initial guess
  Eigen::Matrix4f cloud_tform;
  if ((cloud1_pose) && (cloud2_pose)) {
    if (debug_) {
      cloud1_pose->print("Pose1\n\n\n");
      printf("cloud1 pose det: %lf\n",
             cloud1_pose->rotation().matrix().determinant());
      cloud2_pose->print("Pose2\n\n\n");
      printf("cloud2 pose det: %lf\n",
             cloud2_pose->rotation().matrix().determinant());
    }

    gtsam::Pose3 initial_guess = cloud1_pose->between(
        *cloud2_pose);  // cloud1_pose->transform_to (*cloud2_pose);
    if (debug_) {
      initial_guess.print("\n\nInitial guess\n\n");
      printf("initial guess det: %lf\n",
             initial_guess.rotation().matrix().determinant());
    }

    cloud_tform = initial_guess.matrix().cast<float>();
  } else {
    cloud_tform = Eigen::Matrix4f::Identity();
  }

  CloudPtr aligned_cloud(new Cloud());
  // Eigen::Matrix4f cloud_tform = Eigen::Matrix4f::Identity ();
  double icp_score = 0.0;

  bool icp_converged =
      registerClouds(cloud1, cloud2, aligned_cloud, cloud_tform, icp_score);

  if (icp_converged && (icp_score < icp_score_threshold)) {
    // gtsam::Pose3 relative_pose (gtsam::Rot3 (cloud_tform.block (0, 0, 3,
    // 3).cast<double>()),
    //                            gtsam::Point3 (cloud_tform (0,3), cloud_tform
    //                            (1,3), cloud_tform (2,3)));
    Eigen::Matrix4d tform4d = cloud_tform.cast<double>();
    gtsam::Pose3 relative_pose(tform4d);
    // relative_pose = relative_pose.inverse ();

    // TODO: make these params
    double trans_noise = trans_noise_;  // * icp_score;
    double rot_noise = rot_noise_;      // * icp_score;
    gtsam::SharedDiagonal noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << rot_noise, rot_noise, rot_noise, trans_noise,
         trans_noise, trans_noise));

    // omnimapper::OmniMapperBase::NonlinearFactorPtr between (new
    // gtsam::BetweenFactor<gtsam::Pose3> (sym2, sym1, relative_pose, noise));
    omnimapper::OmniMapperBase::NonlinearFactorPtr between(
        new gtsam::BetweenFactor<gtsam::Pose3>(sym1, sym2, relative_pose,
                                               noise));

    if (debug_) {
      printf("ADDED FACTOR BETWEEN x%zu and x%zu\n", sym1.index(),
             sym2.index());
      relative_pose.print("\n\nICP Relative Pose\n");
      printf("ICP SCORE: %lf\n", icp_score);
      printf("relative pose det: %lf\n",
             relative_pose.rotation().matrix().determinant());
    }

    // if (direct)
    //  mapper_->addFactorDirect (between);
    // else
    mapper_->addFactor(between);
    return (true);
  } else {
    if (debug_) printf("ICP did not converge!\n");
    // Always add a pose
    if (add_identity_on_failure_) {
      Eigen::Matrix4f cloud_tform = Eigen::Matrix4f::Identity();
      // gtsam::Pose3 relative_pose (gtsam::Rot3 (cloud_tform.block (0, 0, 3,
      // 3).cast<double>()),
      //                             gtsam::Point3 (cloud_tform (0,3),
      //                             cloud_tform (1,3), cloud_tform (2,3)));
      Eigen::Matrix4d tform4d = cloud_tform.cast<double>();
      gtsam::Pose3 relative_pose = gtsam::Pose3::identity();  //(tform4d);
      double trans_noise = trans_noise_;
      double rot_noise = rot_noise_;
      gtsam::SharedDiagonal noise = gtsam::noiseModel::Diagonal::Sigmas(
          (gtsam::Vector(6) << rot_noise, rot_noise, rot_noise, trans_noise,
           trans_noise, trans_noise));
      omnimapper::OmniMapperBase::NonlinearFactorPtr between(
          new gtsam::BetweenFactor<gtsam::Pose3>(sym1, sym2, relative_pose,
                                                 noise));
      mapper_->addFactor(between);
    }

    return (false);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool ICPPoseMeasurementPlugin<PointT>::registerClouds(CloudConstPtr& cloud1,
                                                      CloudConstPtr& cloud2,
                                                      CloudPtr& aligned_cloud2,
                                                      Eigen::Matrix4f& tform,
                                                      double& score) {
  if (debug_) {
    printf("Starting icp... Cloud1: %zu Cloud2: %zu\n", cloud1->points.size(),
           cloud2->points.size());
    std::cout << "Cloud1 stamp: " << cloud1->header.stamp
              << " Cloud2 stamp: " << cloud2->header.stamp << std::endl;
  }

  if (cloud1->points.size() < 200 || cloud2->points.size() < 200)
    return (false);
  // pcl::IterativeClosestPointNonLinear<PointT, PointT> icp;
  // pcl::IterativeClosestPoint<PointT, PointT> icp;
  if (use_gicp_) {
    pcl::GeneralizedIterativeClosestPoint<PointT, PointT> icp;
    icp.setMaximumIterations(100);  // 20
    icp.setTransformationEpsilon(1e-6);
    icp.setMaxCorrespondenceDistance(icp_max_correspondence_distance_);  // 1.5
    icp.setInputCloud(cloud2);
    icp.setInputTarget(cloud1);
    icp.align(*aligned_cloud2, tform);
    if (debug_) printf("ICP completed...\n");
    tform = icp.getFinalTransformation();
    score = icp.getFitnessScore();
  } else {
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaximumIterations(100);  // 20
    icp.setTransformationEpsilon(1e-6);
    icp.setMaxCorrespondenceDistance(icp_max_correspondence_distance_);  // 1.5
    icp.setInputCloud(cloud2);
    icp.setInputTarget(cloud1);
    icp.align(*aligned_cloud2, tform);
    if (debug_) printf("ICP completed...\n");
    tform = icp.getFinalTransformation();
    score = icp.getFitnessScore();
  }

  if (debug_) {
    std::cout << "has converged score: " << score << std::endl;
    printf("tform:\n%lf %lf %lf %lf\n", tform(0, 0), tform(0, 1), tform(0, 2),
           tform(0, 3));
  }

  // score = icp.getFitnessScore ();

  // if (!icp.hasConverged ())
  // {
  //   printf ("ICP failed to converge, skipping this cloud!\n");
  //   return false;
  // }

  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool ICPPoseMeasurementPlugin<PointT>::tryLoopClosure(gtsam::Symbol sym) {
  // Check if we have a cloud for this
  if (clouds_.count(sym) == 0) return (false);

  // double loop_closure_dist_thresh_ = 0.10;//5.0;
  int pose_index_thresh_ = 20;

  // Get the latest solution from the mapper
  gtsam::Values solution = mapper_->getSolution();

  // Look up the current pose
  while (!solution.exists<gtsam::Pose3>(sym)) {
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    solution = mapper_->getSolution();
  }
  gtsam::Pose3 current_pose = solution.at<gtsam::Pose3>(sym);
  gtsam::Point3 current_centroid = cloud_centroids_[sym];
  gtsam::Point3 current_centroid_map =
      current_pose.transform_from(current_centroid);

  // Find the closest pose
  gtsam::Values::ConstFiltered<gtsam::Pose3> pose_filtered =
      solution.filter<gtsam::Pose3>();
  double min_dist = std::numeric_limits<double>::max();
  gtsam::Symbol closest_sym;
  BOOST_FOREACH (
      const gtsam::Values::ConstFiltered<gtsam::Pose3>::KeyValuePair& key_value,
      pose_filtered) {
    gtsam::Symbol test_sym(key_value.key);
    int sym_dist = sym.index() - test_sym.index();

    if (debug_) printf("sym: %zu test: %zu\n", sym.index(), test_sym.index());
    if (sym_dist > pose_index_thresh_) {
      if (debug_)
        printf("(%d) > %d\n", (sym.index() - test_sym.index()),
               pose_index_thresh_);
      gtsam::Pose3 test_pose(key_value.value);
      double test_dist = current_pose.range(test_pose);

      // Get centroid dist
      gtsam::Point3 test_centroid = cloud_centroids_[test_sym];
      gtsam::Point3 test_centroid_map = test_pose.transform_from(test_centroid);
      double centroid_dist =
          fabs(test_centroid_map.distance(current_centroid_map));

      // if ((test_dist < min_dist) && (clouds_.count (key_value.key) > 0))
      if ((centroid_dist < min_dist) && (clouds_.count(key_value.key) > 0)) {
        if (debug_) printf("setting min dist to %lf\n", test_dist);
        min_dist = test_dist;
        closest_sym = key_value.key;
      }
    }
  }

  // If we found something, try to add a link
  if (min_dist < loop_closure_distance_threshold_) {
    addConstraint(sym, closest_sym, score_threshold_);
    if (debug_) {
      printf(
          "ADDED LOOP CLOSURE BETWEEN %zu and "
          "%zu!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
          "!!!!!!!!!!!!!!!!!!!!!!!\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n",
          sym.index(), closest_sym.index());
    }

    return (true);
  } else {
    return (false);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool ICPPoseMeasurementPlugin<PointT>::ready() {
  boost::mutex::scoped_lock(current_cloud_mutex_);
  if (debug_) printf("ICPTest: ready: %zu\n", (!have_new_cloud_));
  return (!have_new_cloud_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
omnimapper::Time ICPPoseMeasurementPlugin<PointT>::getLastProcessedTime() {
  boost::mutex::scoped_lock(current_cloud_mutex_);
  return (last_processed_time_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void ICPPoseMeasurementPlugin<PointT>::pause(bool pause) {
  paused_ = pause;
  if (paused_) {
    // grabber_.stop ();
  } else {
    // grabber_.start ();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
typename omnimapper::ICPPoseMeasurementPlugin<PointT>::CloudConstPtr
ICPPoseMeasurementPlugin<PointT>::getCloudPtr(gtsam::Symbol sym) {
  if (debug_) printf("ICPPlugin: In getCloudPtr!\n");
  if (clouds_.count(sym) > 0)
    return (clouds_.at(sym));
  else {
    printf("ERROR: REQUESTED SYMBOL WITH NO POINTS!\n");
    CloudConstPtr empty(new Cloud());
    return (empty);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
typename omnimapper::ICPPoseMeasurementPlugin<PointT>::CloudPtr
ICPPoseMeasurementPlugin<PointT>::getFullResCloudPtr(gtsam::Symbol sym) {
  printf("ICPPlugin: In getCloudPtr!\n");
  if (full_res_clouds_.count(sym) > 0) {
    CloudPtr cloud_ptr(new Cloud());
    pcl::io::loadPCDFile<PointT>(full_res_clouds_.at(sym).c_str(), *cloud_ptr);
    return (cloud_ptr);
    // return (full_res_clouds_.at (sym));
  } else {
    printf("ERROR: REQUESTED SYMBOL WITH NO POINTS!\n");
    CloudPtr empty(new Cloud());
    return (empty);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
typename Eigen::Affine3d
ICPPoseMeasurementPlugin<PointT>::getSensorToBaseAtSymbol(gtsam::Symbol sym) {
  if (sensor_to_base_transforms_.count(sym) > 0) {
    return (sensor_to_base_transforms_.at(sym));
  } else {
    return (Eigen::Affine3d::Identity());
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void ICPPoseMeasurementPlugin<PointT>::reset() {
  initialized_ = false;
  have_new_cloud_ = false;
  first_ = true;
  previous_sym_ = gtsam::Symbol('x', 0);
  previous2_sym_ = gtsam::Symbol('x', 0);
  previous3_sym_ = gtsam::Symbol('x', 0);
  clouds_.clear();
  full_res_clouds_.clear();
  sensor_to_base_transforms_.clear();
}

}  // namespace omnimapper

// TODO: Instantiation macros.
template class omnimapper::ICPPoseMeasurementPlugin<pcl::PointXYZ>;
// template class ICPPoseMeasurementPlugin<pcl::PointXYZRGB>;
template class omnimapper::ICPPoseMeasurementPlugin<pcl::PointXYZRGBA>;
