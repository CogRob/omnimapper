#include <omnimapper/ThreadPool.h>
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
      debug_(false),
      // TODO(shengye): overwrite_timestamps_ was false. Confirm we need true?
      overwrite_timestamps_(false),
      previous_sym_(gtsam::Symbol('x', 0)),
      previous2_sym_(gtsam::Symbol('x', 0)),
      previous3_sym_(gtsam::Symbol('x', 0)),
      icp_max_correspondence_distance_(3.5),
      use_gicp_(true),
      add_identity_on_failure_(false),
      add_multiple_links_(false),
      add_loop_closures_(false),
      loop_closure_distance_threshold_(0.1),
      paused_(false),
      save_full_res_clouds_(false),
      min_cloud_size_(100),
      thread_pool_(4) {
  have_new_cloud_ = false;
  first_ = true;
}

template <typename PointT>
ICPPoseMeasurementPlugin<PointT>::~ICPPoseMeasurementPlugin() {}

template <typename PointT>
void ICPPoseMeasurementPlugin<PointT>::CloudCallback(
    const CloudConstPtr& cloud) {
  boost::lock_guard<boost::mutex> lock(current_cloud_mutex_);
  LOG_IF(INFO, debug_) << "Cloud callback.";
  // Store this as the previous cloud.
  current_cloud_ = cloud;
  if (have_new_cloud_) {
    LOG(ERROR) << "Got new cloud before done processing the old one!";
    // TODO(shengye): Die here?
  }
  have_new_cloud_ = true;
  LOG_IF(INFO, debug_) << "Stored new cloud in CloudCallback.";
}

template <typename PointT>
void ICPPoseMeasurementPlugin<PointT>::Spin() {
  while (true) {
    SpinOnce();
    boost::this_thread::sleep(boost::posix_time::milliseconds(10));
  }
}

template <typename PointT>
bool ICPPoseMeasurementPlugin<PointT>::SpinOnce() {
  const double spin_start = pcl::getTime();

  // Get a shared_ptr to the latest cloud.
  CloudConstPtr current_cloud;
  {
    boost::lock_guard<boost::mutex> lock(current_cloud_mutex_);
    // Do nothing if there isn't a new cloud
    if (!have_new_cloud_) {
      return false;
    }
    // TODO(shengye): The following code was disabled. Reenable for now, but is
    // this necessay?
    if (current_cloud_->points.size() < min_cloud_size_) {
      printf("ICPPoseMeasurementPlugin: Not enough points!\n");
      return false;
    }
    // TODO(shengye): pcl::isFinite check on current_cloud_->points was
    // disabled. Do we need it? Should we enable it?
    current_cloud = current_cloud_;
  }
  const CloudConstPtr current_cloud_original = current_cloud;

  LOG(INFO) << "ICPPoseMeasurementPlugin got a new cloud.";
  LOG_IF(INFO, debug_) << "Current cloud has " << current_cloud->points.size()
                       << "points";

  // Downsample, if needed.
  if (downsample_) {
    pcl::VoxelGrid<PointT> grid;
    grid.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    grid.setInputCloud(current_cloud);
    CloudPtr current_cloud_filtered(new Cloud());
    grid.filter(*current_cloud_filtered);
    current_cloud = std::move(current_cloud_filtered);
  }

  // Get the previous pose and cloud
  gtsam::Symbol current_sym;
  boost::posix_time::ptime current_time;
  if (overwrite_timestamps_) {
    current_time = boost::posix_time::ptime(
        boost::posix_time::microsec_clock::local_time());
  } else {
    current_time = omnimapper::StampToPtime(current_cloud->header.stamp);
  }

  // Apply sensor to base transform, if we have one.
  CloudConstPtr current_cloud_base;
  if (get_sensor_to_base_) {
    if (debug_) printf("ICPPosePlugin: Applying sensor to base transform\n");
    LOG_IF(INFO, debug_) << "Applying sensor to base transform, "
                         << "timestamp: " << current_time;
    Eigen::Affine3d sensor_to_base = (*get_sensor_to_base_)(current_time);
    CloudPtr current_cloud_transformed(new Cloud());
    pcl::transformPointCloud(*current_cloud, *current_cloud_transformed,
                             sensor_to_base);
    current_cloud_base = std::move(current_cloud_transformed);
  } else {
    current_cloud_base = current_cloud;
    LOG(ERROR) << "No sensor to base transform exists. Ignoring.";
  }

  LOG_IF(INFO, debug_) << "Getting symbol for current time: " << current_time;
  mapper_->GetPoseSymbolAtTime(current_time, &current_sym);
  LOG_IF(INFO, debug_) << "Current symbol: " << std::string(current_sym);

  // Compute and save the cloud centroid, for use in loop closure detection.
  Eigen::Vector4f cloud_centroid;
  pcl::compute3DCentroid(*current_cloud_base, cloud_centroid);
  gtsam::Point3 centroid_pt(cloud_centroid[0], cloud_centroid[1],
                            cloud_centroid[2]);
  {
    boost::lock_guard<boost::mutex> lock(clouds_mutex_);
    clouds_.insert(std::make_pair(
        current_sym, static_cast<CloudConstPtr>(current_cloud_base)));
    cloud_centroids_.insert(std::make_pair(current_sym, centroid_pt));
  }

  if (save_full_res_clouds_) {
    const std::string out_file = "/tmp/" + std::string(current_sym) + ".pcd";
    Eigen::Affine3d sensor_to_base = (*get_sensor_to_base_)(current_time);
    full_res_clouds_.insert(std::make_pair(current_sym, out_file));
    sensor_to_base_transforms_.insert(
        std::make_pair(current_sym, sensor_to_base));
    LOG_IF(INFO, debug_) << "ICPPlugin saved full res cloud with "
                         << current_cloud_original->points.size() << " points.";
    pcl::io::savePCDFileBinaryCompressed(out_file, *current_cloud_original);
    LOG_IF(INFO, debug_) << "ICPPlugin saved cloud to " << out_file;
  }

  if (first_) {
    boost::lock_guard<boost::mutex> lock(current_cloud_mutex_);
    have_new_cloud_ = false;
    previous_sym_ = current_sym;
    first_ = false;
    LOG_IF(INFO, debug_) << "Done with first, will return.";
    return false;
  }

  const gtsam::Symbol previous_sym = previous_sym_;
  const gtsam::Symbol previous2_sym = previous2_sym_;
  const gtsam::Symbol previous3_sym = previous3_sym_;
  LOG_IF(INFO, debug_) << "ICP Symbols: "
                       << "prev3: " << std::string(previous3_sym) << ", "
                       << "prev2: " << std::string(previous2_sym) << ", "
                       << "prev: " << std::string(previous_sym) << ", "
                       << "curr: " << std::string(current_sym);

  thread_pool_.enqueue([this, previous_sym, current_sym] {
    LOG(INFO) << "AddConstraint between previous and current symbol.";
    this->AddConstraint(previous_sym, current_sym, score_threshold_);
  });

  std::size_t clouds_size;
  {
    boost::lock_guard<boost::mutex> lock(clouds_mutex_);
    clouds_size = clouds_.size();
  }
  // Try previous too.
  // TODO(shengye): We changed from previous_sym to current_sym. Check if this
  // is correct?
  if (add_multiple_links_) {
    if (clouds_size >= 3) {
      thread_pool_.enqueue([this, previous2_sym, current_sym] {
        LOG(INFO) << "AddConstraint between current and previous2 symbol.";
        // TODO(shengye): Chaned from "true" to score_threshold_.
        // FIXME(shengye): This was between previous2_sym and previous_sym
        AddConstraint(previous2_sym, current_sym, score_threshold_);
        LOG_IF(INFO, debug_) << "AddConstraint for prev2 complete.";
      });
    }
    if (clouds_size >= 4) {
      thread_pool_.enqueue([this, previous3_sym, current_sym] {
        LOG(INFO) << "AddConstraint between current and previous3 symbol.";
        // FIXME(shengye): This was between previous3_sym and previous_sym
        AddConstraint(previous3_sym, current_sym, score_threshold_);
        LOG_IF(INFO, debug_) << "AddConstraint for prev3 complete.";
      });
    }
  }

  if (add_loop_closures_) {
    if (clouds_size > 20) {
      thread_pool_.enqueue([this, previous3_sym] {
        // TODO(shengye): I assume because we have did more for previous3_sym,
        // now it is safe to try loop closure? Confirm this?
        LOG(INFO) << "Try loop closure.";
        TryLoopClosure(previous3_sym);
      });
    }
  }

  // Wait for the threads to complete, at least.
  thread_pool_.wait_until_nothing_in_flight();

  // Note that we're done
  {
    boost::lock_guard<boost::mutex> lock(current_cloud_mutex_);
    LOG_IF(INFO, debug_) << "Done with cloud.";
    previous3_sym_ = previous2_sym_;
    previous2_sym_ = previous_sym_;
    previous_sym_ = current_sym;
    last_processed_time_ = current_time;
    have_new_cloud_ = false;
  }

  const double spin_end = pcl::getTime();
  LOG(INFO) << "ICP Plugin added a pose, took " << double(spin_end - spin_start)
            << " seconds";
  return true;
}

template <typename PointT>
bool ICPPoseMeasurementPlugin<PointT>::AddConstraint(
    gtsam::Symbol sym1, gtsam::Symbol sym2, double icp_score_threshold) {
  // Look up clouds
  CloudConstPtr cloud1, cloud2;
  {
    boost::lock_guard<boost::mutex> lock(clouds_mutex_);
    cloud1 = clouds_.at(sym1);
    cloud2 = clouds_.at(sym2);
  }
  if (!(cloud1 && cloud2)) {
    LOG(INFO) << "Don't have clouds for these poses: " << std::string(sym1)
              << " and " << std::string(sym2);
  }

  // Look up initial guess, if applicable
  boost::optional<gtsam::Pose3> cloud1_pose = mapper_->PredictPose(sym1);
  boost::optional<gtsam::Pose3> cloud2_pose = mapper_->PredictPose(sym2);

  Eigen::Matrix4f cloud_tform = Eigen::Matrix4f::Identity();
  if (cloud1_pose && cloud2_pose) {
    // If we have an initial guess.
    gtsam::Pose3 initial_guess = cloud1_pose->between(*cloud2_pose);
    if (debug_) {
      cloud1_pose->print("Pose1:\n");
      LOG(INFO) << "Cloud 1 pose det: "
                << cloud1_pose->rotation().matrix().determinant();
      cloud2_pose->print("Pose2:\n");
      LOG(INFO) << "Cloud 2 pose det: "
                << cloud2_pose->rotation().matrix().determinant();
      initial_guess.print("Initial guess:");
      LOG(INFO) << "Initial guess det: "
                << initial_guess.rotation().matrix().determinant();
    }
    cloud_tform = initial_guess.matrix().cast<float>();
  }

  CloudPtr aligned_cloud(new Cloud());

  double icp_score = 0.0;
  bool icp_converged =
      RegisterClouds(cloud1, cloud2, &aligned_cloud, &cloud_tform, &icp_score);

  if ((icp_converged && icp_score < icp_score_threshold) ||
      add_identity_on_failure_) {
    Eigen::Matrix4d tform4d;
    if (!(icp_converged && icp_score < icp_score_threshold)) {
      CHECK(add_identity_on_failure_);
      cloud_tform = Eigen::Matrix4f::Identity();
      LOG(ERROR) << "Add a factor even if ICP did not converge!";
    }
    tform4d = cloud_tform.cast<double>();
    gtsam::Pose3 relative_pose(tform4d);

    // TODO: make these params.
    const double trans_noise = trans_noise_;  // * icp_score;
    const double rot_noise = rot_noise_;      // * icp_score;
    gtsam::Vector noise_vector(6);
    noise_vector << rot_noise, rot_noise, rot_noise, trans_noise, trans_noise,
        trans_noise;
    gtsam::SharedDiagonal noise =
        gtsam::noiseModel::Diagonal::Sigmas(noise_vector);
    omnimapper::OmniMapperBase::NonlinearFactorPtr between(
        new gtsam::BetweenFactor<gtsam::Pose3>(sym1, sym2, relative_pose,
                                               noise));
    mapper_->AddFactor(between);
    LOG(INFO) << "Added factor beteen " << std::string(sym1) << " and "
              << std::string(sym2);
    relative_pose.print("ICP Relative Pose:\n");
    LOG(INFO) << "ICP score: " << icp_score << ", relative pose det: "
              << relative_pose.rotation().matrix().determinant();
    if (icp_converged && icp_score < icp_score_threshold) {
      // This is actual success, we will return false if the factor was added
      // because of add_identity_on_failure_.
      return true;
    }
  }
  return false;
}

template <typename PointT>
bool ICPPoseMeasurementPlugin<PointT>::RegisterClouds(
    const CloudConstPtr& cloud1, const CloudConstPtr& cloud2,
    CloudPtr* aligned_cloud2, Eigen::Matrix4f* tform, double* score) {
  LOG_IF(INFO, debug_) << "Starting ICP for cloud1 with "
                       << cloud1->points.size() << " points, and cloud 2 with "
                       << cloud2->points.size() << " points.";
  LOG_IF(INFO, debug_) << "Timestamp of cloud 1: " << cloud1->header.stamp
                       << ", cloud 2: " << cloud2->header.stamp;
  if (cloud1->points.size() < 200 || cloud2->points.size() < 200) {
    LOG(ERROR) << "No enough points, ignore RegisterClouds";
    return false;
  }
  if (use_gicp_) {
    pcl::GeneralizedIterativeClosestPoint<PointT, PointT> icp;
    icp.setMaximumIterations(100);  // 20
    icp.setTransformationEpsilon(1e-6);
    icp.setMaxCorrespondenceDistance(icp_max_correspondence_distance_);  // 1.5
    icp.setInputCloud(cloud2);
    icp.setInputTarget(cloud1);
    icp.align(**aligned_cloud2, *tform);
    *tform = icp.getFinalTransformation();
    *score = icp.getFitnessScore();
    LOG_IF(INFO, debug_) << "ICP (GICP) completed with score " << *score;
  } else {
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaximumIterations(100);  // 20
    icp.setTransformationEpsilon(1e-6);
    icp.setMaxCorrespondenceDistance(icp_max_correspondence_distance_);  // 1.5
    icp.setInputCloud(cloud2);
    icp.setInputTarget(cloud1);
    icp.align(**aligned_cloud2, *tform);
    *tform = icp.getFinalTransformation();
    *score = icp.getFitnessScore();
    LOG_IF(INFO, debug_) << "ICP completed with score " << *score;
  }

  LOG_IF(INFO, debug_) << "RegisterClouds converged with score " << *score;
  LOG_IF(INFO, debug_) << "tform: (" << (*tform)(0, 0) << ", " << (*tform)(0, 1)
                       << ", " << (*tform)(0, 2) << ", " << (*tform)(0, 3)
                       << ")";
  return true;
}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool ICPPoseMeasurementPlugin<PointT>::TryLoopClosure(gtsam::Symbol sym) {
  {
    // Check if we have a cloud for this
    boost::lock_guard<boost::mutex> lock(clouds_mutex_);
    if (clouds_.count(sym) == 0) return (false);
  }

  const int pose_index_thresh = 20;

  // Get the latest solution from the mapper.
  const gtsam::Values solution = mapper_->GetSolution();

  // Look up the current pose.
  while (!solution.exists<gtsam::Pose3>(sym)) {
    LOG(INFO) << "Looking for the pose of " << std::string(sym);
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    solution = mapper_->GetSolution();
  }
  const gtsam::Pose3 current_pose = solution.at<gtsam::Pose3>(sym);
  gtsam::Point3 current_centroid;
  {
    boost::lock_guard<boost::mutex> lock(clouds_mutex_);
    current_centroid = cloud_centroids_[sym];
  }
  const gtsam::Point3 current_centroid_map =
      current_pose.transform_from(current_centroid);

  // Find the closest pose.
  gtsam::Values::ConstFiltered<gtsam::Pose3> pose_filtered =
      solution.filter<gtsam::Pose3>();
  double min_dist = std::numeric_limits<double>::max();
  gtsam::Symbol closest_sym;
  for (const auto& key_value : pose_filtered) {
    const gtsam::Symbol test_sym(key_value.key);
    const int sym_dist = std::abs(static_cast<int>(sym.index()) -
                                  static_cast<int>(test_sym.index()));
    LOG_IF(INFO, debug_) << "sym: " << std::string(sym)
                         << ", test_sym: " << std::string(test_sym);
    if (sym_dist > pose_index_thresh) {
      LOG_IF(INFO, debug_) << "sym_dist " << sym_dist << " > threshold "
                           << pose_index_thresh;
      const gtsam::Pose3 test_pose(key_value.value);
      const double test_dist = current_pose.range(test_pose);

      // Get centroid dist
      gtsam::Point3 test_centroid;
      bool test_in_clouds;
      {
        boost::lock_guard<boost::mutex> lock(clouds_mutex_);
        test_centroid = cloud_centroids_[test_sym];
        test_in_clouds = clouds_.count(test_sym);
      }
      gtsam::Point3 test_centroid_map = test_pose.transform_from(test_centroid);
      const double centroid_dist =
          fabs(test_centroid_map.distance(current_centroid_map));
      if ((centroid_dist < min_dist) && test_in_clouds) {
        LOG_IF(INFO, debug_) << "Setting min dist to " << test_dist
                             << ", symbol to " << std::string(test_sym);
        min_dist = test_dist;
        closest_sym = test_sym;
      }
    }
  }

  // If we found something, try to add a link
  if (min_dist < loop_closure_distance_threshold_) {
    AddConstraint(sym, closest_sym, score_threshold_);
    LOG_IF(INFO, debug_) << "ICP add loop closure between " << std::string(sym)
                         << " and " << std::string(closest_sym);
    return true;
  } else {
    return false;
  }
}

template <typename PointT>
bool ICPPoseMeasurementPlugin<PointT>::Ready() {
  boost::lock_guard<boost::mutex> lock(current_cloud_mutex_);
  // FIXME(shengye): Why "!"?
  LOG_IF(INFO, debug_) << "ICP ready: " << !have_new_cloud_;
  return (!have_new_cloud_);
}

template <typename PointT>
omnimapper::Time ICPPoseMeasurementPlugin<PointT>::GetLastProcessedTime() {
  boost::lock_guard<boost::mutex> lock(current_cloud_mutex_);
  return (last_processed_time_);
}

template <typename PointT>
void ICPPoseMeasurementPlugin<PointT>::Pause(bool pause) {
  paused_ = pause;
  // TODO(shengye): If we have a grabber_, we should start/stop it.
}

template <typename PointT>
typename omnimapper::ICPPoseMeasurementPlugin<PointT>::CloudConstPtr
ICPPoseMeasurementPlugin<PointT>::GetCloudPtr(gtsam::Symbol sym) {
  LOG_IF(INFO, debug_) << "GetCloudPtr called.";
  boost::lock_guard<boost::mutex> lock(clouds_mutex_);
  if (clouds_.count(sym) > 0)
    return (clouds_.at(sym));
  else {
    LOG(ERROR) << "Requested symbol with no points.";
    CloudPtr empty(new Cloud());
    return (empty);
  }
}

template <typename PointT>
typename omnimapper::ICPPoseMeasurementPlugin<PointT>::CloudPtr
ICPPoseMeasurementPlugin<PointT>::GetFullResCloudPtr(gtsam::Symbol sym) {
  LOG(INFO) << "GetFullResCloudPtr";
  if (full_res_clouds_.count(sym) > 0) {
    CloudPtr cloud_ptr(new Cloud());
    pcl::io::loadPCDFile<PointT>(full_res_clouds_.at(sym).c_str(), *cloud_ptr);
    return cloud_ptr;
  } else {
    LOG(ERROR) << "Requested symbol with no points.";
    CloudPtr empty(new Cloud());
    return (empty);
  }
}

template <typename PointT>
typename Eigen::Affine3d
ICPPoseMeasurementPlugin<PointT>::GetSensorToBaseAtSymbol(gtsam::Symbol sym) {
  if (sensor_to_base_transforms_.count(sym) > 0) {
    return (sensor_to_base_transforms_.at(sym));
  } else {
    return (Eigen::Affine3d::Identity());
  }
}

template <typename PointT>
void ICPPoseMeasurementPlugin<PointT>::Reset() {
  boost::lock_guard<boost::mutex> lock_current_cloud(current_cloud_mutex_);
  boost::lock_guard<boost::mutex> lock_clouds(clouds_mutex_);
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
template class omnimapper::ICPPoseMeasurementPlugin<pcl::PointXYZRGBA>;
