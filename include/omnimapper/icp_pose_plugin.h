#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <omnimapper/get_transform_functor.h>
#include <omnimapper/pose_plugin.h>
#include <pcl/io/pcd_grabber.h>

namespace omnimapper {
/** \brief ICPPoseMeasurementPlugin adds sequential pose constraints based on
 * scan matching to the SLAM problem.
 *
 * \author Alex Trevor
 */
template <typename PointT>
class ICPPoseMeasurementPlugin  //: public omnimapper::PosePlugin
{
  typedef typename pcl::PointCloud<PointT> Cloud;
  typedef typename Cloud::Ptr CloudPtr;
  typedef typename Cloud::ConstPtr CloudConstPtr;

  typedef typename boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose3> >
      BetweenPose3Ptr;

 public:
  ICPPoseMeasurementPlugin(omnimapper::OmniMapperBase* mapper);
  // ICPPoseMeasurementPlugin (omnimapper::OmniMapperBase* mapper, pcl::Grabber&
  // grabber);
  ~ICPPoseMeasurementPlugin();
  // bool addInitialPose ();
  void Spin();
  bool SpinOnce();
  bool RegisterClouds(CloudConstPtr& cloud1, CloudConstPtr& cloud2,
                      CloudPtr& aligned_cloud2, Eigen::Matrix4f& tform,
                      double& score);
  bool AddConstraint(gtsam::Symbol sym1, gtsam::Symbol sym2,
                     double icp_score_thresh);
  bool TryLoopClosure(gtsam::Symbol sym);
  void CloudCallback(const CloudConstPtr& cloud);
  bool Ready();
  CloudConstPtr GetCloudPtr(gtsam::Symbol sym);
  CloudPtr GetFullResCloudPtr(gtsam::Symbol sym);
  Eigen::Affine3d GetSensorToBaseAtSymbol(gtsam::Symbol sym);
  void SetMaxCorrespondenceDistance(float max_correspondence_distance) {
    icp_max_correspondence_distance_ = max_correspondence_distance;
  }
  void SetShouldDownsample(bool should_downsample) {
    downsample_ = should_downsample;
  }
  void SetLeafSize(float leaf_size) { leaf_size_ = leaf_size; }
  void SetScoreThreshold(float score_threshold) {
    score_threshold_ = score_threshold;
  }
  void SetUseGICP(bool use_gicp) { use_gicp_ = use_gicp; }
  void SetAddMultipleLinks(bool multi_link) {
    add_multiple_links_ = multi_link;
  }
  void SetAddLoopClosures(bool loop_close) { add_loop_closures_ = loop_close; }
  void SetAddIdentityOnFailure(bool add_identity_on_failure) {
    add_identity_on_failure_ = add_identity_on_failure;
  }
  void Pause(bool pause);
  void SetOverwriteTimestamps(bool overwrite_timestamps) {
    overwrite_timestamps_ = overwrite_timestamps;
  }
  void SetTransNoise(double trans_noise) { trans_noise_ = trans_noise; }
  void SetRotNoise(double rot_noise) { rot_noise_ = rot_noise; }
  void SetLoopClosureDistanceThreshold(double dist_thresh) {
    loop_closure_distance_threshold_ = dist_thresh;
  }
  void SetSaveFullResClouds(bool save_full_res_clouds) {
    save_full_res_clouds_ = save_full_res_clouds;
  }
  void SetSensorToBaseFunctor(
      omnimapper::GetTransformFunctorPtr get_transform) {
    get_sensor_to_base_ = get_transform;
  }
  omnimapper::Time GetLastProcessedTime();
  void Reset();

 protected:
  OmniMapperBase* mapper_;
  GetTransformFunctorPtr get_sensor_to_base_;
  omnimapper::Time last_processed_time_;
  bool initialized_;
  std::map<gtsam::Symbol, CloudConstPtr> clouds_;
  std::map<gtsam::Symbol, gtsam::Point3> cloud_centroids_;
  // std::map<gtsam::Symbol, CloudConstPtr> full_res_clouds_;
  std::map<gtsam::Symbol, std::string> full_res_clouds_;

  std::map<gtsam::Symbol, Eigen::Affine3d> sensor_to_base_transforms_;

  // pcl::Grabber& grabber_;
  CloudConstPtr current_cloud_;
  boost::mutex current_cloud_mutex_;
  bool have_new_cloud_;
  bool first_;
  bool downsample_;
  float leaf_size_;
  float score_threshold_;
  double trans_noise_;
  double rot_noise_;
  bool debug_;
  bool overwrite_timestamps_;
  gtsam::Symbol previous_sym_;
  gtsam::Symbol previous2_sym_;
  gtsam::Symbol previous3_sym_;
  float icp_max_correspondence_distance_;
  bool use_gicp_;
  bool add_identity_on_failure_;
  bool add_multiple_links_;
  bool add_loop_closures_;
  float loop_closure_distance_threshold_;
  bool paused_;
  bool save_full_res_clouds_;
};
}  // namespace omnimapper
