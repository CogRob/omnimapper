#include <gperftools/profiler.h>
#include <omnimapper/3rdparty/distortion_model_standalone.h>
#include <omnimapper/bounded_plane_plugin.h>
#include <omnimapper/icp_pose_plugin.h>
#include <omnimapper/no_motion_pose_plugin.h>
#include <omnimapper/object_plugin.h>
#include <omnimapper/omnimapper_base.h>
#include <omnimapper/organized_feature_extraction.h>
#include <omnimapper/plane_plugin.h>
#include <omnimapper/time.h>
#include <omnimapper/tsdf_output_plugin.h>
#include <omnimapper_ros/OutputMapTSDF.h>
#include <omnimapper_ros/ar_marker_plugin.h>
#include <omnimapper_ros/canonical_scan_matcher_plugin.h>
#include <omnimapper_ros/csm_visualizer.h>
#include <omnimapper_ros/error_evaluation_plugin.h>
#include <omnimapper_ros/get_transform_functor_tf.h>
#include <omnimapper_ros/omnimapper_visualizer_rviz.h>
#include <omnimapper_ros/ros_tf_utils.h>
#include <omnimapper_ros/tf_pose_plugin.h>
#include <omnimapper_ros/tum_data_error_plugin.h>
#include <pcl/common/time.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <boost/filesystem.hpp>

template <typename PointT>
class OmniMapperROS {
  typedef pcl::PointCloud<PointT> Cloud;
  typedef typename Cloud::Ptr CloudPtr;
  typedef typename Cloud::ConstPtr CloudConstPtr;
  typedef typename pcl::PointCloud<pcl::Label> LabelCloud;
  typedef typename LabelCloud::Ptr LabelCloudPtr;
  typedef typename LabelCloud::ConstPtr LabelCloudConstPtr;

 public:
  // Constructor
  OmniMapperROS(ros::NodeHandle nh);

  // Load (or reload) ROS Parameters
  void loadROSParams();

  // Point Cloud Callback
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

  // Laser Scan Callback
  void laserScanCallback(const sensor_msgs::LaserScanConstPtr& msg);

  // Evaluation Timer Callback
  void evalTimerCallback(const ros::TimerEvent& e);

  // Map To Odometry Correction Publication
  void publishMapToOdom();

  // Publish the current pose in the map frame
  void publishCurrentPose();

  // Service call for generating a map TSDF
  bool generateMapTSDFCallback(omnimapper_ros::OutputMapTSDF::Request& req,
                               omnimapper_ros::OutputMapTSDF::Response& res);

  void runEvaluation(std::string& associated_filename,
                     std::string& groundtruth_filename, std::string& pcd_path,
                     std::string& output_trajectory_filename,
                     std::string& output_timing_filename);

  void resetEvaluation();

 protected:
  // ROS Node Handle
  ros::NodeHandle n_;

  // OmniMapper Instance
  omnimapper::OmniMapperBase omb_;

  // TF Pose Plugin
  omnimapper::TFPosePlugin tf_plugin_;

  // No Motion Pose Plugin
  omnimapper::NoMotionPosePlugin no_motion_plugin_;

  // ICP Plugin
  omnimapper::ICPPoseMeasurementPlugin<PointT> icp_plugin_;

  // Edge ICP Plugin
  omnimapper::ICPPoseMeasurementPlugin<PointT> edge_icp_plugin_;

  // Plane Plugin
  omnimapper::PlaneMeasurementPlugin<PointT> plane_plugin_;

  // Bounded Plane Plugin
  omnimapper::BoundedPlanePlugin<PointT> bounded_plane_plugin_;

  // Object Plugin
  omnimapper::ObjectPlugin<PointT> object_plugin_;

  // CSM Plugin
  omnimapper::CanonicalScanMatcherPlugin<sensor_msgs::LaserScan> csm_plugin_;

  // AR Marker Plugin
  omnimapper::ARMarkerPlugin ar_marker_plugin_;

  // Visualization
  omnimapper::OmniMapperVisualizerRViz<PointT> vis_plugin_;

  // CSM Visualization
  omnimapper::CSMVisualizerRViz<sensor_msgs::LaserScan> csm_vis_plugin_;

  // TSDF Plugin
  omnimapper::TSDFOutputPlugin<PointT> tsdf_plugin_;

  // Benchmark Data Analysis
  omnimapper::TUMDataErrorPlugin bag_error_plugin_;

  omnimapper::ErrorEvaluationPlugin eval_plugin_;

  // Distortion Model
  DistortionModelStandalone distortion_model_;

  // Fake Grabber (TODO: Fix this)
  std::vector<std::string> fake_files_;
  pcl::PCDGrabber<PointT> fake_grabber_;
  // Organized Feature Extraction
  omnimapper::OrganizedFeatureExtraction<PointT> organized_feature_extraction_;

  // TF Listener  (for initialization)
  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;

  // Subscribers
  ros::Subscriber pointcloud_sub_;
  ros::Subscriber laserScan_sub_;

  ros::ServiceServer generate_tsdf_srv_;

  // Mapper config
  bool use_planes_;
  bool use_bounded_planes_;
  bool use_objects_;
  bool use_csm_;
  bool use_icp_;
  bool use_occ_edge_icp_;
  bool use_tf_;
  bool use_no_motion_;

  // ROS Params
  std::string odom_frame_name_;
  std::string base_frame_name_;
  std::string rgbd_frame_name_;
  std::string cloud_topic_name_;

  bool init_pose_from_tf_;
  bool use_init_pose_;
  bool suppress_commit_window_;
  double init_x_, init_y_, init_z_;
  double init_qx_, init_qy_, init_qz_, init_qw_;

  // ICP Params
  double icp_leaf_size_;
  double icp_max_correspondence_distance_;
  double icp_score_thresh_;
  double icp_trans_noise_;
  double icp_rot_noise_;
  bool icp_add_identity_on_fail_;
  bool icp_add_loop_closures_;
  double icp_loop_closure_distance_threshold_;

  // Occluding Edge ICP Params
  double occ_edge_trans_noise_;
  double occ_edge_rot_noise_;
  double occ_edge_score_thresh_;
  double occ_edge_max_correspondence_dist_;
  bool occ_edge_add_identity_on_fail_;

  // Plane Plugin Params
  double plane_range_threshold_;
  double plane_angular_threshold_;
  double plane_range_noise_;
  double plane_angular_noise_;

  // Object Plugin Params
  std::string object_database_location_;
  bool object_loop_closures_;
  bool object_landmarks_;
  bool save_object_models_;
  double object_min_height_;

  // Labelled Cloud Plugin Params
  bool use_label_cloud_;

  // TF Plugin Params
  double tf_trans_noise_;
  double tf_rot_noise_;
  double tf_roll_noise_, tf_pitch_noise_, tf_yaw_noise_;

  // Visualization params
  bool use_rviz_plugin_;
  bool draw_pose_array_;
  bool draw_pose_graph_;
  bool draw_label_cloud_;
  bool draw_clusters_;
  bool draw_icp_clouds_always_;

  // TSDF plugin params
  bool use_tsdf_plugin_;

  // Error plugin params
  bool use_error_plugin_;
  bool use_error_eval_plugin_;

  // Other Flags
  bool add_pose_per_cloud_;
  bool broadcast_map_to_odom_;
  bool broadcast_current_pose_;
  bool use_distortion_model_;
  bool use_rgbd_sensor_base_tf_functor_;
  std::string distortion_model_path_;
  bool debug_;
  bool ar_mode_;

  // Evaluation Mode
  bool evaluation_mode_;
  std::string evaluation_pcd_path_;
  std::vector<std::string> evaluation_pcd_files_;
  int evaluation_file_idx_;
  // Note: we require the associated_txt path because the PCD format does not
  // include timestamps. We instead load the timestamps from this file.
  std::string evaluation_associated_txt_path_;
  std::string evaluation_ground_truth_txt_path_;
  std::string evaluation_output_trajectory_txt_path_;
  bool evaluation_mode_write_trajectory_;
  bool evaluation_mode_write_tsdf_;
  ros::Timer eval_timer_;
  bool evaluation_mode_paused_;
  bool use_organized_feature_extraction_;
  bool evaluation_show_frames_;
};
