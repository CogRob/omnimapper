#include <geometry_msgs/Point.h>
#include <gtsam/geometry/Pose3.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <omnimapper/omnimapper_base.h>
#include <omnimapper_ros/ros_time_utils.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <std_msgs/Duration.h>
#include <std_msgs/Float32.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/MarkerArray.h>

namespace omnimapper {

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

/** \brief ErrorEvaluationPlugin will take the factor graph and compute per-pose
 * error. Additionally visualizes error in RViz, so we can see where things go
 * wrong. This is useful for two purposes: uncertainty estimation and error
 * evaluation.
 */
class ErrorEvaluationPlugin : public omnimapper::OutputPlugin {
 public:
  ErrorEvaluationPlugin(omnimapper::OmniMapperBase* mapper);

  // Update callback
  void Update(boost::shared_ptr<gtsam::Values>& vis_values,
              boost::shared_ptr<gtsam::NonlinearFactorGraph>& vis_graph);

  // Loads and parses a ground truth file
  void LoadGroundTruthFile(std::string ground_truth_filename);

  // Loads and parses an associated.txt, for timestamps
  void LoadAssociatedFile(std::string associated_filename);

  // Returns the timestamp for the ith cloud from associated.txt.  This is
  // required because PCD files do not include time
  uint64_t GetStampFromIndex(int idx);

  // Returns the ground truth pose of the 0th cloud.  Used to start our map at
  // this pose, rather than the origin for vis
  gtsam::Pose3 GetInitialPose();

  // Returns the ground truth pose at a given timestamp
  gtsam::Pose3 GetPoseAtTime(omnimapper::Time time);

  // Writes mapper trajectory as a file, in the format used by the TUM RGBD
  // Benchmarking tools
  void WriteMapperTrajectoryFile(std::string trajectory_filename,
                                 gtsam::Values& current_solution);

  // Initializes the interactive markers menu
  void InitMenu();

  // Callback function for a pose that's been clicked on
  void PoseClickCallback(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  // Sets the interactive marker server (for example, to share one with the RViz
  // plugin
  void SetInteractiveMarkerServerPtr(
      boost::shared_ptr<interactive_markers::InteractiveMarkerServer>&
          marker_server) {
    marker_server_ = marker_server;
  }

  // Sets the menu handler (for example, to share one with the RViz plugin
  void SetMenuHandlerPtr(
      boost::shared_ptr<interactive_markers::MenuHandler>& menu_handler) {
    menu_handler_ = menu_handler;
  }

  // Visualize each point cloud frame
  void VisualizeEachFrame(CloudPtr cloud);

  void VisualizeStats();

  void ComputeTrajectoryStatistics(
      boost::shared_ptr<gtsam::Values>& values,
      boost::shared_ptr<gtsam::NonlinearFactorGraph>& graph);

  void Reset();

 protected:
  ros::NodeHandle nh_;

  // Interactive Markers
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer>
      marker_server_;

  // Interactive Menu Handler
  boost::shared_ptr<interactive_markers::MenuHandler> menu_handler_;

  // Playback Menu Entry Handle
  interactive_markers::MenuHandler::EntryHandle playback_menu_;

  // Trajectory Statistics Menu Entry Handle
  interactive_markers::MenuHandler::EntryHandle error_stats_menu_;

  // Per Pose Menu Handlers
  std::map<gtsam::Symbol, boost::shared_ptr<interactive_markers::MenuHandler> >
      pose_menus_;
  std::map<gtsam::Symbol, interactive_markers::MenuHandler::EntryHandle>
      pose_error_entries_;

  // Publisher for markers
  ros::Publisher marker_array_pub_;
  ros::Publisher live_frame_pub_;          // Used in evaluation mode
  ros::Publisher time_pub_;                // Publish the duration
  ros::Publisher ate_trans_per_pose_pub_;  // Per pose translation error:
                                           // sqrt(sum(t1-t*)^2)
  ros::Publisher ate_trans_rmse_pub_;      // RMSE error

  ros::Time current_time_;

  // A list of timestamps for each point cloud used in the dataset
  std::vector<omnimapper::Time> cloud_timestamps_;

  // A set of ground truth poses and timestamps
  std::vector<std::pair<omnimapper::Time, gtsam::Pose3> >
      ground_truth_trajectory_;

  OmniMapperBase* mapper_;

  // Containers for statistics
  // Holds the translational error between the the pose at Symbol and the
  // previous pose
  std::map<gtsam::Symbol, gtsam::Point3> sequential_translation_errors_;

  // Debug flag
  bool debug_;
};

}  // namespace omnimapper
