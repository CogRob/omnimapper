#include <omnimapper/omnimapper_base.h>
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/MarkerArray.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <geometry_msgs/Point.h>
#include <omnimapper_ros/ros_time_utils.h>

#include <gtsam/geometry/Pose3.h>


namespace omnimapper
{
  /** \brief ErrorEvaluationPlugin will take the factor graph and compute per-pose error.  
   *   Additionally visualizes error in RViz, so we can see where things go wrong. 
   *   This is useful for two purposes: uncertainty estimation and error evaluation.
   */
  class ErrorEvaluationPlugin : public omnimapper::OutputPlugin
  {
    public:
      ErrorEvaluationPlugin (omnimapper::OmniMapperBase* mapper);
      void update (boost::shared_ptr<gtsam::Values>& vis_values, boost::shared_ptr<gtsam::NonlinearFactorGraph>& vis_graph);

      // Loads and parses a ground truth file
      void loadGroundTruthFile (std::string ground_truth_filename);

      // Loads and parses an associated.txt, for timestamps
      void loadAssociatedFile (std::string associated_filename);

      // Returns the timestamp for the ith cloud from associated.txt.  This is required because PCD files do not include time
      uint64_t getStampFromIndex (int idx);

      // Returns the ground truth pose of the 0th cloud.  Used to start our map at this pose, rather than the origin for vis
      gtsam::Pose3 getInitialPose ();

      // Returns the ground truth pose at a given timestamp
      gtsam::Pose3 getPoseAtTime (omnimapper::Time time);

      // Writes mapper trajectory as a file, in the format used by the TUM RGBD Benchmarking tools
      void writeMapperTrajectoryFile (std::string trajectory_filename, gtsam::Values& current_solution);

      // Initializes the interactive markers menu
      void initMenu ();

      // Callback function for a pose that's been clicked on
      void poseClickCallback (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

      // Sets the interactive marker server (for example, to share one with the RViz plugin
      void setInteractiveMarkerServerPtr (boost::shared_ptr<interactive_markers::InteractiveMarkerServer>& marker_server) { marker_server_ = marker_server; }
      
      // Sets the menu handler (for example, to share one with the RViz plugin
      void setMenuHandlerPtr (boost::shared_ptr<interactive_markers::MenuHandler>& menu_handler) { menu_handler_ = menu_handler; }

    protected:
      ros::NodeHandle nh_;

      // Interactive Markers
      boost::shared_ptr<interactive_markers::InteractiveMarkerServer> marker_server_;

      // Interactive Menu Handler
      boost::shared_ptr<interactive_markers::MenuHandler> menu_handler_;

      // Playback Menu Entry Handle
      interactive_markers::MenuHandler::EntryHandle playback_menu_;

      // Publisher for markers
      ros::Publisher marker_array_pub_;

      // A list of timestamps for each point cloud used in the dataset
      std::vector<omnimapper::Time> cloud_timestamps_;

      // A set of ground truth poses and timestamps
      std::vector<std::pair<omnimapper::Time, gtsam::Pose3> > ground_truth_trajectory_;
      
      OmniMapperBase* mapper_;
  };
  
  
}

