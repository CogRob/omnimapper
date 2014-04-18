#include <omnimapper/omnimapper_base.h>
#include <omnimapper/icp_pose_plugin.h>
#include <omnimapper/object_plugin.h>
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <omnimapper_ros/VisualizeFullCloud.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <pcl/segmentation/planar_region.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>


namespace omnimapper
{
    /** \brief OmniMapperVisualizerRViz is an output plugin for OmniMapper based on the RViz.  Currently
   *  this only supports visualization of a trajectory, and clouds from an ICP plugin.
   *
   * \author Alex Trevor
   */
  template <typename PointT>
  class OmniMapperVisualizerRViz : public omnimapper::OutputPlugin
  {
    typedef typename pcl::PointCloud<PointT> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;
    typedef pcl::PointCloud<pcl::Label> LabelCloud;
    typedef typename LabelCloud::Ptr LabelCloudPtr;
    typedef typename LabelCloud::ConstPtr LabelCloudConstPtr;

    public:
      OmniMapperVisualizerRViz (omnimapper::OmniMapperBase* mapper);
      void update (boost::shared_ptr<gtsam::Values>& vis_values, boost::shared_ptr<gtsam::NonlinearFactorGraph>& vis_graph);
      void planarRegionCallback (std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions, omnimapper::Time t);
      void drawBBox (pcl::PointCloud<pcl::PointXYZRGB>& cloud, ros::Publisher& marker_pub_, int obj_idx);
      void labelCloudCallback (const CloudConstPtr& cloud, const LabelCloudConstPtr& labels);
      void clusterCloudCallback (std::vector<CloudPtr> clusters, omnimapper::Time t,  boost::optional<std::vector<pcl::PointIndices> > );
      void setICPPlugin (boost::shared_ptr<omnimapper::ICPPoseMeasurementPlugin<PointT> >& icp_plugin) { icp_plugin_ = icp_plugin; }
      void setObjectPlugin (boost::shared_ptr<omnimapper::ObjectPlugin<PointT> >& object_plugin) { object_plugin_ = object_plugin; }
      bool drawObjectObservationCloud (omnimapper_ros::VisualizeFullCloud::Request &req, omnimapper_ros::VisualizeFullCloud::Response &res);
      bool drawICPCloudsCallback (omnimapper_ros::VisualizeFullCloud::Request &req, omnimapper_ros::VisualizeFullCloud::Response &res);
      void setDrawPoseArray (bool draw_pose_array) { draw_pose_array_ = draw_pose_array; }
      void setDrawPoseGraph (bool draw_pose_graph) { draw_pose_graph_ = draw_pose_graph; }
      void setOutputGraphviz (bool output_graphviz) { output_graphviz_ = output_graphviz; }
      void setDrawICPCloudsAlways (bool draw_always) { draw_icp_clouds_always_ = draw_always; if (draw_always) draw_icp_clouds_ = true; }

      boost::shared_ptr<interactive_markers::InteractiveMarkerServer> getInteractiveMarkerServerPtr () { return (marker_server_); }

      boost::shared_ptr<interactive_markers::MenuHandler> getMenuHandlerPtr () { return (menu_handler_); }

      /** \brief Initializes the Interactive Menu */
      void initMenu ();

      /** \brief playPauseCb is used by interactive marker menu to control the mapper's playback */
      void playPauseCb (const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

      void drawMapCloudCb (const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

      void drawPoseMarginalsCb (const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

      /** \brief objectCallback draws the estimated objects computed by object_plugin */
      void objectCallback(std::map<gtsam::Symbol, Object<PointT> > object_map, gtsam::Point3 direction, gtsam::Point3 center);
    protected:
      // A ROS Node Handle
      ros::NodeHandle nh_;
      
      // A reference to a mapper instance
      OmniMapperBase* mapper_;

      // Interactive Markers
      boost::shared_ptr<interactive_markers::InteractiveMarkerServer> marker_server_;

      // Menu Handler
      boost::shared_ptr<interactive_markers::MenuHandler> menu_handler_;

      // Playback Menu Entry Handle
      interactive_markers::MenuHandler::EntryHandle playback_menu_;

      // Visualizaion Menu Handle
      interactive_markers::MenuHandler::EntryHandle visualization_menu_;

      // Publisher for the trajectory
      ros::Publisher pose_array_pub_;
      
      // Publisher for the map clouds
      ros::Publisher map_cloud_pub_;
      
      // Publisher for Planar boundaries
      ros::Publisher planar_boundary_pub_;

      // Publisher for visualization marker arrays
      ros::Publisher marker_array_pub_;

      ros::Publisher pose_covariances_pub_;

      // Publishers for segmentation results of planes
      ros::Publisher segmented_plane_pub_;
      ros::Publisher segmented_label_cloud_pub_;
      ros::Publisher segmented_clusters_pub_;
      
      // Publisher for object observations
      ros::Publisher object_observation_pub_;
      ros::Publisher object_modeled_pub_;


      ros::ServiceServer draw_icp_clouds_srv_;

      ros::ServiceServer draw_object_observation_cloud_srv_;

      // ICP Plugin Ref
      boost::shared_ptr<omnimapper::ICPPoseMeasurementPlugin<PointT> > icp_plugin_;

      // Object Plugin Ref
      boost::shared_ptr<omnimapper::ObjectPlugin<PointT> > object_plugin_;

      bool draw_icp_clouds_;

      bool draw_icp_clouds_always_;
      
      bool draw_planar_landmarks_;

      bool draw_pose_array_;

      bool draw_pose_graph_;

      bool draw_object_observation_cloud_;

      bool draw_object_observation_bboxes_;

      bool draw_pose_marginals_;

      bool output_graphviz_;

      bool passthrough_filter_map_cloud_;
  };
}
