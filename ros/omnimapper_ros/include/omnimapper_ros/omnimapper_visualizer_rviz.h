#include <omnimapper/omnimapper_base.h>
#include <omnimapper/icp_pose_plugin.h>
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <omnimapper_ros/VisualizeFullCloud.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

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

    public:
      OmniMapperVisualizerRViz (omnimapper::OmniMapperBase* mapper);
      void update (boost::shared_ptr<gtsam::Values>& vis_values, boost::shared_ptr<gtsam::NonlinearFactorGraph>& vis_graph);
      void setICPPlugin (boost::shared_ptr<omnimapper::ICPPoseMeasurementPlugin<PointT> >& icp_plugin) { icp_plugin_ = icp_plugin; }
      bool drawICPCloudsCallback (omnimapper_ros::VisualizeFullCloud::Request &req, omnimapper_ros::VisualizeFullCloud::Response &res);

    protected:
      // A ROS Node Handle
      ros::NodeHandle nh_;
      
      // A reference to a mapper instance
      OmniMapperBase* mapper_;
      
      // Publisher for the trajectory
      ros::Publisher pose_array_pub_;
      
      // Publisher for the map clouds
      ros::Publisher map_cloud_pub_;
      
      // Publisher for Planar boundaries
      ros::Publisher planar_boundary_pub_;

      // Publisher for visualization marker arrays
      ros::Publisher marker_array_pub_;

      ros::ServiceServer draw_icp_clouds_srv_;

      // ICP Plugin Ref
      boost::shared_ptr<omnimapper::ICPPoseMeasurementPlugin<PointT> > icp_plugin_;

      bool draw_icp_clouds_;
      
      bool draw_planar_landmarks_;

  };
}
