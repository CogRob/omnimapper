#pragma once

#include <omnimapper/omnimapper_base.h>
#include <omnimapper_ros/canonical_scan_matcher_plugin.h>
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

namespace omnimapper
{
  template <typename LScanT>
  class CSMVisualizerRViz : public omnimapper::OutputPlugin
  {
    public:
      CSMVisualizerRViz (omnimapper::OmniMapperBase* mapper);
      void update (boost::shared_ptr<gtsam::Values>& vis_values, boost::shared_ptr<gtsam::NonlinearFactorGraph>& vis_graph);
      void setCSMPlugin (boost::shared_ptr<omnimapper::CanonicalScanMatcherPlugin<LScanT> >& csm_plugin) { csm_plugin_ = csm_plugin; }
      //bool drawICPCloudsCallback (omnimapper_ros::VisualizeFullCloud::Request &req, omnimapper_ros::VisualizeFullCloud::Response &res);
      
    protected:
      ros::NodeHandle nh_;
      
      OmniMapperBase* mapper_;
      
      ros::Publisher pose_array_pub_;

      ros::Publisher marker_array_pub_;
      
      ros::Publisher map_cloud_pub_;

      ros::ServiceServer draw_icp_clouds_srv_;
      
      boost::shared_ptr<omnimapper::CanonicalScanMatcherPlugin<LScanT> > csm_plugin_;

      bool draw_graph_;

      bool draw_map_;

      
  };
  
}
