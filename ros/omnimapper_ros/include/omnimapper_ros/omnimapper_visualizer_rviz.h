/*
 * Software License Agreement (BSD License)
 *
 *  OmniMapper
 *  Copyright (c) 2012-, Georgia Tech Research Corporation,
 *  Atlanta, Georgia 30332-0415
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <omnimapper/omnimapper_base.h>
#include <omnimapper/icp_plugin.h>
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <omnimapper_ros/VisualizeFullCloud.h>
#include <omnimapper_ros/PublishModel.h>
#include <omnimapper_ros/WriteTrajectoryFile.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <pcl/segmentation/planar_region.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>


namespace omnimapper
{
    /** \brief OmniMapperVisualizerRViz is an output plugin for OmniMapper based on the RViz.  
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
      void spinOnce ();
      void spin ();
      void planarRegionCallback (std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions, omnimapper::Time t);
      void drawBBox (pcl::PointCloud<pcl::PointXYZRGB>& cloud, ros::Publisher& marker_pub_, int obj_idx);
      void labelCloudCallback (const CloudConstPtr& cloud, const LabelCloudConstPtr& labels);
      void clusterCloudCallback (std::vector<CloudPtr> clusters, omnimapper::Time t,  boost::optional<std::vector<pcl::PointIndices> > );
      void setICPPlugin (boost::shared_ptr<omnimapper::ICPPoseMeasurementPlugin<PointT> >& icp_plugin) { icp_plugin_ = icp_plugin; }
      bool drawICPCloudsCallback (omnimapper_ros::VisualizeFullCloud::Request &req, omnimapper_ros::VisualizeFullCloud::Response &res);
      bool publishModel (omnimapper_ros::PublishModel::Request &req, omnimapper_ros::PublishModel::Response &res);
      void setDrawPoseArray (bool draw_pose_array) { draw_pose_array_ = draw_pose_array; }
      void setDrawPoseGraph (bool draw_pose_graph) { draw_pose_graph_ = draw_pose_graph; }
      void setOutputGraphviz (bool output_graphviz) { output_graphviz_ = output_graphviz; }
      void setDrawICPCloudsAlways (bool draw_always) { draw_icp_clouds_always_ = draw_always; if (draw_always) draw_icp_clouds_ = true; }
      void setDrawICPCloudsFullRes (bool full_res) { draw_icp_clouds_full_res_ = full_res; }

      boost::shared_ptr<interactive_markers::InteractiveMarkerServer> getInteractiveMarkerServerPtr () { return (marker_server_); }

      boost::shared_ptr<interactive_markers::MenuHandler> getMenuHandlerPtr () { return (menu_handler_); }

      /** \brief Initializes the Interactive Menu */
      void initMenu ();

      /** \brief playPauseCb is used by interactive marker menu to control the mapper's playback */
      void playPauseCb (const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

      void drawMapCloudCb (const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

      void drawPoseMarginalsCb (const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

      // For drawing planes, and use in AR application
      //void planarRegionCallback (std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions, omnimapper::Time t);

      bool writeTrajectoryFile(omnimapper_ros::WriteTrajectoryFile::Request &req, omnimapper_ros::WriteTrajectoryFile::Response &res);

    protected:
      // A ROS Node Handle
      ros::NodeHandle nh_;
      
      // A reference to a mapper instance
      OmniMapperBase* mapper_;

      // Latest Map Information
      boost::mutex state_mutex_;
      boost::shared_ptr<gtsam::Values> vis_values_;
      boost::shared_ptr<gtsam::NonlinearFactorGraph> vis_graph_;
      bool updated_;

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

      ros::ServiceServer write_trajectory_srv_;

      ros::ServiceServer draw_icp_clouds_srv_;

      ros::ServiceServer draw_object_observation_cloud_srv_;

      ros::ServiceServer publish_model_srv_;

      // ICP Plugin Ref
      boost::shared_ptr<omnimapper::ICPPoseMeasurementPlugin<PointT> > icp_plugin_;


      std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > latest_planes_;

      bool draw_icp_clouds_;

      bool draw_icp_clouds_always_;

      double draw_icp_clouds_interval_;

      double draw_icp_clouds_prev_time_;

      bool draw_icp_clouds_full_res_;

      bool draw_icp_clouds_downsampled_;
      
      bool draw_planar_landmarks_;

      bool draw_pose_array_;

      bool draw_pose_graph_;

      bool draw_object_observation_cloud_;

      bool draw_object_observation_bboxes_;

      bool draw_pose_marginals_;

      bool output_graphviz_;

      bool passthrough_filter_map_cloud_;

      bool write_trajectory_text_file_;
  };
}
