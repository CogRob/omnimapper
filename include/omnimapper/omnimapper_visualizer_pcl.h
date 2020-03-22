
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

#include <omnimapper/icp_pose_plugin.h>
#include <omnimapper/omnimapper_base.h>
#include <omnimapper/plane.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace omnimapper {

/** \brief OmniMapperVisualizerPCL is an output plugin for OmniMapper based on
 * the PCLVisualizer.  Currently this only supports visualization of a
 * trajectory, and clouds from an ICP plugin.
 *
 * \author Alex Trevor
 */
template <typename PointT>
class OmniMapperVisualizerPCL : public omnimapper::OutputPlugin {
  // typedef typename pcl::PointXYZ PointT;
  typedef typename pcl::PointCloud<PointT> Cloud;
  typedef typename Cloud::Ptr CloudPtr;
  typedef typename Cloud::ConstPtr CloudConstPtr;

 public:
  OmniMapperVisualizerPCL(omnimapper::OmniMapperBase* mapper);
  void update(boost::shared_ptr<gtsam::Values>& vis_values,
              boost::shared_ptr<gtsam::NonlinearFactorGraph>& vis_graph);
  void spin();
  void spinThread();
  void spinOnce();
  void setICPPlugin(
      boost::shared_ptr<omnimapper::ICPPoseMeasurementPlugin<PointT> >&
          icp_plugin) {
    icp_plugin_ = icp_plugin;
  }
  void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void*);

  // void spinAndUpdate ();
 protected:
  // A PCL Visualizer
  pcl::visualization::PCLVisualizer viewer_;
  // Visualizer mutex
  boost::mutex vis_mutex_;
  // A reference to a mapper instance
  OmniMapperBase* mapper_;

  // Pose Cloud
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > pose_cloud_;
  // Updated flag
  bool new_slam_data_;
  // Flag for drawing ICP clouds
  bool draw_icp_clouds_;
  // Flag for drawing planar boundaries
  bool draw_planar_boundaries_;
  // Flag for drawing planar normals
  bool draw_planar_normals_;
  // ICP Plugin Ref
  boost::shared_ptr<omnimapper::ICPPoseMeasurementPlugin<PointT> > icp_plugin_;

  // Debug flag
  bool debug_;
};
}  // namespace omnimapper
