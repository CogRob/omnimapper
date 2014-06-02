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

#pragma once

#include <omnimapper/omnimapper_base.h>
#include <omnimapper_ros/canonical_scan_matcher_plugin.h>
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <omnimapper_ros/VisualizeFullCloud.h>

namespace omnimapper
{
  template <typename LScanT>
  class CSMVisualizerRViz : public omnimapper::OutputPlugin
  {
    public:
      CSMVisualizerRViz (omnimapper::OmniMapperBase* mapper);
      void update (boost::shared_ptr<gtsam::Values>& vis_values, boost::shared_ptr<gtsam::NonlinearFactorGraph>& vis_graph);
      void setCSMPlugin (boost::shared_ptr<omnimapper::CanonicalScanMatcherPlugin<LScanT> >& csm_plugin) { csm_plugin_ = csm_plugin; }
      bool drawCSMMap (omnimapper_ros::VisualizeFullCloud::Request &req, omnimapper_ros::VisualizeFullCloud::Response &res);
      
    protected:
      ros::NodeHandle nh_;
      
      OmniMapperBase* mapper_;
      
      ros::Publisher pose_array_pub_;

      ros::Publisher marker_array_pub_;
      
      ros::Publisher map_cloud_pub_;

      ros::ServiceServer draw_csm_map_srv_;
      
      boost::shared_ptr<omnimapper::CanonicalScanMatcherPlugin<LScanT> > csm_plugin_;

      boost::shared_ptr<gtsam::Values> vis_values_;

      boost::shared_ptr<gtsam::NonlinearFactorGraph> vis_graph_;

      boost::mutex vis_mutex_;

      bool draw_graph_;

      bool draw_map_;

      
  };
  
}
