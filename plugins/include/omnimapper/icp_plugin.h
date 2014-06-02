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

#include <omnimapper/pose_plugin.h>
#include <omnimapper/get_transform_functor.h>
#include <omnimapper/trigger.h>
#include <pcl/io/pcd_grabber.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose3.h>

namespace omnimapper
{
  /** \brief ICPPoseMeasurementPlugin adds sequential pose constraints based on scan matching to the SLAM problem.
   *
   * \author Alex Trevor
   */
  template <typename PointT>
  class ICPPoseMeasurementPlugin //: public omnimapper::PosePlugin
  {
    typedef typename pcl::PointCloud<PointT> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;
    typedef typename boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose3> > BetweenPose3Ptr;

    public:

      /** \brief ICPPoseMeasurementPlugin constructor. */
      ICPPoseMeasurementPlugin (omnimapper::OmniMapperBase* mapper);
      ~ICPPoseMeasurementPlugin ();

      /** \brief Spin starts a thread spinning, which will process clouds as they become available. */
      void spin ();

      /** \brief spinOnce handles one update cycle, if a new cloud is available.  Not needed if spin() has been called. */
      bool spinOnce ();

      /** \brief Performs registration of two clouds. */
      bool registerClouds (CloudConstPtr& cloud1, CloudConstPtr& cloud2, CloudPtr& aligned_cloud2, Eigen::Matrix4f& tform, double& score);

      /** \brief Attempts ICP and adds a constraint between sym1 and sym2. */
      bool addConstraint (gtsam::Symbol sym1, gtsam::Symbol sym2, double icp_score_thresh);

      /** \brief Attempts to find a loop closure at requested symbol, by examining all past poses. */
      bool tryLoopClosure (gtsam::Symbol sym);

      /** \brief cloudCallback is used to provide input to the ICP Plugin. */
      void cloudCallback (const CloudConstPtr& cloud);

      /** \brief ready returns false if the plugin is currently processing a cloud, false otherwise. */
      bool ready ();

      /** \brief getCloudPtr returns a boost::shared_ptr to the point cloud at symbol sym. */
      CloudConstPtr getCloudPtr (gtsam::Symbol sym);

      /** \brief getFullRestCloudPtr returns a boost::shared_ptr to the full resolution point cloud at symbol sym. */
      CloudPtr getFullResCloudPtr (gtsam::Symbol sym);

      /** \brief getSensorToBaseAtSymbol returns the cached sensor to base transform for symbol sym. */
      Eigen::Affine3d getSensorToBaseAtSymbol (gtsam::Symbol sym);

      /** \brief setMaxCorrespondenceDistance sets the maximum correspondence distance for ICP -- see PCL's ICP documentation for details. */
      void setMaxCorrespondenceDistance (float max_correspondence_distance) { icp_max_correspondence_distance_ = max_correspondence_distance; }

      /** \brief setShouldDownsample enables or disables downsampling of input point clouds. */
      void setShouldDownsample (bool should_downsample) { downsample_ = should_downsample; }

      /** \brief setLeafSize sets the voxel grid leaf size for downsampling (when enabled.) */
      void setLeafSize (float leaf_size) { leaf_size_ = leaf_size; }

      /** \brief setScoreThreshold sets the ICP score threshold for sequential pose matches. */
      void setScoreThreshold (float score_threshold) { score_threshold_ = score_threshold; }

      /** \brief setUseGICP enables the Generalized ICP Algorithm (PCL implementation), false uses the default PCL implementaiton. */
      void setUseGICP (bool use_gicp) { use_gicp_ = use_gicp; }

      /** \brief setAddMultipleLinks will additionally add links for (x, x-2) and (x, x-3). */
      void setAddMultipleLinks (bool multi_link) { add_multiple_links_ = multi_link; }

      /** \brief setAddLoopClosures enables or disables loop closure constraints -- only sequential ICP factors are added if false. */
      void setAddLoopClosures (bool loop_close) { add_loop_closures_ = loop_close; }

      /** \brief setAddIdentityOnFailure allows an identity pose to be added if ICP fails. */
      void setAddIdentityOnFailure (bool add_identity_on_failure) { add_identity_on_failure_ = add_identity_on_failure; }

      /** \brief setOverwriteTimestamps enables the current system clock to be used, instead of cloud timestamps. */
      void setOverwriteTimestamps (bool overwrite_timestamps) { overwrite_timestamps_ = overwrite_timestamps; }

      /** \brief setTransNoise sets the translational noise to use on constraints. */
      void setTransNoise (double trans_noise) { trans_noise_ = trans_noise; }

      /** \brief setRotNoise sets the rotational noise to use on constraints. */
      void setRotNoise (double rot_noise) { rot_noise_ = rot_noise; }

      /** \brief setLoopClosureDistanceThreshold sets a threshold for loop closures.  Loop closures are considered if the cloud centroid is within thresh of the requested symbol. */
      void setLoopClosureDistanceThreshold (double dist_thresh) { loop_closure_distance_threshold_ = dist_thresh; }

      /** \brief setSaveFullResClouds allows full resolution clouds to be saved (by writing them to /tmp) */
      void setSaveFullResClouds (bool save_full_res_clouds) { save_full_res_clouds_ = save_full_res_clouds; }

      /** \brief setSensorToBaseFuctor sets the functor that will give the relative pose of the sensor frame in the base frame.  (Can be dynamic, e.g. PTU) */
      void setSensorToBaseFunctor (omnimapper::GetTransformFunctorPtr get_transform) { get_sensor_to_base_ = get_transform; }

      /** \brief setTriggerFunctor sets a trigger functor to use. */
      void setTriggerFunctor(omnimapper::TriggerFunctorPtr ptr) { trigger_ = ptr; }

      /** \brief returns the time the previous frame processing was completed. */
      omnimapper::Time getLastProcessedTime ();

      /** \brief resets the plugin to the initial state. */
      void reset ();

    protected:
      OmniMapperBase* mapper_;
      GetTransformFunctorPtr get_sensor_to_base_;
      omnimapper::Time last_processed_time_;

      TriggerFunctorPtr trigger_;
      Time triggered_time_;

      bool initialized_;
      std::map<gtsam::Symbol, CloudConstPtr> clouds_;

      /** \brief Cache the cloud centroids, used to determine potential loop closures. */
      std::map<gtsam::Symbol, gtsam::Point3> cloud_centroids_;

      std::map<gtsam::Symbol, std::string> full_res_clouds_;

      std::map<gtsam::Symbol, Eigen::Affine3d> sensor_to_base_transforms_;

      CloudConstPtr current_cloud_;
      boost::mutex current_cloud_mutex_;
      boost::condition_variable current_cloud_cv_;
      bool have_new_cloud_;
      bool ready_;
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
      float loop_closure_score_threshold_;
      int loop_closure_pose_index_threshold_;
      bool save_full_res_clouds_;
  };
}
