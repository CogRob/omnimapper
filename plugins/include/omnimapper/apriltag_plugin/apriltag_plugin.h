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

#include "omnimapper/apriltag_plugin/apriltag_base.h"

#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose3.h>

namespace omnimapper
{
/* \brief AprilTagPoseMeasurementPlugin adds sequential pose constraints based on apriltag markers.
    author Ruffin White
*/

class AprilTagPoseMeasurementPlugin //: public omnimapper::PosePlugin
{
    typedef typename boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose3> > BetweenPose3Ptr;

public:

    typedef APRILTagBase::APRILTagDetection APRILTagDetection;
    typedef APRILTagDetection::Ptr APRILTagDetectionPtr;
    typedef APRILTagDetection::ConstPtr APRILTagDetectionConstPtr;

    typedef APRILTagBase::APRILTagMessage APRILTagMessage;
    typedef APRILTagMessage::Ptr APRILTagMessagePtr;
    typedef APRILTagMessage::ConstPtr APRILTagMessageConstPtr;

    /** \brief AprilTagPoseMeasurementPlugin constructor. */
    AprilTagPoseMeasurementPlugin (omnimapper::OmniMapperBase* mapper);
    ~AprilTagPoseMeasurementPlugin ();
    
    /** \brief spin starts a thread spinning, which will process messages as they become available. */
    void spin ();
    
    /** \brief spinOnce handles one update cycle, if a new message is available.  Not needed if spin() has been called. */
    bool spinOnce ();
    
    /** \brief Attempts EBT and adds a constraint between sym1 and sym2. */
    bool addConstraint (gtsam::Symbol sym1, gtsam::Symbol sym2, gtsam::Pose3 relative_pose);
    
    /** \brief Attempts to find a loop closure at requested symbol, by examining all past poses. */
    bool tryLoopClosure (gtsam::Symbol sym);
    
    /** \brief apriltagCallback is used to provide input to the EBT Plugin. */
    void apriltagCallback (const APRILTagMessageConstPtr& message);
    
    /** \brief ready returns false if the plugin is currently processing a message, false otherwise. */
    bool ready ();
    
    /** \brief getAPRILTagMessagePtr returns a boost::shared_ptr to the point message at symbol sym. */
    APRILTagMessageConstPtr getAPRILTagMessagePtr (gtsam::Symbol sym);
    
    /** \brief getSensorToBaseAtSymbol returns the cached sensor to base transform for symbol sym. */
    Eigen::Affine3d getSensorToBaseAtSymbol (gtsam::Symbol sym);




    /** \brief setUsePose sets to use the Pose3 message for the factor. */
    void setUsePose (bool use_pose) { use_pose_ = use_pose; }

    /** \brief setUseProjection sets to use the Projection message for the factor. */
    void setUseProjection (bool use_projection) { use_projection_ = use_projection; }

    /** \brief setTransNoise sets the translational noise to use on constraints. */
    void setTransNoise (double trans_noise) { trans_noise_ = trans_noise; }
    
    /** \brief setRotNoise sets the rotational noise to use on constraints. */
    void setRotNoise (double rot_noise) { rot_noise_ = rot_noise; }

    /** \brief setUniqueIds sets if to expect if IDs are unique. */
    void setUniqueIds (bool unique_ids) { unique_ids_ = unique_ids; }

    /** \brief setUseProjection sets to use the Projection message for the factor. */
    void setLoopClosureDistanceThreshold (double loop_closure_distance_threshold) { loop_closure_distance_threshold_ = loop_closure_distance_threshold; }


    
    /** \brief setSensorToBaseFuctor sets the functor that will give the relative pose of the sensor frame in the base frame.  (Can be dynamic, e.g. PTU) */
    void setSensorToBaseFunctor (omnimapper::GetTransformFunctorPtr get_transform) { get_sensor_to_base_ = get_transform; }
    
    /** \brief setTriggerFunctor sets a trigger functor to use. */
    void setTriggerFunctor(omnimapper::TriggerFunctorPtr ptr) { trigger_ = ptr; }
    
    /** \brief returns the time the previous frame processing was completed. */
    omnimapper::Time getLastProcessedTime ();

    
    /** \brief resets the plugin to the initial state. */
    void reset ();

    /** \brief transformPointAPRILTag transform the message given a pose3. */
    void transformPointAPRILTag (const APRILTagMessagePtr &message, const gtsam::Pose3 &T);

    /** \brief applyBaseTransform transform the message using the base. */
    void applyBaseTransform (APRILTagMessagePtr& message);
    
protected:
    OmniMapperBase* mapper_;

    GetTransformFunctorPtr get_sensor_to_base_;
    omnimapper::Time last_processed_time_;
    
    TriggerFunctorPtr trigger_;
    Time triggered_time_;
    
    bool initialized_;
    std::map<gtsam::Symbol, APRILTagMessageConstPtr> messages_;
    std::map<gtsam::Symbol, std::map<boost::posix_time::ptime, APRILTagDetectionConstPtr> > detections_;
    
    std::map<gtsam::Symbol, Eigen::Affine3d> sensor_to_base_transforms_;

    APRILTagMessageConstPtr current_message_;
    boost::mutex current_message_mutex_;
    boost::condition_variable current_message_cv_;
    bool have_new_message_;

    bool ready_;
    bool first_;

    bool use_pose_;
    bool use_projection_;
    double trans_noise_;
    double rot_noise_;
    bool unique_ids_;
    double loop_closure_distance_threshold_;

    bool debug_;
    bool overwrite_timestamps_;

    boost::hash<std::string> string_hash;

};

}
