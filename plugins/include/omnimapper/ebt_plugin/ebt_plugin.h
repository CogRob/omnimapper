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

#include "omnimapper/ebt_plugin/ebt_base.h"

#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose3.h>

//#include <boost/functional/hash.hpp>


namespace omnimapper
{
/* \brief EBTPoseMeasurementPlugin adds sequential pose constraints based on edge tracking to the SLAM problem.
    author Ruffin White
*/

class EBTPoseMeasurementPlugin //: public omnimapper::PosePlugin
{
    typedef typename boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose3> > BetweenPose3Ptr;

public:

    typedef EBTBase::EBTDetection EBTDetection;
    typedef EBTDetection::Ptr EBTDetectionPtr;
    typedef EBTDetection::ConstPtr EBTDetectionConstPtr;

    typedef EBTBase::EBTMessage EBTMessage;
    typedef EBTMessage::Ptr EBTMessagePtr;
    typedef EBTMessage::ConstPtr EBTMessageConstPtr;

    /** \brief EBTPoseMeasurementPlugin constructor. */
    EBTPoseMeasurementPlugin (omnimapper::OmniMapperBase* mapper);
    ~EBTPoseMeasurementPlugin ();
    
    /** \brief spin starts a thread spinning, which will process messages as they become available. */
    void spin ();
    
    /** \brief spinOnce handles one update cycle, if a new message is available.  Not needed if spin() has been called. */
    bool spinOnce ();

    /** \brief Attempts EBT and adds a constraint between sym1 and sym2. */
    bool addConstraint (gtsam::Symbol sym1, gtsam::Symbol sym2, gtsam::Pose3 relative_pose);
    
    /** \brief ebtmessageCallback is used to provide input to the EBT Plugin. */
    void ebtmessageCallback (const EBTMessageConstPtr& message);
    
    /** \brief ready returns false if the plugin is currently processing a message, false otherwise. */
    bool ready ();
    
    /** \brief getEBTMessagePtr returns a boost::shared_ptr to the point message at symbol sym. */
    EBTMessageConstPtr getEBTMessagePtr (gtsam::Symbol sym);
    
    /** \brief getSensorToBaseAtSymbol returns the cached sensor to base transform for symbol sym. */
    Eigen::Affine3d getSensorToBaseAtSymbol (gtsam::Symbol sym);



    /** \brief setTransNoise sets the translational noise to use on constraints. */
    void setTransNoise (double trans_noise) { trans_noise_ = trans_noise; }

    /** \brief setRotNoise sets the rotational noise to use on constraints. */
    void setRotNoise (double rot_noise) { rot_noise_ = rot_noise; }

    /** \brief setInitDistanceThreshold sets the Init Distance Threshold. */
    void setInitDistanceThreshold (double init_distance_threshold) { init_distance_threshold_ = init_distance_threshold; }

    /** \brief setInitDistanceThreshold sets the Init Distance Threshold. */
    void setInitAngleThreshold (double init_angle_threshold) { init_angle_threshold_ = init_angle_threshold; }

    /** \brief setInitDistanceThreshold sets the Init Distance Threshold. */
    void setInitWithEstimate (bool init_with_estimate) { init_with_estimate_ = init_with_estimate; }

    void setInitCovar(double rot_noise_, double trans_noise_) { initial_covar_obj_ <<pow((1/rot_noise_),2),pow((1/rot_noise_),2),pow((1/rot_noise_),2),pow((1/trans_noise_),2),pow((1/trans_noise_),2),pow((1/trans_noise_),2);}


    
    /** \brief setSensorToBaseFuctor sets the functor that will give the relative pose of the sensor frame in the base frame.  (Can be dynamic, e.g. PTU) */
    void setSensorToBaseFunctor (omnimapper::GetTransformFunctorPtr get_transform) { get_sensor_to_base_ = get_transform; }
    
    /** \brief setTriggerFunctor sets a trigger functor to use. */
    void setTriggerFunctor(omnimapper::TriggerFunctorPtr ptr) { trigger_ = ptr; }
    
    /** \brief returns the time the previous frame processing was completed. */
    omnimapper::Time getLastProcessedTime ();


    /* \brief Installs a callback for the ebtmessage init */
    void setEBTMessageInitCallback (boost::function<void (const EBTMessageConstPtr&)>& ebtmessage_init_callback){
        ebtmessage_init_callback_ = ebtmessage_init_callback;
    }
    
    /** \brief resets the plugin to the initial state. */
    void reset ();
    
protected:
    OmniMapperBase* mapper_;

    GetTransformFunctorPtr get_sensor_to_base_;
    omnimapper::Time last_processed_time_;
    
    TriggerFunctorPtr trigger_;
    Time triggered_time_;
    
    bool initialized_;
    std::map<gtsam::Symbol, EBTMessageConstPtr> messages_;
    std::map<gtsam::Symbol, std::map<boost::posix_time::ptime, EBTDetectionConstPtr> > detections_;
    
    std::map<gtsam::Symbol, Eigen::Affine3d> sensor_to_base_transforms_;

    EBTMessageConstPtr current_message_;
    boost::mutex current_message_mutex_;
    boost::condition_variable current_message_cv_;
    bool have_new_message_;

    bool ready_;
    bool first_;
    double trans_noise_;
    double rot_noise_;
    double init_distance_threshold_;
    double init_angle_threshold_;
    bool init_with_estimate_;
    bool debug_;
    bool overwrite_timestamps_;
    gtsam::Vector6 initial_covar_obj_;



    boost::function<void (const EBTMessageConstPtr&)> ebtmessage_init_callback_;

    boost::hash<std::string> string_hash;
};

}
