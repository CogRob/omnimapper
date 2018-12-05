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

#include <omnimapper/enp_plugin/enp_base.h>

#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose3.h>

#include <opencv2/core/core.hpp>


namespace omnimapper
{
/* \brief ENPPoseMeasurementPlugin adds sequential pose constraints based on edge tracking to the SLAM problem.
    author Ruffin White
*/

class ENPPoseMeasurementPlugin //: public omnimapper::PosePlugin
{
    typedef typename boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose3> > BetweenPose3Ptr;

public:

    typedef ENPBase::ENPImage ENPImage;
    typedef ENPImage::Ptr ENPImagePtr;
    typedef ENPImage::ConstPtr ENPImageConstPtr;

    /** \brief ENPPoseMeasurementPlugin constructor. */
    ENPPoseMeasurementPlugin (omnimapper::OmniMapperBase* mapper);
    ~ENPPoseMeasurementPlugin ();
    
    /** \brief spin starts a thread spinning, which will process images as they become available. */
    void spin ();
    
    /** \brief spinOnce handles one update cycle, if a new image is available.  Not needed if spin() has been called. */
    bool spinOnce ();

    /** \brief Attempts ENP and adds a constraint between sym1 and sym2. */
    bool addConstraint (gtsam::Symbol sym1, gtsam::Symbol sym2, double enp_score_thresh);
    
    /** \brief enpimageCallback is used to provide input to the ENP Plugin. */
    void enpimageCallback (const ENPImageConstPtr& image);
    
    /** \brief ready returns false if the plugin is currently processing a image, false otherwise. */
    bool ready ();
    
    /** \brief getENPImagePtr returns a boost::shared_ptr to the point image at symbol sym. */
    ENPImageConstPtr getENPImagePtr (gtsam::Symbol sym);
    
    /** \brief getSensorToBaseAtSymbol returns the cached sensor to base transform for symbol sym. */
    Eigen::Affine3d getSensorToBaseAtSymbol (gtsam::Symbol sym);



    /** \brief setTransNoise sets the translational noise to use on constraints. */
    void setTransNoise (double trans_noise) { trans_noise_ = trans_noise; }
    
    /** \brief setRotNoise sets the rotational noise to use on constraints. */
    void setRotNoise (double rot_noise) { rot_noise_ = rot_noise; }
    

    
    /** \brief setSensorToBaseFuctor sets the functor that will give the relative pose of the sensor frame in the base frame.  (Can be dynamic, e.g. PTU) */
    void setSensorToBaseFunctor (omnimapper::GetTransformFunctorPtr get_transform) { get_sensor_to_base_ = get_transform; }
    
    /** \brief setTriggerFunctor sets a trigger functor to use. */
    void setTriggerFunctor(omnimapper::TriggerFunctorPtr ptr) { trigger_ = ptr; }
    
    /** \brief returns the time the previous frame processing was completed. */
    omnimapper::Time getLastProcessedTime ();
    
    /** \brief resets the plugin to the initial state. */
    void reset ();

    /** \brief transformPointENPImage transform the image given a pose3. */
    void transformPointENPImage (const ENPImagePtr &image, const gtsam::Pose3 &T);

    /** \brief applyBaseTransform transform the image using the base. */
    void applyBaseTransform (ENPImagePtr& image);
    
protected:
    OmniMapperBase* mapper_;

    GetTransformFunctorPtr get_sensor_to_base_;
    omnimapper::Time last_processed_time_;
    
    TriggerFunctorPtr trigger_;
    Time triggered_time_;
    
    bool initialized_;
    std::map<gtsam::Symbol, ENPImageConstPtr> images_;
    
    std::map<gtsam::Symbol, Eigen::Affine3d> sensor_to_base_transforms_;

    ENPImageConstPtr current_image_;
    boost::mutex current_image_mutex_;
    boost::condition_variable current_image_cv_;
    bool have_new_image_;

    bool ready_;
    bool first_;
    double trans_noise_;
    double rot_noise_;
    bool debug_;
    bool overwrite_timestamps_;
};

}
