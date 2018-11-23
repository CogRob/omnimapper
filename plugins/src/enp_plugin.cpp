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

#include <omnimapper/enp_plugin/enp_plugin.h>
#include <omnimapper/omnimapper_base.h>
#include <omnimapper/time.h>
#include <pcl/common/time.h>

namespace omnimapper{
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ENPPoseMeasurementPlugin::ENPPoseMeasurementPlugin (omnimapper::OmniMapperBase* mapper) :
    mapper_ (mapper),
    get_sensor_to_base_ (GetTransformFunctorPtr ()),
    last_processed_time_ (),
    trigger_ (new omnimapper::TriggerAlways ()),
    triggered_time_ (omnimapper::epoch_time ()),
    initialized_ (false),
    have_new_image_ (false),
    ready_ (true),
    first_ (true),
    trans_noise_ (.05),
    rot_noise_ (.05),
    debug_ (false),
    overwrite_timestamps_ (false)
  {
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  ENPPoseMeasurementPlugin::~ENPPoseMeasurementPlugin ()
  {
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void
  ENPPoseMeasurementPlugin::enpimageCallback (const ENPImageConstPtr &image)
  {
    if (debug_)
      printf ("enpimage callback\n");

    Time measurement_time = omnimapper::stamp2ptime (image->header.stamp);
    bool use_measurement = (*trigger_)(measurement_time);
    if (!use_measurement)
      return;

    // Store this as the previous image
    boost::mutex::scoped_lock lock (current_image_mutex_);
    current_image_ = image;
    have_new_image_ = true;
    current_image_cv_.notify_one ();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void ENPPoseMeasurementPlugin::spin ()
  {
    while (true)
      {
        spinOnce ();
        boost::this_thread::sleep (boost::posix_time::milliseconds (10));
      }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  bool
  ENPPoseMeasurementPlugin::spinOnce ()
  {
    if(!initialized_)
      {
        initialized_ = true;
      }

    ENPImagePtr current_image (new ENPImage ());
    {
      boost::mutex::scoped_lock lock (current_image_mutex_);

      while (!have_new_image_)
        {
          current_image_cv_.wait (lock);
        }
      *current_image = *current_image_;
      have_new_image_ = false;
    }

    if (debug_)
      {
        printf ("current ENPImage size: %d x %d\n", current_image->img.rows, current_image->img.cols);
      }

    // Downsample, if needed
    ENPImagePtr current_image_filtered (new ENPImage ());
    *current_image_filtered = *current_image;


    // Do Edge Based Tracking with
    //current_image_filtered
//    applyBaseTransform(current_image_filtered);


    // Get the previous pose and image
    gtsam::Symbol current_sym;
    boost::posix_time::ptime current_time;
    if (overwrite_timestamps_)
      {
        current_time = boost::posix_time::ptime ( boost::posix_time::microsec_clock::local_time() );
      }
    else
      {
        current_time = omnimapper::stamp2ptime (current_image->header.stamp);
      }

    if (debug_)
      std::cout << "ENP Plugin: Getting symbol for current time: " << current_time << std::endl;
    mapper_->getPoseSymbolAtTime (current_time, current_sym);

    if (debug_)
      printf ("ENP Plugin: current symbol: %zu, inserting image\n", current_sym.index ());
    //{
    //  boost::mutex::scoped_lock (current_image_processed_mutex_);
    ENPImagePtr image_processed_base (new ENPImage ());

//    image_processed_base->header = current_image_processed_base->header;
//    images_.insert (std::pair<gtsam::Symbol, ENPImageConstPtr> (current_sym, image_processed_base));//current_image_processed_));

//    // We're done if that was the first image
//    //printf ("first: %d\n", first_);
//    if (first_)
//      {
//        //FIXME: add how?
//        boost::optional<gtsam::Pose3> current_pose = mapper_->predictPose (current_sym);

//        gtsam::Pose3 initial_guess = current_pose->compose(image_processed_base->pose3); //current_pose->between (image_processed_base->pose3);
//        mapper_->addNewValue(object_sym_,initial_guess);


//        //boost::mutex::scoped_lock (current_image_processed_mutex_);
//        if (debug_)
//          printf ("ENPTest: done with first, returning\n");
//        previous_sym_ = current_sym;
//        first_ = false;
//        ready_ = true;
//        return (false);
//      }

//    // Add constraints
//    boost::thread latest_enp_thread (&ENPPoseMeasurementPlugin::addConstraint, this, current_sym, object_sym_, score_threshold_);

//    // Wait for latest one to complete, at least
//    latest_enp_thread.join ();

    // Note that we're done
    {
      //boost::mutex::scoped_lock lock (current_image_processed_mutex_);
      if (debug_)
        {
          printf ("ENPTest: done with enpimage!\n");
        }
      last_processed_time_ = current_time;
      ready_ = true;
    }
    if (debug_)
      {
        printf ("ENPPoseMeasurementPlugin: Added a pose!\n");
      }
    return (true);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  bool
  ENPPoseMeasurementPlugin::addConstraint (gtsam::Symbol sym1, gtsam::Symbol sym2, double enp_score_threshold)
  {
    printf ("ENPPoseMeasurementPlugin: Adding factor between %d and %d\n", sym1.index (), sym2.index ());

//    // Look up images
//    ENPImageConstPtr image1 = images_.at (sym1);
//    gtsam::Pose3 relative_pose = image1->pose3;

//    double trans_noise = trans_noise_;// * enp_score;
//    double rot_noise = rot_noise_;// * enp_score;
//    gtsam::SharedDiagonal noise = gtsam::noiseModel::Diagonal::Sigmas ((gtsam::Vector(6) << rot_noise, rot_noise, rot_noise, trans_noise, trans_noise, trans_noise));

//    omnimapper::OmniMapperBase::NonlinearFactorPtr between (new gtsam::BetweenFactor<gtsam::Pose3> (sym1, sym2, relative_pose, noise));

//    if (debug_)
//      {
//        printf ("ADDED FACTOR BETWEEN x%zu and x%zu\n", sym1.index (), sym2.index ());
//        relative_pose.print ("\n\nENP Relative Pose\n");
//        //        printf ("ENP SCORE: %lf\n", enp_score);
//        printf ("relative pose det: %lf\n", relative_pose.rotation ().matrix ().determinant ());
//      }

//    mapper_->addFactor (between);
//    return (true);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  bool
  ENPPoseMeasurementPlugin::ready ()
  {
    boost::mutex::scoped_lock (current_image_mutex_);
    if (debug_)
      printf ("ENPTest: ready: %zu\n", (!have_new_image_));
    return (!have_new_image_);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  omnimapper::Time
  ENPPoseMeasurementPlugin::getLastProcessedTime ()
  {
    boost::mutex::scoped_lock (current_image_mutex_);
    return (last_processed_time_);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ENPPoseMeasurementPlugin::ENPImageConstPtr
  ENPPoseMeasurementPlugin::getENPImagePtr(gtsam::Symbol sym)
  {
    if (debug_)
      printf ("ENPPlugin: In getENPImagePtr!\n");
    if (images_.count (sym) > 0)
      return (images_.at (sym));
    else
      {
        printf ("ERROR: REQUESTED SYMBOL WITH NO POINTS!\n");
        ENPImageConstPtr empty (new ENPImage ());
        return (empty);
      }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  typename Eigen::Affine3d
  ENPPoseMeasurementPlugin::getSensorToBaseAtSymbol (gtsam::Symbol sym)
  {
    if (sensor_to_base_transforms_.count (sym) > 0)
      {
        return (sensor_to_base_transforms_.at (sym));
      }
    else
      {
        return (Eigen::Affine3d::Identity ());
      }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void
  ENPPoseMeasurementPlugin::applyBaseTransform (ENPImagePtr& image){

//    // Get the previous pose and image
//    boost::posix_time::ptime current_time;
//    if (overwrite_timestamps_)
//      {
//        current_time = boost::posix_time::ptime ( boost::posix_time::microsec_clock::local_time() );
//      }
//    else
//      {
//        current_time = omnimapper::stamp2ptime (image->header.stamp);
//      }


//    // Apply sensor to base transform, if we have one
//    //    ENPImagePtr image_base (new ENPImage ());
//    ENPImagePtr image_base = image;
//    if (get_sensor_to_base_)
//      {
//        if (debug_)
//          {
//            printf ("ENPPosePlugin: Applying sensor to base transform\n");
//          }
//        //        std::cout << "Timestamp: " << current_time << std::endl;
//        gtsam::Pose3 sensor_to_base (((*get_sensor_to_base_)(current_time)).matrix());
//        //        std::cout << "Sensor to base: " << sensor_to_base.translation() << std::endl;
//        //        std::cout << "Sensor to base rot: " << sensor_to_base.rotation() << std::endl;
//        image_base->pose3.transform_to(sensor_to_base);

//      }
//    else
//      {
//        if (debug_)
//          printf ("ENPPosePlugin: No sensor to base transform exists!\n");
//      }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void
  ENPPoseMeasurementPlugin::reset ()
  {
    initialized_ = false;
    have_new_image_ = false;
    first_ = true;
    images_.clear ();
    sensor_to_base_transforms_.clear ();
  }

}
