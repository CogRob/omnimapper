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

#include <omnimapper/apriltag_plugin/apriltag_plugin.h>
#include <omnimapper/omnimapper_base.h>
#include <omnimapper/time.h>
#include <pcl/common/time.h>

namespace omnimapper{
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  AprilTagPoseMeasurementPlugin::AprilTagPoseMeasurementPlugin (omnimapper::OmniMapperBase* mapper) :
    mapper_ (mapper),
    get_sensor_to_base_ (GetTransformFunctorPtr ()),
    last_processed_time_ (),
    trigger_ (new omnimapper::TriggerAlways ()),
    triggered_time_ (omnimapper::epoch_time ()),
    initialized_ (false),
    have_new_message_ (false),
    ready_ (true),
    first_ (true),
    trans_noise_ (.05),
    rot_noise_ (.05),
    debug_ (true),
    overwrite_timestamps_ (false)
  {
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  AprilTagPoseMeasurementPlugin::~AprilTagPoseMeasurementPlugin ()
  {
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void
  AprilTagPoseMeasurementPlugin::apriltagCallback (const APRILTagMessageConstPtr &message)
  {
    if (debug_)
      printf ("apriltag callback\n");

    Time measurement_time = omnimapper::stamp2ptime (message->header.stamp);
    bool use_measurement = (*trigger_)(measurement_time);
    if (!use_measurement)
      return;

    // Store this as the previous message
    boost::mutex::scoped_lock lock (current_message_mutex_);
    current_message_ = message;
    have_new_message_ = true;
    current_message_cv_.notify_one ();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void AprilTagPoseMeasurementPlugin::spin ()
  {
    while (true)
      {
        spinOnce ();
        boost::this_thread::sleep (boost::posix_time::milliseconds (10));
      }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  bool
  AprilTagPoseMeasurementPlugin::spinOnce ()
  {
      if(!initialized_)
      {
          initialized_ = true;
      }

      APRILTagMessagePtr current_message (new APRILTagMessage ());
      {
          boost::mutex::scoped_lock lock (current_message_mutex_);

          while (!have_new_message_)
          {
              current_message_cv_.wait (lock);
          }
          *current_message = *current_message_;
          have_new_message_ = false;
      }

      if (debug_)
      {
          printf ("current APRILTagMessage size: %lu\n", current_message->detections.size());
      }

      // Get the previous pose and message
      gtsam::Symbol current_sym;
      boost::posix_time::ptime current_time;
      if (overwrite_timestamps_)
      {
          current_time = boost::posix_time::ptime ( boost::posix_time::microsec_clock::local_time() );
      }
      else
      {
          current_time = omnimapper::stamp2ptime (current_message->header.stamp);
      }

      if (debug_)
      {
          std::cout << "APRILTag Plugin: Getting symbol for current time: " << current_time << std::endl;
      }

      mapper_->getPoseSymbolAtTime (current_time, current_sym);
      boost::optional<gtsam::Pose3> current_pose = mapper_->predictPose (current_sym);

      if (debug_)
      {
          printf ("APRILTag Plugin: current symbol: %zu, inserting message\n", current_sym.index ());
      }

  //    gtsam::Pose3 sensor_to_base = gtsam::Pose3(getSensorToBaseAtSymbol(current_sym).matrix());
      gtsam::Pose3 sensor_to_base (((*get_sensor_to_base_)(current_time)).matrix());


      //    messages_.insert (std::pair<gtsam::Symbol, APRILTagMessageConstPtr> (current_sym, current_message));

      for(unsigned int i = 0; i < current_message->detections.size(); ++i){
          APRILTagDetection detection_base = current_message->detections[i];
          gtsam::Symbol tag_sym_(gtsam::Symbol ('apriltag', detection_base.id));
          detection_base.pose = sensor_to_base.compose(detection_base.pose);

          if (debug_)
          {
              sensor_to_base.print ("\n\nsensor_to_base\n");
              detection_base.pose.print ("\n\ndetection_base.pose\n");
          }

          if (detections_.count (tag_sym_) == 0){
              APRILTagDetectionPtr detection_base_const (new APRILTagDetection ());
              *detection_base_const = detection_base;
              std::map<boost::posix_time::ptime, APRILTagDetectionConstPtr> detection_map_const;
              detection_map_const.insert(
                          std::pair<boost::posix_time::ptime, APRILTagDetectionConstPtr>
                          (current_time, detection_base_const));

              detections_.insert (
                          std::pair<gtsam::Symbol, std::map<boost::posix_time::ptime, APRILTagDetectionConstPtr> >
                          (tag_sym_, detection_map_const));

              gtsam::Pose3 initial_guess = current_pose->compose(detection_base.pose);
              initial_guess.print ("\n\ninitial_guess\n");
              mapper_->addNewValue(tag_sym_,initial_guess);
          }
          // Add constraints
          boost::thread latest_apriltag_thread (
                      &AprilTagPoseMeasurementPlugin::addConstraint, this, current_sym, tag_sym_, detection_base.pose);
          // Wait for latest one to complete, at least
          latest_apriltag_thread.join ();

      }

      // Note that we're done
      {
          //boost::mutex::scoped_lock lock (current_message_processed_mutex_);
          if (debug_)
          {
              printf ("APRILTagTest: done with apriltagmessage!\n");
          }
          last_processed_time_ = current_time;
          ready_ = true;
      }
      if (debug_)
      {
          printf ("AprilTagPoseMeasurementPlugin: Added a pose!\n");
      }
      return (true);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  bool
  AprilTagPoseMeasurementPlugin::addConstraint (gtsam::Symbol sym1, gtsam::Symbol sym2, gtsam::Pose3 relative_pose)
  {
      printf ("AprilTagPoseMeasurementPlugin: Adding factor between %lu and %lu\n", sym1.index (), sym2.index ());

      double trans_noise = trans_noise_;
      double rot_noise = rot_noise_;
      gtsam::SharedDiagonal noise = gtsam::noiseModel::Diagonal::Sigmas (
                  (gtsam::Vector(6) << rot_noise, rot_noise, rot_noise, trans_noise, trans_noise, trans_noise));

      omnimapper::OmniMapperBase::NonlinearFactorPtr between (
                  new gtsam::BetweenFactor<gtsam::Pose3> (sym1, sym2, relative_pose, noise));

      if (debug_)
      {
          printf ("ADDED FACTOR BETWEEN x%zu and apriltag%zu\n", sym1.index (), sym2.index ());
          relative_pose.print ("\n\nEBT Relative Pose\n");
          printf ("relative pose det: %lf\n", relative_pose.rotation ().matrix ().determinant ());
      }

      mapper_->addFactor (between);
      return (true);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  bool
  AprilTagPoseMeasurementPlugin::ready ()
  {
    boost::mutex::scoped_lock (current_message_mutex_);
    if (debug_)
      printf ("APRILTagTest: ready: %zu\n", (!have_new_message_));
    return (!have_new_message_);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  omnimapper::Time
  AprilTagPoseMeasurementPlugin::getLastProcessedTime ()
  {
    boost::mutex::scoped_lock (current_message_mutex_);
    return (last_processed_time_);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  AprilTagPoseMeasurementPlugin::APRILTagMessageConstPtr
  AprilTagPoseMeasurementPlugin::getAPRILTagMessagePtr(gtsam::Symbol sym)
  {
    if (debug_)
      printf ("AprilTagPlugin: In getAPRILTagMessagePtr!\n");
    if (messages_.count (sym) > 0)
      return (messages_.at (sym));
    else
      {
        printf ("ERROR: REQUESTED SYMBOL WITH NO POINTS!\n");
        APRILTagMessageConstPtr empty (new APRILTagMessage ());
        return (empty);
      }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  typename Eigen::Affine3d
  AprilTagPoseMeasurementPlugin::getSensorToBaseAtSymbol (gtsam::Symbol sym)
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
  AprilTagPoseMeasurementPlugin::reset ()
  {
    initialized_ = false;
    have_new_message_ = false;
    first_ = true;
    messages_.clear ();
    sensor_to_base_transforms_.clear ();
  }

}
