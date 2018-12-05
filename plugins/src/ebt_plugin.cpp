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

#include <omnimapper/ebt_plugin/ebt_plugin.h>
#include <omnimapper/omnimapper_base.h>
#include <omnimapper/time.h>
#include <pcl/common/time.h>

namespace omnimapper{
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
EBTPoseMeasurementPlugin::EBTPoseMeasurementPlugin (omnimapper::OmniMapperBase* mapper) :
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

EBTPoseMeasurementPlugin::~EBTPoseMeasurementPlugin ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
EBTPoseMeasurementPlugin::ebtmessageCallback (const EBTMessageConstPtr &message)
{
    if (debug_)
        printf ("ebtmessage callback\n");

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
void EBTPoseMeasurementPlugin::spin ()
{
    while (true)
    {
        spinOnce ();
        boost::this_thread::sleep (boost::posix_time::milliseconds (10));
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
EBTPoseMeasurementPlugin::spinOnce ()
{
    if(!initialized_)
    {
        initialized_ = true;
    }

    EBTMessagePtr current_message (new EBTMessage ());
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
        printf ("current EBTMessage size: %lu\n", current_message->detections.size());
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
        std::cout << "EBT Plugin: Getting symbol for current time: " << current_time << std::endl;
    }

    mapper_->getPoseSymbolAtTime (current_time, current_sym);
    boost::optional<gtsam::Pose3> current_pose = mapper_->predictPose (current_sym);

    if (debug_)
    {
        printf ("EBT Plugin: current symbol: %zu, inserting message\n", current_sym.index ());
    }

//    gtsam::Pose3 sensor_to_base = gtsam::Pose3(getSensorToBaseAtSymbol(current_sym).matrix());
   //  gtsam::Pose3 sensor_to_base (((*get_sensor_to_base_)(current_time)).matrix());

    gtsam::Rot3 tempRot = gtsam::Rot3::ypr( -1.571,0.00, -1.571);
    gtsam::Point3 tempT = gtsam::Point3(0.00,-0.045, 0.000);
    gtsam::Pose3 sensor_to_base = gtsam::Pose3(tempRot, tempT);
    // if sensor_to_base ==

    //    messages_.insert (std::pair<gtsam::Symbol, EBTMessageConstPtr> (current_sym, current_message));

    bool all_good = true;

    for(unsigned int i = 0; i < current_message->detections.size(); ++i){
        EBTDetection detection_base = current_message->detections[i];
        // hash the landmark's name
        std::size_t h = string_hash(detection_base.ns);
        // bit shift to the right so it's not to large for GTSAM
        h = h >> 8;
        gtsam::Symbol object_sym_(gtsam::Symbol ('l', h));
        detection_base.pose = sensor_to_base.compose(detection_base.pose);
        std::cout<<"Values for sensor to base"<<sensor_to_base.rotation().rpy()<<std::endl;

        if (debug_){
            sensor_to_base.print ("\n\nsensor_to_base\n");
            detection_base.pose.print ("\n\ndetection_base.pose\n");
        }

        if (detections_.count (object_sym_) == 0){
            EBTDetectionPtr detection_base_const (new EBTDetection ());
            *detection_base_const = detection_base;
            std::map<boost::posix_time::ptime, EBTDetectionConstPtr> detection_map_const;
            detection_map_const.insert(
                        std::pair<boost::posix_time::ptime, EBTDetectionConstPtr>
                        (current_time, detection_base_const));

            detections_.insert (
                        std::pair<gtsam::Symbol, std::map<boost::posix_time::ptime, EBTDetectionConstPtr> >
                        (object_sym_, detection_map_const));
          //  gtsam::Pose3 initial_guess = intial_guess.
            gtsam::Pose3 initial_guess = current_pose->compose(detection_base.pose);
            initial_guess = initial_guess.identity();
            initial_guess.print ("\n\ninitial_guess\n");
            mapper_->addNewValue(object_sym_,initial_guess);
        }


        gtsam::NonlinearFactorGraph current_graph = mapper_->getGraph();
        gtsam::Pose3 test_pose = current_pose->compose(detection_base.pose);
        gtsam::Values current_solution = mapper_->getSolution();
        gtsam::Marginals marginals = gtsam::Marginals(current_graph,current_solution);
        if (current_solution.exists(object_sym_))
        {
            gtsam::Pose3 estimate_pose = mapper_->getSolution().at<gtsam::Pose3>(object_sym_);
            double test_dist = estimate_pose.range(test_pose);
            gtsam::Rot3 test_rot = estimate_pose.rotation().between(test_pose.rotation());
            Eigen::AngleAxisd test_angle(test_rot.matrix());

            if((test_dist > init_distance_threshold_) || (test_angle.angle() > init_angle_threshold_)){
                if (debug_)
                {
                    printf ("EBTPoseMeasurementPlugin: The test_dist of %f is too larg, sending init signal!\n", test_dist);
                }
                all_good = false;

                if(init_with_estimate_){
                    current_message->detections[i].good = false;
                    gtsam::Pose3 world_to_sensor = current_pose->compose(sensor_to_base); //.inverse());
                    current_message->detections[i].pose = world_to_sensor.between(estimate_pose);

                    world_to_sensor.translation().print("world_to_sensor after!!!!!!!!!!!!!");
                    estimate_pose.translation().print("estimate_pose!!!!!!!!!!!!!!!!!");
                    current_message->detections[i].pose.translation().print("current_message->detections[i].pose!!!!!!!!!!!!!!!!!");
                  //  init_distance_threshold_ = 1.5*init_distance_threshold_;
                }
                else
                {
                    current_message->detections[i].init = true;
                }
            }
        }
        if(all_good)
        {
            // Add constraints
            boost::thread latest_ebt_thread (
                        &EBTPoseMeasurementPlugin::addConstraint, this, current_sym, object_sym_, detection_base.pose);
            // Wait for latest one to complete, at least
            if (current_solution.exists(object_sym_))
            {
                gtsam::Matrix information = marginals.marginalInformation(object_sym_);
                gtsam::Vector6 vecdiag = information.diagonal();

                if( (current_sym.index() > 30) && ((vecdiag(0)-initial_covar_obj_(0))>0) && ((vecdiag(1)-initial_covar_obj_(1))>0) && ((vecdiag(2)-initial_covar_obj_(2))>0) && ((vecdiag(3)-initial_covar_obj_(3))>5) && ((vecdiag(4)-initial_covar_obj_(4))>5) && ((vecdiag(5)-initial_covar_obj_(5))>5)  )

                {
                     std::cout<<"information matrix changed"<<std::endl;
                     initial_covar_obj_ = vecdiag;
                     init_distance_threshold_ = 0.1*init_distance_threshold_;
                }

                //    if(information.diagonal() > initial_covar_obj_ )
            //    {
            //        std::cout<<"information matrix changed"<<std::endl;
            //    }

                std::cout<<"information matrix of landmark "<<vecdiag<<" "<<init_distance_threshold_<<std::endl;
            }
            latest_ebt_thread.join ();
        }
    }

    if(!all_good){
        ebtmessage_init_callback_(current_message);
        if (debug_)
        {
            printf ("EBTTest: failing ebtmessage!\n");
        }
        return (false);
    }

    // Note that we're done
    {
        //boost::mutex::scoped_lock lock (current_message_processed_mutex_);
        if (debug_)
        {
            printf ("EBTTest: done with ebtmessage!\n");
        }
        last_processed_time_ = current_time;
        ready_ = true;
    }
    if (debug_)
    {
        printf ("EBTPoseMeasurementPlugin: Added a pose!\n");
    }
    return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
EBTPoseMeasurementPlugin::addConstraint (gtsam::Symbol sym1, gtsam::Symbol sym2, gtsam::Pose3 relative_pose)
{
    printf ("EBTPoseMeasurementPlugin: Adding factor between %lu and %lu\n", sym1.index (), sym2.index ());

    double trans_noise = trans_noise_;
    double rot_noise = rot_noise_;
    gtsam::Vector noise_vector_(6);
    noise_vector_ << rot_noise, rot_noise, rot_noise, trans_noise, trans_noise, trans_noise;
    gtsam::SharedDiagonal noise = gtsam::noiseModel::Diagonal::Sigmas(noise_vector_);

    omnimapper::OmniMapperBase::NonlinearFactorPtr between (new gtsam::BetweenFactor<gtsam::Pose3> (sym1, sym2, relative_pose, noise));

    if (debug_)
    {
        printf ("ADDED FACTOR BETWEEN l%zu and x%zu\n", sym1.index (), sym2.index ());
        relative_pose.print ("\n\nEBT Relative Pose\n");
        printf ("relative pose det: %lf\n", relative_pose.rotation ().matrix ().determinant ());
    }

    mapper_->addFactor (between);
    return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
bool
EBTPoseMeasurementPlugin::ready ()
{
    boost::mutex::scoped_lock (current_message_mutex_);
    if (debug_)
        printf ("EBTTest: ready: %i\n", (!have_new_message_));
    return (!have_new_message_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
omnimapper::Time
EBTPoseMeasurementPlugin::getLastProcessedTime ()
{
    boost::mutex::scoped_lock (current_message_mutex_);
    return (last_processed_time_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
EBTPoseMeasurementPlugin::EBTMessageConstPtr
EBTPoseMeasurementPlugin::getEBTMessagePtr(gtsam::Symbol sym)
{
    if (debug_)
        printf ("EBTPlugin: In getEBTMessagePtr!\n");
    if (messages_.count (sym) > 0)
        return (messages_.at (sym));
    else
    {
        printf ("ERROR: REQUESTED SYMBOL WITH NO POINTS!\n");
        EBTMessageConstPtr empty (new EBTMessage ());
        return (empty);
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
typename Eigen::Affine3d
EBTPoseMeasurementPlugin::getSensorToBaseAtSymbol (gtsam::Symbol sym)
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
EBTPoseMeasurementPlugin::reset ()
{
    initialized_ = false;
    have_new_message_ = false;
    first_ = true;
    messages_.clear ();
    detections_.clear ();
    sensor_to_base_transforms_.clear ();
}

}
