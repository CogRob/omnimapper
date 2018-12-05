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
#include <math.h>

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
    counter_ (0),
    counter_th_(10),
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
    gtsam::Pose3 sensor_to_base (((*get_sensor_to_base_)(current_time)).matrix());
  //  sensor_to_base.print("pose");

    bool all_good = true;
    rot_noise_ = 1;
    trans_noise_ = 0.5;

    // For each detection
    for(unsigned int i = 0; i < current_message->detections.size(); ++i)
    {
        //coppy the detection into baseframe
        EBTDetection detection_base = current_message->detections[i];
        // hash the landmark's name
        std::size_t h = string_hash(detection_base.ns);
        // bit shift to the right so it's not to large for GTSAM
        h = h >> 8;
        gtsam::Symbol object_sym_(gtsam::Symbol ('l', h));
        detection_base.pose = sensor_to_base.compose(detection_base.pose);
        detection_base.noise->print("Writing the covariance for the ebt");





        // if detection is good
       if (detection_base.good){

            if (detections_.count (object_sym_) == 0){
                EBTPoseMeasurementPlugin::addDetection (detection_base, current_time, object_sym_, current_pose);
            }
            gtsam::Pose3 test_pose = current_pose->compose(detection_base.pose);
            gtsam::Values current_solution = mapper_->getSolution();
            gtsam::NonlinearFactorGraph current_graph = mapper_->getGraph();
            double x = 0.6;
            double y = 0.6;
            bool xr = true;
            bool yr = true;
            bool zr = true;
            bool xt = true;
            bool yt = true;
            bool zt = true;
            gtsam::Pose3 delta_pose;
            gtsam::Marginals marginals(current_graph,current_solution);
            gtsam::Matrix mar_covar;

            if (current_solution.exists(object_sym_) && current_pose.is_initialized())
            {
                detection_base.noise->print("Before: detection_base.noise");
                gtsam::Pose3 estimate_pose = mapper_->getSolution().at<gtsam::Pose3>(object_sym_);
                delta_pose = estimate_pose.between(test_pose);
                double test_dist = estimate_pose.range(test_pose);
                double test_angdist = gtsam::norm_2(delta_pose.rotation().xyz());
                double theta = 0.05;
                double thetang = 0.15;
                x = (2*theta)/(theta + test_dist);
                y = (2*thetang)/(thetang + test_angdist);
                gtsam::Vector sigmas(detection_base.noise->sigmas());
                std::cout << "Before: spp: " << sigmas << " " <<x<< " "<<y<<std::endl;
                sigmas = sigmas/x;
                trans_noise_ = trans_noise_/x;
                rot_noise_ = rot_noise_/y;
                std::cout << "After: spp: " << rot_noise_ <<" "<<trans_noise_<< std::endl;
                //detection_base.noise = gtsam::noiseModel::Diagonal::Sigmas ((sigmas));
                // test_pose.print("After: test_pose: ");
                // printf("After: test_dist: %f \n", test_dist);
                // printf("After: theta: %f \n", theta);
                // printf("After: x: %f \n", x);
                // detection_base.noise->print("After: detection_base.noise");

                mar_covar = marginals.marginalCovariance(object_sym_.key());
                std::cout<<"Outputting marginals"<<mar_covar<<std::endl;
                std::cout<<"Outputting values"<<estimate_pose<<std::endl;
            //    bool temp = if(current_pose == NULL);
                std::cout<<"Outputting delta"<<current_pose.is_initialized()<<" "<<delta_pose.translation()<<" "<<delta_pose.rotation().xyz()<<std::endl;
                current_pose->print(" The current tracking solution");
                int sigmaNum = 2;

                xr = fabs(delta_pose.rotation().xyz().coeff(0)) < fabs(1.5*sigmaNum*mar_covar.coeff(0,0));// + 0.1);
                yr = fabs(delta_pose.rotation().xyz().coeff(1)) < fabs(1.5*sigmaNum*mar_covar.coeff(1,1));// + 0.1);
                zr = fabs(delta_pose.rotation().xyz().coeff(2)) < fabs(1.5*sigmaNum*mar_covar.coeff(2,2)); // + 0.1);
                xt = fabs(delta_pose.x()) < fabs(sigmaNum*mar_covar.coeff(3,3)); // + 0.05);
                yt = fabs(delta_pose.y()) < fabs(sigmaNum*mar_covar.coeff(4,4)); // + 0.05);
                zt = fabs(delta_pose.z()) < fabs(sigmaNum*mar_covar.coeff(5,5));// + 0.05);

            }

            // Add constraints
            if( xt && yt  && zt && xr && yr && zr){

                if(current_pose.is_initialized())
                {
                boost::thread latest_ebt_thread (
                            &EBTPoseMeasurementPlugin::addConstraint, this, current_sym, object_sym_, detection_base);
                latest_ebt_thread.join ();
                // reset our counter
                counter_ = 0;

                }



                // Wait for latest one to complete, at least

            }
            else{
                // set the detection to bad
                current_message->detections[i].good = false;

                // check if we've seen it before
                if (detections_.count (object_sym_) == 0){
                    current_message->detections[i].init = true;
                }
                // if we have use the predicted pose
                else{
                    gtsam::Pose3 estimate_pose = mapper_->getSolution().at<gtsam::Pose3>(object_sym_);
                    gtsam::Pose3 world_to_sensor = current_pose->compose(sensor_to_base);
                    current_message->detections[i].pose = world_to_sensor.between(estimate_pose);
                    // but consider our counter
                    EBTPoseMeasurementPlugin::reinit (current_message, i);
                    ++counter_;
                }
                all_good = false;
            }//*/
        }//*/

        // if detection is bad
        else{
            // set the detection to bad
            current_message->detections[i].good = false;

            // check if we've seen it before
            if (detections_.count (object_sym_) == 0){
                current_message->detections[i].init = true;
            }
            // if we have use the predicted pose
            else{
                gtsam::Pose3 estimate_pose = mapper_->getSolution().at<gtsam::Pose3>(object_sym_);
                gtsam::Pose3 world_to_sensor = current_pose->compose(sensor_to_base);
                current_message->detections[i].pose = world_to_sensor.between(estimate_pose);
                // but consider our counter
                EBTPoseMeasurementPlugin::reinit (current_message, i);
                ++counter_;
            }
            all_good = false;
        }//*/
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
EBTPoseMeasurementPlugin::addConstraint (gtsam::Symbol sym1, gtsam::Symbol sym2, EBTDetection detection)
{
    printf ("EBTPoseMeasurementPlugin: Adding factor between %lu and %lu\n", sym1.index (), sym2.index ());

    double trans_noise = trans_noise_;
    double rot_noise = rot_noise_;
    std::cout<<"Noise models R & T"<<trans_noise<<"  "<<rot_noise<<std::endl;
    gtsam::SharedDiagonal noise = gtsam::noiseModel::Diagonal::Sigmas ((gtsam::Vector(6) << rot_noise, rot_noise, rot_noise, trans_noise, trans_noise, trans_noise));

    omnimapper::OmniMapperBase::NonlinearFactorPtr between (new gtsam::BetweenFactor<gtsam::Pose3> (sym1, sym2, detection.pose, noise));

    if (debug_)
    {
        printf ("ADDED FACTOR BETWEEN l%zu and x%zu\n", sym1.index (), sym2.index ());
        detection.pose.print ("\n\nEBT Relative Pose\n");
        printf ("relative pose det: %lf\n", detection.pose.rotation ().matrix ().determinant ());
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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
EBTPoseMeasurementPlugin::reinit (EBTMessagePtr current_message, unsigned int i)
{
    if (counter_ <= counter_th_){
        if (debug_)
        {
            printf ("EBTPoseMeasurementPlugin: Reinit using Pose! Counter=%d\n", counter_);
        }
    }
    else{
        if (debug_)
        {
            printf ("EBTPoseMeasurementPlugin: Reinit using Surf! Counter=%d\n", counter_);
        }
        current_message->detections[i].init = true;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
EBTPoseMeasurementPlugin::addDetection (EBTDetection detection, boost::posix_time::ptime current_time, gtsam::Symbol object_sym_, boost::optional<gtsam::Pose3> current_pose)
{
    EBTDetectionPtr detection_const (new EBTDetection ());
    *detection_const = detection;
    std::map<boost::posix_time::ptime, EBTDetectionConstPtr> detection_map_const;
    detection_map_const.insert(
                std::pair<boost::posix_time::ptime, EBTDetectionConstPtr>
                (current_time, detection_const));

    detections_.insert (
                std::pair<gtsam::Symbol, std::map<boost::posix_time::ptime, EBTDetectionConstPtr> >
                (object_sym_, detection_map_const));

    gtsam::Pose3 initial_guess = current_pose->compose(detection.pose);
    initial_guess.print ("\n\ninitial_guess\n");
    mapper_->addNewValue(object_sym_,initial_guess);
}

}
