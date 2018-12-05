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

#include <gtsam/geometry/Pose3.h>
#include <pcl/io/pcd_grabber.h>

/* \brief APRILTagBase adds sequential pose constraints based on edge tracking to the SLAM problem.
    author Ruffin White
*/

class APRILTagBase
{

public:

    struct APRILTagDetection
    {
        APRILTagDetection (): id(), pose ()
        {}

        uint id;
        gtsam::Pose3 pose;

        typedef boost::shared_ptr<APRILTagDetection> Ptr;
        typedef boost::shared_ptr<APRILTagDetection const> ConstPtr;
    };
    typedef boost::shared_ptr<APRILTagDetection> APRILTagDetectionPtr;
    typedef boost::shared_ptr<APRILTagDetection const> APRILTagDetectionConstPtr;

    struct APRILTagHeader
    {
        APRILTagHeader (): seq (0), stamp (), frame_id ()
        {}

        pcl::uint32_t seq;
        pcl::uint64_t stamp;

        std::string frame_id;

        typedef boost::shared_ptr<APRILTagHeader> Ptr;
        typedef boost::shared_ptr<APRILTagHeader const> ConstPtr;
    };
    typedef boost::shared_ptr<APRILTagHeader> APRILTagHeaderPtr;
    typedef boost::shared_ptr<APRILTagHeader const> APRILTagHeaderConstPtr;

    struct APRILTagMessage
    {
        APRILTagMessage (): header (), detections ()
        {}

        APRILTagHeader header;
        std::vector<APRILTagDetection> detections;

        typedef boost::shared_ptr<APRILTagMessage> Ptr;
        typedef boost::shared_ptr<APRILTagMessage const> ConstPtr;
    };
    typedef APRILTagMessage::Ptr APRILTagMessagePtr;
    typedef APRILTagMessage::ConstPtr APRILTagMessageConstPtr;

    APRILTagBase()
    {}
};
