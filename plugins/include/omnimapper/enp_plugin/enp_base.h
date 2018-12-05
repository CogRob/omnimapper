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

#include <opencv2/core/core.hpp>
#include <pcl/io/pcd_grabber.h>

/* \brief ENPBase adds sequential pose constraints based on edge tracking to the SLAM problem.
    author Ruffin White
*/

class ENPBase
{

public:

    struct ENPHeader
    {
        ENPHeader (): seq (0), stamp (), frame_id ()
        {}

        pcl::uint32_t seq;
        pcl::uint64_t stamp;

        std::string frame_id;

        typedef boost::shared_ptr<ENPHeader> Ptr;
        typedef boost::shared_ptr<ENPHeader const> ConstPtr;
    };
    typedef boost::shared_ptr<ENPHeader> ENPHeaderPtr;
    typedef boost::shared_ptr<ENPHeader const> ENPHeaderConstPtr;

    struct ENPImage
    {
        ENPImage (): img (), header ()
        {}

        cv::Mat img;
        ENPHeader header;

        typedef boost::shared_ptr<ENPImage> Ptr;
        typedef boost::shared_ptr<ENPImage const> ConstPtr;
    };
    typedef ENPImage::Ptr ENPImagePtr;
    typedef ENPImage::ConstPtr ENPImageConstPtr;

    ENPBase()
    {}
};
