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
#include <omnimapper/time.h>

namespace omnimapper {
/** \brief TriggerFunctor
 *
 */
class TriggerFunctor {
 public:
  virtual bool operator()(Time time) = 0;
};

/** \brief TriggerAlways will always add a pose. */
class TriggerAlways : public TriggerFunctor {
 public:
  TriggerAlways() {}

  bool operator()(Time time) { return true; }

 private:
  Time the_future_;
};

/** \brief TriggerPeriodic will add a pose at the specified rate. */
class TriggerPeriodic : public TriggerFunctor {
 public:
  TriggerPeriodic(GetTimeFunctorPtr get_time, float duration)
      : get_time_(get_time), duration_(duration), prev_time_((*get_time_)()) {}

  bool operator()(Time time) {
    Time now = (*get_time_)();
    if (((now - prev_time_) > boost::posix_time::seconds(duration_))) {
      prev_time_ = now;
      return (true);
    }
    return (false);
  }

 private:
  GetTimeFunctorPtr get_time_;
  float duration_;
  Time prev_time_;
};

typedef boost::shared_ptr<omnimapper::TriggerFunctor> TriggerFunctorPtr;
}  // namespace omnimapper
