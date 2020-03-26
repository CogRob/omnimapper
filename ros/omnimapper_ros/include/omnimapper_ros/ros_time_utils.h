#pragma once

#ifndef OMNIMAPPER_ROS_TIME_UTILS_H_
#define OMNIMAPPER_ROS_TIME_UTILS_H_

#include <omnimapper/time.h>
#include <ros/ros.h>

#include <boost/date_time/posix_time/posix_time.hpp>

namespace omnimapper {
/** Converts a ros Time to Boost posix time.  Just syntactic sugar to match our
 * other function. */
boost::posix_time::ptime RosTimeToPtime(ros::Time r_time);

/** Converts a ptime to a ros::Time.  This can be done by computing the total
 * nanoseconds since epoch. */
ros::Time PtimeToRosTime(boost::posix_time::ptime p_time);

/** \brief An omnimapper compatible time functor, which will return the current
 * ros time. */
class GetROSTimeFunctor : public omnimapper::GetTimeFunctor {
 public:
  omnimapper::Time operator()() { return (RosTimeToPtime(ros::Time::now())); }
};

}  // namespace omnimapper

#endif
