#pragma once

#ifndef OMNIMAPPER_ROS_TIME_UTILS_H_
#define OMNIMAPPER_ROS_TIME_UTILS_H_

#include <ros/ros.h>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace omnimapper
{
  /** Converts a ros Time to Boost posix time.  Just syntactic sugar to match our other function. */
  boost::posix_time::ptime rostime2ptime (ros::Time r_time);
  
  // {
  //   return (r_time.toBoost ());
  // }

  /** Converts a ptime to a ros::Time.  This can be done by computing the total nanoseconds since epoch. */
  ros::Time ptime2rostime (boost::posix_time::ptime p_time);
  
  // {
  //   boost::posix_time::ptime time_t_epoch(boost::gregorian::date(1970,1,1));
  //   boost::posix_time::time_duration diff = p_time - time_t_epoch;
  //   ros::Time r_time;
  //   r_time.fromNSec(diff.total_nanoseconds ());
  //   return (r_time);
  // }
  
}

#endif
