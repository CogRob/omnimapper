#include <omnimapper_ros/ros_time_utils.h>

boost::posix_time::ptime omnimapper::RosTimeToPtime(ros::Time r_time) {
  return (r_time.toBoost());
}

ros::Time omnimapper::PtimeToRosTime(boost::posix_time::ptime p_time) {
  boost::posix_time::ptime time_t_epoch(boost::gregorian::date(1970, 1, 1));
  boost::posix_time::time_duration diff = p_time - time_t_epoch;
  ros::Time r_time;
  r_time.fromNSec(diff.total_nanoseconds());
  return (r_time);
}
