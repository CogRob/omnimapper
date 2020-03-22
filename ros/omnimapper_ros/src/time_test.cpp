#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_listener.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "boost/date_time/local_time/local_time.hpp"
//#include <omnimapper_ros/ros_time_utils.h>

class TimeTest {
 public:
  ros::NodeHandle n_;

  TimeTest() : n_("~") {
    ros::Time t1_ros = ros::Time::now();
    std::cout << "Ros Time: " << t1_ros << std::endl;
    boost::posix_time::ptime t1_boost = t1_ros.toBoost();
    std::string t1_boost_str = boost::posix_time::to_simple_string(t1_boost);
    std::cout << "Boost Time: " << t1_boost << std::endl;
    // std::cout << "Boost msec: " << t1_boost.total_milliseconds() <<
    // std::endl; ros::Time t1_ros_again = ptime2rostime (t1_boost); std::cout
    // << "Ros Time: " << t1_ros << std::endl;

    boost::posix_time::ptime time_t_epoch(boost::gregorian::date(1970, 1, 1));
    boost::posix_time::time_duration diff = t1_boost - time_t_epoch;
    std::cout << "Since epoch: " << diff.total_nanoseconds() << std::endl;

    ros::Time t1_ros_again;
    t1_ros_again.fromNSec(diff.total_nanoseconds());
    std::cout << "Ros Time again: " << t1_ros_again << std::endl;
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "TimeTestNode");
  TimeTest ttn;
  ros::spin();
  return (0);
}
