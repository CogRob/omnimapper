#include <glog/logging.h>
#include <omnimapper_ros/omnimapper_ros.h>
#include <omnimapper_ros/ros_log_sink.h>

int main(int argc, char** argv) {
  FLAGS_logtostderr = 1;
  google::InitGoogleLogging(argv[0]);
  omnimapper::ScopedRosLogSink ros_log_sink;
  ros::init(argc, argv, "OmniMapperROSNode");
  // ProfilerStart ("omnimapper_ros_node.prof");
  ros::NodeHandle nh("~");
  OmniMapperROS<PointT> omnimapper(nh);
  ros::spin();
  // ProfilerStop ();
}
