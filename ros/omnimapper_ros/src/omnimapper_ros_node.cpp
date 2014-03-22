#include <omnimapper_ros/omnimapper_ros.h>

int main (int argc, char** argv)
{
  ros::init (argc, argv, "OmniMapperROSNode");
  //ProfilerStart ("omnimapper_ros_node.prof");
  ros::NodeHandle nh ("~");
  OmniMapperROS<PointT> omnimapper (nh);
  ros::spin ();
  //ProfilerStop ();
}
