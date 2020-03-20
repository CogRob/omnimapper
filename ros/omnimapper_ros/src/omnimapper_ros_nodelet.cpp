#include  <omnimapper_ros/omnimapper_ros_nodelet.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(omnimapper_ros::OmniMapperROSNodelet, nodelet::Nodelet)

namespace omnimapper_ros
{
  OmniMapperROSNodelet::OmniMapperROSNodelet ()
  {
  }

  OmniMapperROSNodelet::~OmniMapperROSNodelet ()
  {
  }

  void
  OmniMapperROSNodelet::onInit ()
  {
    omnimapper_.reset (new OmniMapperROS<pcl::PointXYZRGBA>(this->getPrivateNodeHandle ()));
  }


}
