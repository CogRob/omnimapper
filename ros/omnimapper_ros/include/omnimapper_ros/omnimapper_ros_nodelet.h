#pragma once

#include <nodelet/nodelet.h>
#include <omnimapper_ros/omnimapper_ros.h>

namespace omnimapper_ros
{
  class OmniMapperROSNodelet : public nodelet::Nodelet
  {
    private:
      boost::shared_ptr<OmniMapperROS<pcl::PointXYZRGBA> > omnimapper_;
    public:
      OmniMapperROSNodelet ();
      virtual ~OmniMapperROSNodelet ();
      virtual void onInit ();  
  };

}
