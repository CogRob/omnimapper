#pragma once

#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <omnimapper/get_transform_functor.h>
#include <omnimapper/omnimapper_base.h>
#include <ros/ros.h>

namespace omnimapper {
class ARMarkerPlugin {
 public:
  ARMarkerPlugin(omnimapper::OmniMapperBase* mapper);

  void markerCallback(const ar_track_alvar_msgs::AlvarMarkers& msg);

  void setSensorToBaseFunctor(
      omnimapper::GetTransformFunctorPtr get_transform) {
    get_sensor_to_base_ = get_transform;
  }

 protected:
  OmniMapperBase* mapper_;
  std::set<uint> known_markers_;
  GetTransformFunctorPtr get_sensor_to_base_;
  ros::NodeHandle nh_;
  ros::Subscriber marker_sub_;
};

}  // namespace omnimapper
