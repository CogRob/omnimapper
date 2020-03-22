#pragma once

#include <omnimapper/get_transform_functor.h>
#include <omnimapper_ros/ros_tf_utils.h>
#include <tf/transform_listener.h>

namespace omnimapper {
/** \brief GetTransformFunctorTF enables lookup of a (potentially  dynamic)
 * transform at a given time, using ROS's TF. See
 * omnimapper/get_transform_functor.h for more details.
 */
class GetTransformFunctorTF : public GetTransformFunctor {
 public:
  GetTransformFunctorTF(std::string sensor_frame_name,
                        std::string base_frame_name);
  Eigen::Affine3d operator()(omnimapper::Time t);

 protected:
  tf::TransformListener tf_listener_;
  std::string sensor_frame_name_;
  std::string base_frame_name_;
};

}  // namespace omnimapper
