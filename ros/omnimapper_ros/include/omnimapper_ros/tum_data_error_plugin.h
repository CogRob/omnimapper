#include <omnimapper/omnimapper_base.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <omnimapper_ros/ros_time_utils.h>

namespace omnimapper
{
  /** \brief TUMDataError plugin is for doing an evaluation using the TUM Bag files, rather than PCDs.  See ErrorEvaluationPlugin for the plugin used for experiments.*/
  class TUMDataErrorPlugin : public omnimapper::OutputPlugin
  {
    public:
      TUMDataErrorPlugin (omnimapper::OmniMapperBase* mapper);
      void update (boost::shared_ptr<gtsam::Values>& vis_values, boost::shared_ptr<gtsam::NonlinearFactorGraph>& vis_graph);
      
    protected:
      ros::NodeHandle nh_;
      
      tf::TransformListener tf_listener_;

      ros::Publisher marker_array_pub_;

      OmniMapperBase* mapper_;
  };
  
}
