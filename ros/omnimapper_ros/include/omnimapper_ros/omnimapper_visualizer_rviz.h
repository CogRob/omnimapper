#include <omnimapper/omnimapper_base.h>
#include <omnimapper/icp_pose_plugin.h>
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>

namespace omnimapper
{
    /** \brief OmniMapperVisualizerRViz is an output plugin for OmniMapper based on the RViz.  Currently
   *  this only supports visualization of a trajectory, and clouds from an ICP plugin.
   *
   * \author Alex Trevor
   */
  template <typename PointT>
  class OmniMapperVisualizerRViz : public omnimapper::OutputPlugin
  {
    typedef typename pcl::PointCloud<PointT> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    public:
      OmniMapperVisualizerRViz (omnimapper::OmniMapperBase* mapper);
      void update (boost::shared_ptr<gtsam::Values>& vis_values);

    protected:
      ros::NodeHandle nh_;
      
      OmniMapperBase* mapper_;
      
      ros::Publisher pose_array_pub_;
      
  };
}
