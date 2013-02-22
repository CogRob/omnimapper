#include <omnimapper/omnimapper_base.h>
#include <omnimapper/icp_pose_plugin.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace omnimapper
{
  template <typename PointT>
  class OmniMapperVisualizerPCL : public omnimapper::OutputPlugin
  {
    //typedef typename pcl::PointXYZ PointT;
    typedef typename pcl::PointCloud<PointT> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    public:
      OmniMapperVisualizerPCL (omnimapper::OmniMapperBase* mapper);
      void update (boost::shared_ptr<gtsam::Values>& vis_values);
      void spin ();
      void spinThread ();
      void spinOnce ();
      void setICPPlugin (boost::shared_ptr<omnimapper::ICPPoseMeasurementPlugin<PointT> >& icp_plugin) { icp_plugin_ = icp_plugin; }
      void keyboardCallback (const pcl::visualization::KeyboardEvent& event, void*);
      
      //void spinAndUpdate ();
    protected:
      // A PCL Visualizer
      pcl::visualization::PCLVisualizer viewer_;
      // Visualizer mutex
      boost::mutex vis_mutex_;
      // A reference to a mapper instance
      OmniMapperBase* mapper_;
      
      // Pose Cloud
      boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > pose_cloud_;
      // Updated flag
      bool new_slam_data_;
      // Flag for drawing ICP clouds
      bool draw_icp_clouds_;
      // ICP Plugin Ref
      boost::shared_ptr<omnimapper::ICPPoseMeasurementPlugin<PointT> > icp_plugin_;

      // Debug flag
      bool debug_;
  };
}
