
namespace omnimapper
{
  /** \brief PlaneMeasurementPlugin adds factors for planar landmarks extracted from 3D point cloud data.
   *  \author Alex Trevor
   */
  temppate <typename PointT>
  class PlaneMeasurementPlugin //:public omnimapper::MeasurementPlugin
  {
    typedef typename pcl::PointCloud<PointT> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;    
    
    public:
      PlaneMeasurementPlugin (omnimapper::OmniMapperBase* mapper);
      
  };
  
}
