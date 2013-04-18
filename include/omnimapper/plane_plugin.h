
namespace omnimapper
{
  /** \brief PlaneMeasurementPlugin adds factors for planar landmarks extracted from 3D point cloud data.
   *  \author Alex Trevor
   */
  template <typename PointT>
  class PlaneMeasurementPlugin //:public omnimapper::MeasurementPlugin
  {
    typedef typename pcl::PointCloud<PointT> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;    
    
    public:
      PlaneMeasurementPlugin (omnimapper::OmniMapperBase* mapper);

      /** \brief regionsToMeasurements converts a set of planar regions as extracted by PCL's organized segmentation tools into a set of Planar landmark measurements suitable for use with the OmniMapper. */
      std::vector<gtsam::Plane> regionsToMeasurements (std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >&);
      
      /** \brief planarRegionCallback receives segmented data from the segmentation. */
      void planarRegionCallback (std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >& regions);
  };
  
}
