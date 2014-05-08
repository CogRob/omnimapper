#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/planar_region.h>
#include <omnimapper/omnimapper_base.h>
#include <omnimapper/BoundedPlane3.h>
#include <omnimapper/BoundedPlaneFactor.h>
#include <omnimapper/get_transform_functor.h>

namespace omnimapper
{
  template <typename PointT>
  class BoundedPlanePlugin
  {
    typedef typename pcl::PointCloud<PointT> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;    

    public:
      BoundedPlanePlugin (omnimapper::OmniMapperBase* mapper);
      
      /** \brief regionsToMeasurements converts a set of planar regions as extracted by PCL's organized segmentation tools into a set of Planar landmark measurements suitable for use with the OmniMapper. */
      void regionsToMeasurements (std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >& regions, omnimapper::Time t, std::vector<omnimapper::BoundedPlane3<PointT> >& plane_measurements);

      bool polygonsOverlap (CloudPtr boundary1, CloudPtr boundary2);

      /** \brief planarRegionCallback receives segmented data from the segmentation. */
      void planarRegionCallback (std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions, omnimapper::Time t);

      /** \brief setAngularThreshold sets the angular threshold to be used for data association. */
      void setAngularThreshold (double angular_threshold) { angular_threshold_ = angular_threshold; }
     
      /** \brief setRangeThreshold sets the range threshold to be used for data association. */
      void setRangeThreshold (double range_threshold) { range_threshold_ = range_threshold; }

      void setAngularNoise (double angular_noise) { angular_noise_ = angular_noise; }

      void setRangeNoise (double range_noise) { range_noise_ = range_noise; }

      void setSensorToBaseFunctor (omnimapper::GetTransformFunctorPtr get_transform) { get_sensor_to_base_ = get_transform; }

    protected:
      OmniMapperBase* mapper_;
      GetTransformFunctorPtr get_sensor_to_base_;
      int max_plane_id_;
      double angular_threshold_;
      double range_threshold_;
      double angular_noise_;
      double range_noise_;
  };
}
