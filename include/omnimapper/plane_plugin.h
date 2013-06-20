#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/planar_region.h>
#include <omnimapper/omnimapper_base.h>
#include <omnimapper/organized_feature_extraction.h>
#include <omnimapper/plane.h>
#include <omnimapper/plane_factor.h>

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
      void regionsToMeasurements (std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >& regions, std::vector<gtsam::Plane<PointT> >& plane_measurements);
      
      /** \brief polygonsOverlapCloud tests if planar boundaries have some overlap or not.  TODO: this could be much more efficient. */
      bool polygonsOverlap (Cloud& boundary1, Cloud& boundary2);

      /** \brief planarRegionCallback receives segmented data from the segmentation. */
      void planarRegionCallback (std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >& regions, omnimapper::Time& t);

      /** \brief setAngularThreshold sets the angular threshold to be used for data association. */
      void setAngularThreshold (double angular_threshold) { angular_threshold_ = angular_threshold; }
     
      /** \brief setRangeThreshold sets the range threshold to be used for data association. */
      void setRangeThreshold (double range_threshold) { range_threshold_ = range_threshold; }

      void setAngularNoise (double angular_noise) { angular_noise_ = angular_noise; }

      void setRangeNoise (double range_noise) { range_noise_ = range_noise; }
        
      void setOverwriteTimestamps (bool overwrite_timestamps) { overwrite_timestamps_ = overwrite_timestamps; }

      void setDisableDataAssociation (bool disable_da) { disable_data_association_ = disable_da; }
      
    protected:
      OmniMapperBase* mapper_;
      int max_plane_id_;
      double angular_threshold_;
      double range_threshold_;
      double angular_noise_;
      double range_noise_;
      bool overwrite_timestamps_;
      bool disable_data_association_;
  };
  
}
