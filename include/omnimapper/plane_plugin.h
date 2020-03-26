#include <omnimapper/get_transform_functor.h>
#include <omnimapper/omnimapper_base.h>
#include <omnimapper/organized_feature_extraction.h>
#include <omnimapper/plane.h>
#include <omnimapper/plane_factor.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/planar_region.h>

namespace omnimapper {
/** \brief PlaneMeasurementPlugin adds factors for planar landmarks extracted
 * from 3D point cloud data. \author Alex Trevor
 */
template <typename PointT>
class PlaneMeasurementPlugin  //:public omnimapper::MeasurementPlugin
{
  typedef typename pcl::PointCloud<PointT> Cloud;
  typedef typename Cloud::Ptr CloudPtr;
  typedef typename Cloud::ConstPtr CloudConstPtr;

 public:
  PlaneMeasurementPlugin(omnimapper::OmniMapperBase* mapper);

  /** \brief regionsToMeasurements converts a set of planar regions as extracted
   * by PCL's organized segmentation tools into a set of Planar landmark
   * measurements suitable for use with the OmniMapper. */
  void RegionsToMeasurements(
      std::vector<pcl::PlanarRegion<PointT>,
                  Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >&
          regions,
      omnimapper::Time t,
      std::vector<gtsam::Plane<PointT> >& plane_measurements);

  /** \brief polygonsOverlapCloud tests if planar boundaries have some overlap
   * or not.  TODO: this could be much more efficient. */
  bool PolygonsOverlap(Cloud& boundary1, Cloud& boundary2);

  /** \brief planarRegionCallback receives segmented data from the segmentation.
   */
  void PlanarRegionCallback(
      std::vector<pcl::PlanarRegion<PointT>,
                  Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >
          regions,
      omnimapper::Time t);

  /** \brief setAngularThreshold sets the angular threshold to be used for data
   * association. */
  void SetAngularThreshold(double angular_threshold) {
    angular_threshold_ = angular_threshold;
  }

  /** \brief setRangeThreshold sets the range threshold to be used for data
   * association. */
  void SetRangeThreshold(double range_threshold) {
    range_threshold_ = range_threshold;
  }

  void SetAngularNoise(double angular_noise) { angular_noise_ = angular_noise; }

  void SetRangeNoise(double range_noise) { range_noise_ = range_noise; }

  void SetOverwriteTimestamps(bool overwrite_timestamps) {
    overwrite_timestamps_ = overwrite_timestamps;
  }

  void SetDisableDataAssociation(bool disable_da) {
    disable_data_association_ = disable_da;
  }

  void SetSensorToBaseFunctor(
      omnimapper::GetTransformFunctorPtr get_transform) {
    get_sensor_to_base_ = get_transform;
  }

  void Spin();

 protected:
  OmniMapperBase* mapper_;
  GetTransformFunctorPtr get_sensor_to_base_;
  int max_plane_id_;
  double angular_threshold_;
  double range_threshold_;
  double angular_noise_;
  double range_noise_;
  bool overwrite_timestamps_;
  bool disable_data_association_;
  std::vector<pcl::PlanarRegion<PointT>,
              Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >
      prev_regions_;
  omnimapper::Time prev_time_;
  bool updated_;
  boost::condition_variable updated_cond_;
  boost::mutex data_mutex_;
};

}  // namespace omnimapper
