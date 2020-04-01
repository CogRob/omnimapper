#include <omnimapper/BoundedPlane3.h>
#include <omnimapper/BoundedPlaneFactor.h>
#include <omnimapper/get_transform_functor.h>
#include <omnimapper/omnimapper_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/planar_region.h>

namespace omnimapper {
template <typename PointT>
class BoundedPlanePlugin {
  typedef typename pcl::PointCloud<PointT> Cloud;
  typedef typename Cloud::Ptr CloudPtr;
  typedef typename Cloud::ConstPtr CloudConstPtr;

 public:
  BoundedPlanePlugin(omnimapper::OmniMapperBase* mapper);

  /** \brief regionsToMeasurements converts a set of planar regions as extracted
   * by PCL's organized segmentation tools into a set of Planar landmark
   * measurements suitable for use with the OmniMapper. */
  void RegionsToMeasurements(
      const std::vector<pcl::PlanarRegion<PointT>,
                        Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >&
          regions,
      const omnimapper::Time& t,
      std::vector<omnimapper::BoundedPlane3<PointT> >* plane_measurements);

  static void RemoveDuplicatePoints(pcl::PointCloud<PointT>* boundary_cloud);

  static bool PolygonsOverlap(CloudConstPtr boundary1, CloudConstPtr boundary2);

  static bool PolygonsOverlapBoost(Eigen::Vector4d& coeffs1,
                                   CloudConstPtr boundary1,
                                   Eigen::Vector4d& coeffs2,
                                   CloudConstPtr boundary2);

  /** \brief planarRegionCallback receives segmented data from the segmentation.
   */
  void PlanarRegionCallback(
      const std::vector<pcl::PlanarRegion<PointT>,
                        Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >&
          regions,
      const omnimapper::Time& t);

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

  void SetSensorToBaseFunctor(
      omnimapper::GetTransformFunctorPtr get_transform) {
    get_sensor_to_base_ = get_transform;
  }

  void SetDebug(bool debug) { debug_ = debug; }

 protected:
  OmniMapperBase* mapper_;
  bool debug_;
  GetTransformFunctorPtr get_sensor_to_base_;
  int max_plane_id_;
  double angular_threshold_;
  double range_threshold_;
  double angular_noise_;
  double range_noise_;
};
}  // namespace omnimapper
