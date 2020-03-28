#include <glog/logging.h>
#include <omnimapper/bounded_plane_plugin.h>
#include <omnimapper/geometry.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>

namespace omnimapper {
template <typename PointT>
BoundedPlanePlugin<PointT>::BoundedPlanePlugin(
    omnimapper::OmniMapperBase* mapper)
    : mapper_(mapper), max_plane_id_(0) {
  LOG(INFO) << "Constructing BoundedPlanePlugin";
}

template <typename PointT>
static bool BoundedPlanePlugin<PointT>::PolygonsOverlap(CloudPtr boundary1,
                                                        CloudPtr boundary2) {
  for (const auto& point : boundary1->points) {
    if (pcl::isPointIn2DPolygon(point, *boundary2)) return true;
  }
  for (const auto& point : boundary2->points) {
    if (pcl::isPointIn2DPolygon(point, *boundary1)) return true;
  }
  return false;
}

template <typename PointT>
static bool BoundedPlanePlugin<PointT>::PolygonsOverlapBoost(
    Eigen::Vector4d& coeffs1, CloudPtr boundary1, Eigen::Vector4d& coeffs2,
    CloudPtr boundary2) {
  const Eigen::Vector4d z_axis(0.0, 0.0, 1.0, 0.0);
  const Eigen::Vector4d minus_z_axis(0.0, 0.0, -1.0, 0.0);

  // Move to xy
  Eigen::Affine3d p1_to_xy = PlanarAlignmentTransform(z_axis, coeffs1);
  CloudPtr p1_xy(new Cloud());
  pcl::transformPointCloud(*boundary1, *p1_xy, p1_to_xy);

  Eigen::Affine3d p2_to_xy = PlanarAlignmentTransform(z_axis, coeffs2);
  CloudPtr p2_xy(new Cloud());
  pcl::transformPointCloud(*boundary2, *p2_xy, p2_to_xy);

  bool intersects = boost::geometry::intersects(p1_xy->points, p2_xy->points);
  return intersects;
}

template <typename PointT>
static void BoundedPlanePlugin<PointT>::RemoveDuplicatePoints(
    pcl::PointCloud<PointT>* boundary_cloud) {
  for (std::size_t i = 1; i < boundary_cloud->points.size() - 1; i++) {
    for (std::size_t j = i + 1; j < boundary_cloud->points.size() - 1; j++) {
      if ((boundary_cloud->points[i].x == boundary_cloud->points[j].x) &&
          (boundary_cloud->points[i].y == boundary_cloud->points[j].y)) {
        if (j - i > (boundary_cloud->points.size() / 2.0)) {
          boundary_cloud->points.erase(boundary_cloud->points.begin() + j,
                                       boundary_cloud->points.end());
          boundary_cloud->points.erase(boundary_cloud->points.begin(),
                                       boundary_cloud->points.begin() + i);
          i = 1;
          j = i + 1;
        } else {
          boundary_cloud->points.erase(boundary_cloud->points.begin() + i,
                                       boundary_cloud->points.begin() + j);
          i = 1;
          j = i + 1;
        }
      }
    }
  }
}

template <typename PointT>
void BoundedPlanePlugin<PointT>::RegionsToMeasurements(
    const std::vector<pcl::PlanarRegion<PointT>,
                      Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >&
        regions,
    const omnimapper::Time& t,
    std::vector<omnimapper::BoundedPlane3<PointT> >* plane_measurements) {
  // If we have a sensor_to_base_transform function, use it and apply the
  // transform to incoming measurements
  Eigen::Affine3d sensor_to_base;
  bool use_transform = false;
  if (get_sensor_to_base_) {
    use_transform = true;
    sensor_to_base = (*get_sensor_to_base_)(t);
  }

  for (size_t i = 0; i < regions.size(); ++i) {
    Eigen::Vector4f model = regions[i].getCoefficients();
    Eigen::Vector3f centroid = regions[i].getCentroid();
    Eigen::Vector4f centroid4f(centroid[0], centroid[1], centroid[2], 0.0);
    CloudPtr border_cloud(new Cloud());
    border_cloud->points = regions[i].getContour();

    const std::size_t border_size_before_remove = border_cloud->points.size();
    RemoveDuplicatePoints(border_cloud.get());
    LOG(INFO) << "Removed points from border, size before: "
              << border_size_before_remove
              << ", after: " << border_cloud->points.size();

    const bool intersects = boost::geometry::intersects(border_cloud->points);
    if (intersects) {
      LOG(ERROR) << "BoundedPlanePlugin got invalid measurement.";
      // TODO: remove debug
      const std::string meas_name = "/tmp/invalid_meas.pcd";
      pcl::io::savePCDFileBinaryCompressed(meas_name, *border_cloud);
      LOG(FATAL) << "Saved measurement cloud to " << meas_name;
    } else {
      LOG(INFO) << "BoundedPlanePlugin got valid measurement.";
    }

    // Make a Plane.
    if (use_transform) {
      Eigen::Vector3d trans = sensor_to_base.inverse().translation();
      Eigen::Vector4f centroid4f_base =
          sensor_to_base.cast<float>() * centroid4f;
      Eigen::Vector3d model_norm(model[0], model[1], model[2]);
      Eigen::Vector3d model_norm_rot = sensor_to_base.linear() * model_norm;
      double d = model[0] * trans[0] + model[1] * trans[1] +
                 model[2] * trans[2] + model[3];
      Eigen::Vector4f model_base(model_norm_rot[0], model_norm_rot[1],
                                 model_norm_rot[2], d);

      Eigen::Vector4f vp = -centroid4f_base;
      float cos_theta = vp.dot(model_base);
      if (cos_theta < 0) {
        model_base *= -1;
        model_base[3] = 0;
        model_base[3] = -1 * model_base.dot(centroid4f_base);
      }

      LOG(INFO) << "Model: " << model[0] << " " << model[1] << " " << model[2]
                << " " << model[3] << "; Base model: " << model_base[0] << " "
                << model_base[1] << " " << model_base[2] << " "
                << model_base[3];
      CloudPtr border_base(new Cloud());
      pcl::transformPointCloud(*border_cloud, *border_base, sensor_to_base);
      omnimapper::BoundedPlane3<PointT> plane(model_base[0], model_base[1],
                                              model_base[2], model_base[3],
                                              border_base);
      plane_measurements->push_back(plane);
    } else {
      omnimapper::BoundedPlane3<PointT> plane(model[0], model[1], model[2],
                                              model[3], border_cloud);
      plane_measurements->push_back(plane);
    }
  }
}

template <typename PointT>
void BoundedPlanePlugin<PointT>::PlanarRegionCallback(
    const std::vector<pcl::PlanarRegion<PointT>,
                      Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >&
        regions,
    const omnimapper::Time& t) {
  LOG(INFO) << "BoundedPlanePlugin got " << regions.size() << " regions.";

  // Convert the regions to omnimapper::BoundedPlane3.
  std::vector<omnimapper::BoundedPlane3<PointT> > plane_measurements;
  RegionsToMeasurements(regions, t, &plane_measurements);
  LOG(INFO) << "The regoins have " << plane_measurements.size()
            << " measurements.";

  // Get the planes from the mapper.
  gtsam::Values::Filtered<omnimapper::BoundedPlane3<PointT> > plane_filtered =
      mapper_->GetSolutionAndUncommitted()
          .filter<omnimapper::BoundedPlane3<PointT> >();

  // Get the pose symbol for this time.
  gtsam::Symbol pose_sym;
  mapper_->GetPoseSymbolAtTime(t, &pose_sym);
  boost::optional<gtsam::Pose3> new_pose = mapper_->PredictPose(pose_sym);
  // If above didn't work.
  if (!new_pose) {
    LOG(ERROR) << "No pose available at this time.";
    return;
  }
  LOG(INFO) << "Processing planes for pose " << std::string(pose_sym);

  gtsam::Pose3 new_pose_inv = new_pose->inverse();
  Eigen::Matrix4f new_pose_inv_tform = new_pose->matrix().cast<float>();

  for (const auto& meas_plane : plane_measurements) {
    double lowest_error = std::numeric_limits<double>::infinity();
    gtsam::Symbol best_symbol = gtsam::Symbol('p', max_plane_id_);

    Eigen::Vector3d meas_norm = meas_plane.normal().point3().vector();
    double meas_d = meas_plane.d();
    CloudPtr meas_boundary = meas_plane.boundary();
    CloudPtr meas_boundary_map(new Cloud());
    Eigen::Affine3f pose2map = Pose3ToTransform(*new_pose);
    pcl::transformPointCloud(*meas_boundary, *meas_boundary_map, pose2map);

    Eigen::Vector4d meas_map_coeffs =
        omnimapper::BoundedPlane3<PointT>::TransformCoefficients(meas_plane,
                                                                 new_pose_inv);
    for (const auto& point : meas_boundary_map->points) {
      LOG(INFO) << "Meas Boundary Map: Boundary Point: (" << point.x << ", "
                << point.y << ", " << point.z << ")";
      const double ptp_dist =
          fabs(meas_map_coeffs[0] * point.x + meas_map_coeffs[1] * point.y +
               meas_map_coeffs[2] * point.z + meas_map_coeffs[3]);
      if (ptp_dist > 0.01) {
        LOG(FATAL) << "ERROR: Initializing boundary at bad place: "
                   << "Point is " << ptp_dist << "from plane.";
      }
    }

    bool intersects = boost::geometry::intersects(meas_boundary_map->points);
    if (intersects) {
      // TODO(shengye): Is this fatal?
      LOG(ERROR) << "BoundedPlanePlugin got invalid measurement, map_meas";
    }

    if (meas_d < 0.1) continue;
    Eigen::Vector3d ceiling_norm(0.0, 0.0, -1.0);
    if (acos(meas_norm.dot(ceiling_norm)) < angular_threshold_) continue;

    for (const typename gtsam::Values::Filtered<
             omnimapper::BoundedPlane3<PointT> >::KeyValuePair& key_value :
         plane_filtered) {
      gtsam::Symbol key_symbol(key_value.key);
      const omnimapper::BoundedPlane3<PointT> plane = key_value.value;
      Eigen::Vector4d pred_coeffs =
          omnimapper::BoundedPlane3<PointT>::TransformCoefficients(plane,
                                                                   (*new_pose));
      const Eigen::Vector3d pred_norm(pred_coeffs[0], pred_coeffs[1],
                                      pred_coeffs[2]);
      const double pred_d = pred_coeffs[3];

      const double angular_error = acos(meas_norm.dot(pred_norm));
      const double range_error = fabs(meas_d - pred_d);
      LOG(INFO) << "angular_error = " << angular_error << ", "
                << "range_error = " << range_error;
      if ((angular_error < angular_threshold_) &&
          (range_error < range_threshold_)) {
        double error = angular_error + range_error;
        // Polygon overlap.
        // TODO: this should not be in the map frame due to lever-arm.
        CloudPtr match_map_boundary = plane.boundary();
        Eigen::Vector4d match_map_coeffs = plane.planeCoefficients();
        if (PolygonsOverlapBoost(match_map_coeffs, match_map_boundary,
                                 meas_map_coeffs, meas_boundary_map)) {
          if ((error < lowest_error)) {
            lowest_error = error;
            best_symbol = key_symbol;
          }
        } else {
          LOG(ERROR) << "Polygon overlap failed.";
        }
      }
    }

    // Add factors.
    if (lowest_error == std::numeric_limits<double>::infinity()) {
      Eigen::Vector4d map_p3_coeffs =
          omnimapper::BoundedPlane3<PointT>::TransformCoefficients(
              meas_plane, new_pose_inv);
      omnimapper::BoundedPlane3<PointT> map_plane(
          map_p3_coeffs[0], map_p3_coeffs[1], map_p3_coeffs[2],
          map_p3_coeffs[3], meas_boundary_map);
      mapper_->AddNewValue(best_symbol, map_plane);
      LOG(INFO) << "BoundedPlanePlugin created a new plane: "
                << std::string(best_symbol) << ", (" << map_p3_coeffs[0] << ","
                << map_p3_coeffs[1] << "," << map_p3_coeffs[2] << ","
                << map_p3_coeffs[3] << ")";
      ++max_plane_id_;
    } else {
      mapper_->UpdateBoundedPlane(best_symbol, *new_pose, &meas_plane);
      LOG(INFO) << "BoundedPlanePlugin extended plane: "
                << std::string(best_symbol);
    }

    gtsam::Vector measurement_noise_vector(3);
    measurement_noise_vector << angular_noise_, angular_noise_, range_noise_;
    gtsam::SharedDiagonal measurement_noise =
        gtsam::noiseModel::Diagonal::Sigmas(measurement_noise_vector);

    gtsam::Vector measurement_vector = meas_plane.planeCoefficients();
    omnimapper::OmniMapperBase::NonlinearFactorPtr plane_factor(
        new omnimapper::BoundedPlaneFactor<PointT>(
            measurement_vector, meas_boundary, measurement_noise, pose_sym,
            best_symbol));
    mapper_->AddFactor(plane_factor);
    LOG(INFO) << "BoundedPlanePlugin added a factor.";
  }  // plane measurements
}

}  // namespace omnimapper

template class omnimapper::BoundedPlanePlugin<pcl::PointXYZRGBA>;
