/*
 * Software License Agreement (BSD License)
 *
 *  OmniMapper
 *  Copyright (c) 2012-, Georgia Tech Research Corporation,
 *  Atlanta, Georgia 30332-0415
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <cpu_tsdf/marching_cubes_tsdf_octree.h>
#include <cpu_tsdf/tsdf_volume_octree.h>
#include <omnimapper/tsdf_output_plugin.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

template <typename PointT>
omnimapper::TSDFOutputPlugin<PointT>::TSDFOutputPlugin(
    omnimapper::OmniMapperBase* mapper)
    : mapper_(mapper) {}

template <typename PointT>
void omnimapper::TSDFOutputPlugin<PointT>::Update(
    boost::shared_ptr<gtsam::Values>& vis_values,
    boost::shared_ptr<gtsam::NonlinearFactorGraph>& vis_graph) {
  printf("tsdf_plugin: updating latest solution!\n");

  latest_solution_ = vis_values;
  std::cout << "TSDFPlugin Updated " << std::endl;
}

template <typename PointT>
void omnimapper::TSDFOutputPlugin<PointT>::GenerateTSDF(double grid_size,
                                                        int resolution) {
  printf("tsdf_plugin: starting generateTSDF\n");
  // Make a TSDF
  cpu_tsdf::TSDFVolumeOctree::Ptr tsdf(new cpu_tsdf::TSDFVolumeOctree);
  printf("tsdf_plugin: created TSDFVolumeOctree\n");
  tsdf->setGridSize(grid_size, grid_size, grid_size);  // 10m x 10m x 10m
  // tsdf->setGridSize (30.0, 30.0, 30.0);
  // tsdf->setGridSize (3.0, 3.0, 3.0);
  // tsdf->setResolution (1024, 1024, 1024);
  tsdf->setResolution(
      resolution, resolution,
      resolution);  // Smallest sell cize = 10m / 2048 = about half a centimeter
  // tsdf->setResolution (4096, 4096, 4096);
  // tsdf->setGridSize (3000, 3000, 3000);
  Eigen::Affine3d tsdf_center =
      Eigen::Affine3d::Identity();  // Optionally offset the center
  tsdf->setGlobalTransform(tsdf_center);
  // tsdf->setDepthTruncationLimits ();
  // tsdf->setDepthTruncationLimits (0.3, 5.0);
  // tsdf->setWeightTruncationLimit (100.0);
  tsdf->setIntegrateColor(true);
  tsdf->reset();  // Initialize it to be empty

  printf("tsdf_plugin: initialized to empty\n");

  printf("tsdf_plugin: getting poses\n");
  gtsam::Values::ConstFiltered<gtsam::Pose3> pose_filtered =
      latest_solution_->filter<gtsam::Pose3>();
  // gtsam::Values::ConstFiltered<gtsam::Pose3> pose_filtered =
  // current_solution.filter<gtsam::Pose3>();

  printf("tsdf_plugin: solution has: %zu\n", pose_filtered.size());

  // Debug
  CloudPtr aggregate_cloud(new Cloud());
  // end debug

  int pose_num = 0;
  BOOST_FOREACH (
      const gtsam::Values::ConstFiltered<gtsam::Pose3>::KeyValuePair& key_value,
      pose_filtered) {
    gtsam::Symbol key_symbol(key_value.key);
    gtsam::Pose3 sam_pose = key_value.value;

    CloudConstPtr frame_cloud = icp_plugin_->getFullResCloudPtr(key_symbol);
    // CloudConstPtr frame_cloud = icp_plugin_->getCloudPtr (key_symbol);
    printf("TSDFPlugin: Cloud has %zu points\n", frame_cloud->points.size());
    if (frame_cloud->points.size() == 0) {
      // check the other
      CloudConstPtr test_cloud = icp_plugin_->getCloudPtr(key_symbol);
      printf("TSDFPlugin: Test Cloud has %zu, full cloud has %zu\n",
             test_cloud->points.size(), frame_cloud->points.size());
    }
    CloudPtr map_cloud(new Cloud());
    Eigen::Matrix4f map_tform = sam_pose.matrix().cast<float>();
    Eigen::Affine3d tform;
    gtsam::Quaternion sam_quat = sam_pose.rotation().toQuaternion();
    tform = Eigen::Quaterniond(sam_quat.w(), sam_quat.x(), sam_quat.y(),
                               sam_quat.z());
    tform.translation() =
        Eigen::Vector3d(sam_pose.x(), sam_pose.y(), sam_pose.z());
    Eigen::Affine3d tform_inv = tform.inverse();
    // pcl::transformPointCloud (*frame_cloud, *map_cloud, map_tform);
    // Eigen::Affine3d tform
    // (Eigen::Quaterniond(sam_quat[0],sam_quat[1],sam_quat[2],sam_quat[3]),
    // Eigen::Vector3d (sam_pose.x (), sam_pose.y (), sam_pose.z ()));

    Eigen::Affine3d sensor_to_base =
        icp_plugin_->getSensorToBaseAtSymbol(key_symbol);
    Eigen::Affine3d base_to_sensor;
    base_to_sensor = sensor_to_base.inverse();

    Eigen::Affine3d sensor_pose;
    sensor_pose =
        tform * sensor_to_base;  // tform * sensor_to_base;//base_to_sensor;
    Eigen::Affine3d sensor_pose_inv;
    sensor_pose_inv = sensor_pose.inverse();

    pcl::PointCloud<pcl::Normal> empty_normals;
    pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
    ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(20.0f);
    ne.setDepthDependentSmoothing(true);
    // ne.setRadiusSearch (0.1);
    // ne.setInputCloud (frame_cloud);
    // ne.compute (empty_normals);
    // empty_normals.resize (frame_cloud->points.size ());

    printf("Cloud has: %zu normals has: %zu\n", frame_cloud->points.size(),
           empty_normals.points.size());

    if (frame_cloud->points.size() > 0) {
      printf("tsdf_plugin: Adding cloud %d...\n", ++pose_num);
      // Debug
      CloudPtr map_cloud(new Cloud());
      pcl::transformPointCloud(*frame_cloud, *map_cloud,
                               sensor_pose);  // tform);
      (*aggregate_cloud) += (*map_cloud);
      // end debug

      tsdf->integrateCloud<pcl::PointXYZRGBA, pcl::Normal>(
          *frame_cloud, empty_normals,
          sensor_pose);  // tform); // Integrate the cloud
    }

    // Note, the normals aren't being used in the default settings. Feel free to
    // pass in an empty cloud
  }
  tsdf->save("/home/atrevor/Desktop/output.vol");  // Save it?

  // debug
  pcl::io::savePCDFileBinaryCompressed(
      "/home/atrevor/Desktop/aggregate_cloud.pcd", *aggregate_cloud);
  // end debug

  // Maching Cubes
  cpu_tsdf::MarchingCubesTSDFOctree mc;
  mc.setInputTSDF(tsdf);
  mc.setColorByConfidence(false);
  mc.setColorByRGB(false);
  mc.setMinWeight(0.01);
  pcl::PolygonMesh mesh;
  mc.reconstruct(mesh);
  pcl::io::savePLYFileBinary("/home/atrevor/Desktop/mesh.ply", mesh);

  // Render from xo
  Eigen::Affine3d init_pose = Eigen::Affine3d::Identity();
  pcl::PointCloud<pcl::PointNormal>::Ptr raytraced =
      tsdf->renderView(init_pose);
  pcl::io::savePCDFileBinary("/home/atrevor/Desktop/rendered_view.pcd",
                             *raytraced);
}

template class omnimapper::TSDFOutputPlugin<pcl::PointXYZRGBA>;
