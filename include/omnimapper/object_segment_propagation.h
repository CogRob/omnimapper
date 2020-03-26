#pragma once

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
// PCL
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
// GTSAM
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Symbol.h>

#define NUM_PARAMS 13
#define SEGMENT_SIZE 500
#define OPTIMAL_CLOUD 0
template <typename PointT>
class SegmentPropagation {
 public:
  SegmentPropagation();
  typedef typename pcl::PointCloud<PointT> Cloud;
  typedef typename Cloud::Ptr CloudPtr;
  typedef typename Cloud::ConstPtr CloudConstPtr;
  typedef typename std::vector<CloudPtr> CloudPtrVector;

  std::vector<pcl::PointIndices> PropogateLabels(
      std::vector<pcl::PointIndices> prev_label_indices_,
      std::vector<pcl::PointIndices> curr_label_indices,
      CloudConstPtr prev_cloud_ptr_, CloudConstPtr curr_cloud_ptr_);

  std::vector<pcl::PointIndices> propogateLabels(
      std::vector<pcl::PointIndices> curr_label_indices,
      CloudConstPtr curr_cloud_ptr_) {
    std::vector<pcl::PointIndices> final_labels;
    if (prev_labels_.size() > 0) {
      final_labels = propogateLabels(prev_labels_, curr_label_indices,
                                     prev_cloud_, curr_cloud_ptr_);
      prev_labels_ = final_labels;
      prev_cloud_ = curr_cloud_ptr_;

    } else {
      PCL_INFO(
          "Previous Cloud is Empty!! Saving the current cloud for future "
          "propagation\n");

      prev_labels_ = curr_label_indices;
      prev_cloud_ = curr_cloud_ptr_;
    }

    return final_labels;
  }

  pcl::PointCloud<pcl::Label> propogateLabels(
      pcl::PointCloud<pcl::Label> labels, CloudConstPtr curr_cloud_ptr_) {
    pcl::Label invalid_pt;
    invalid_pt.label = std::numeric_limits<unsigned>::max();

    std::vector<pcl::PointIndices> label_indices_, final_label_indices_;
    label_indices_.resize(labels.points.size());

    for (std::size_t i = 0; i < labels.points.size(); i++) {
      if (labels.points[i].label > labels.points.size()) continue;
      label_indices_[labels.points[i].label].indices.push_back(i);
    }
    final_label_indices_ = propogateLabels(label_indices_, curr_cloud_ptr_);

    pcl::PointCloud<pcl::Label> final_labels;

    if (final_label_indices_.size() == 0) return final_labels;

    final_labels.points.resize(labels.points.size(), invalid_pt);
    final_labels.width = labels.width;
    final_labels.height = labels.height;

    for (std::size_t i = 0; i < final_label_indices_.size(); i++) {
      for (std::size_t j = 0; j < final_label_indices_[i].indices.size(); j++) {
        int pt = final_label_indices_[i].indices[j];
        if (labels.points[pt].label > labels.points.size()) continue;
        final_labels.points[pt].label = i;
      }
    }

    return final_labels;
  }

  CloudPtrVector propagateLabels(CloudPtrVector prev_label_indices_,
                                 CloudPtrVector curr_label_indices_);

  CloudPtrVector propagateLabels(CloudPtrVector label) {
    CloudPtrVector final_label;

    if (prev_cloud_vec_.size() == 0 || restart_flag_) {
      if (restart_flag_) {
        std::cout << "Restarting temporal propagation..." << std::endl;
        restart_flag_ = 0;
      }
      std::cout << "[initializing] PrevCloudPtrVector is empty:  "
                << prev_cloud_vec_.size()
                << " CurrentCloudPtrVector size: " << label.size();
      std::cout << std::endl;
      final_label = InitializeLabel(label);
      prev_cloud_vec_ = final_label;

      std::cout << "[after initializing] Size of prev_cloud_vec_: "
                << prev_cloud_vec_.size();
      std::cout << std::endl;

      // prev_pose_ = pose_;
    } else {
      std::cout << "[propagating labels] PrevCloudPtrVector is empty:  "
                << prev_cloud_vec_.size()
                << " CurrentCloudPtrVector size: " << label.size();
      std::cout << std::endl;
      ;

      final_label = propagateLabels(prev_cloud_vec_, label);
      prev_cloud_vec_ = final_label;
    }

    return final_label;
  }

  CloudPtrVector propagateLabels(CloudPtrVector label, gtsam::Pose3 pose_,
                                 gtsam::Symbol sym);
  CloudPtrVector PredictLabels(CloudPtrVector label, gtsam::Pose3 pose_,
                               gtsam::Symbol sym);

  std::vector<pcl::PointIndices> ConvertToPointIndices(
      std::vector<pcl::PointIndices> current_label_indices,
      CloudPtrVector label) {
    std::vector<pcl::PointIndices> final_label_indices;
    final_label_indices =
        findFinalLabels(final_neighbors_, current_label_indices);

    return final_label_indices;
  }

  std::vector<pcl::PointIndices> FindFinalLabels(
      std::vector<int> neighbor_vec,
      std::vector<pcl::PointIndices> euclidean_label_indices);

  CloudPtrVector findFinalLabels(std::vector<int> neighbor_vec,
                                 CloudPtrVector euclidean_label_indices);

  std::vector<int> TwoWayMatch(
      std::vector<float> prev_centroids, std::vector<float> curr_centroids,
      std::vector<int> neigbhor_vec,
      std::map<int, std::vector<int> > rev_neighbor_vec, int label_size);

  std::vector<int> LinearMatch(std::vector<float> curr_centroids,
                               std::vector<float> prev_centroids);

  std::vector<float> ComputeCentroids(
      std::vector<pcl::PointIndices> label_indices, CloudConstPtr cloud_ptr);

  std::vector<float> computeCentroids(CloudPtrVector label);

  void SetActiveLabelIndices(int size);
  CloudPtrVector InitializeLabel(CloudPtrVector label);
  CloudPtrVector CopyLabel(CloudPtrVector label, CloudPtrVector final_label);

  float ComputeJaccardIndex(std::vector<float> curr_centroids,
                            std::vector<float> prev_centroids, float idx,
                            float nn_idx);

  CloudPtrVector CreateFinalCloud(gtsam::Symbol sym, gtsam::Pose3 pose);

  bool restart_flag_;  // initialize new labels -- time gap between previous and
                       // current frame is large
  CloudPtrVector final_cloud_vec_;
  std::map<int, int> final_count_;
  std::map<gtsam::Symbol, CloudPtrVector> observations_;
  std::map<int, int> back_label_;
  CloudPtrVector final_map_cloud_;

 protected:
  bool verbose_, debug_;
  int max_size_;
  CloudConstPtr prev_cloud_;
  CloudPtrVector prev_cloud_vec_;
  gtsam::Pose3 prev_pose_;
  std::vector<pcl::PointIndices> prev_labels_;
  std::map<int, int> active_label_indices_;
  std::vector<int> final_neighbors_;
};

/* SEGMENT_PROPAGATION_H_ */
