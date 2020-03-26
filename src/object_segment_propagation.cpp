#ifndef SEGMENT_PROPAGATION_IMPL_H_
#define SEGMENT_PROPAGATION_IMPL_H_

#include <omnimapper/object_segment_propagation.h>
#include <pcl/common/concatenate.h>
#include <pcl/common/io.h>

#include <cfloat>
#include <iostream>

#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))

template <typename PointT>
SegmentPropagation<PointT>::SegmentPropagation()
    : restart_flag_(false),
      final_cloud_vec_(),
      final_count_(),
      final_map_cloud_(),
      verbose_(true),
      debug_(true),
      max_size_(0),
      prev_cloud_vec_(),
      prev_labels_(),
      active_label_indices_() {
  if (debug_) {
    std::cout << "Inside the temporal segmentation cluster " << std::endl;
    std::cout << "Size of prev_cloud_vec " << prev_cloud_vec_.size()
              << std::endl;
    std::cout << "Size of final_cloud_vec " << final_cloud_vec_.size()
              << std::endl;
    std::cout << "Size of active label indices " << active_label_indices_.size()
              << std::endl;
    std::cout << "Size of frame count " << std::endl;
  }
}

template <typename PointT>
float SegmentPropagation<PointT>::ComputeJaccardIndex(
    std::vector<float> curr_centroids, std::vector<float> prev_centroids,
    float idx, float nn_idx) {
  // compute intersection of boxes
  float minX = curr_centroids[idx + 7];
  float minY = curr_centroids[idx + 8];
  float minZ = curr_centroids[idx + 9];
  float maxX = curr_centroids[idx + 10];
  float maxY = curr_centroids[idx + 11];
  float maxZ = curr_centroids[idx + 12];

  float minX1 = prev_centroids[nn_idx + 7];
  float minY1 = prev_centroids[nn_idx + 8];
  float minZ1 = prev_centroids[nn_idx + 9];
  float maxX1 = prev_centroids[nn_idx + 10];
  float maxY1 = prev_centroids[nn_idx + 11];
  float maxZ1 = prev_centroids[nn_idx + 12];

  float inter_area = MAX(MIN(maxX, maxX1) - MAX(minX, minX1), 0) *
                     MAX(MIN(maxY, maxY1) - MAX(minY, minY1), 0) *
                     MAX(MIN(maxZ, maxZ1) - MAX(minZ, minZ1), 0);

  float areaA = (maxX - minX) * (maxY - minY) * (maxZ - minZ);
  float areaB = (maxX1 - minX1) * (maxY1 - minY1) * (maxZ1 - minZ1);

  float jaccard_index = inter_area / (areaA + areaB - inter_area);

  std::cout << "[inside jaccard] " << inter_area << " " << areaA << " " << areaB
            << std::endl;
  return jaccard_index;
}

template <typename PointT>
typename SegmentPropagation<PointT>::CloudPtrVector
SegmentPropagation<PointT>::CopyLabel(CloudPtrVector label,
                                      CloudPtrVector final_label) {
  CloudPtrVector final_label_indices;
  back_label_.clear();

  int count = 0;
  std::cout << "Size of final_label" << final_label.size() << std::endl;
  for (std::size_t i = 0; i < final_label.size(); i++) {
    if (final_label[i]->points.size() != 0) {
      std::cout << "Final label " << final_label[i]->points.size() << std::endl;
      final_label_indices.push_back(label[count]);
      back_label_.insert(std::pair<int, int>(i, count));
      count++;

    } else {
      Cloud temp;
      std::cout << "Pushing into label" << std::endl;
      final_label_indices.push_back(temp.makeShared());
    }
  }
  return final_label_indices;
}

template <typename PointT>
typename SegmentPropagation<PointT>::CloudPtrVector
SegmentPropagation<PointT>::InitializeLabel(CloudPtrVector label) {
  std::cout << "Size of active label indices " << active_label_indices_.size()
            << std::endl;

  if (active_label_indices_.size() == 0) {
    for (std::size_t i = 0; i < label.size(); i++) {
      active_label_indices_[i] = 1;
    }
    return label;
  }
  std::map<int, int>::reverse_iterator it;
  it = active_label_indices_.rbegin();
  int active_size = it->first;
  CloudPtrVector final_label_indices;

  for (int i = 0; i < active_size + 1; i++) {
    Cloud temp;
    final_label_indices.push_back(temp.makeShared());
  }

  for (std::size_t i = 0; i < label.size(); i++) {
    final_label_indices.push_back(label[i]);
    active_label_indices_[i + active_size + 1] = 1;
  }

  return final_label_indices;
}

template <typename PointT>
std::vector<float> SegmentPropagation<PointT>::ComputeCentroids(
    std::vector<pcl::PointIndices> label_indices, CloudConstPtr cloud_ptr) {
  std::vector<float> centroids;

  // std::cout << "Size of label indices " << label_indices.size() << std::endl;

  for (std::size_t idx = 0; idx < label_indices.size(); idx++) {
    float meanX = -1, meanY = -1, meanZ = -1;
    float meanR = -1, meanG = -1, meanB = -1;

    /* find all segments greater than SEGMENT_SIZE */
    if (label_indices[idx].indices.size() > SEGMENT_SIZE &&
        label_indices[idx].indices.size() <
            cloud_ptr->height * cloud_ptr->width) {
      meanX = 0, meanY = 0, meanZ = 0;
      meanR = 0, meanG = 0, meanB = 0;

      if (debug_)
        std::cout << "Size of segment " << idx << " "
                  << label_indices[idx].indices.size() << std::endl;

      // compute euclidean distance centroid and color centroid
      for (std::size_t i = 0; i < label_indices[idx].indices.size(); i++) {
        int coord = label_indices[idx].indices[i];

        meanR += (float)cloud_ptr->points[coord].r;
        meanG += (float)cloud_ptr->points[coord].g;
        meanB += (float)cloud_ptr->points[coord].b;

        meanX += (float)cloud_ptr->points[coord].x;
        meanY += (float)cloud_ptr->points[coord].y;
        meanZ += (float)cloud_ptr->points[coord].z;
      }

      meanX = meanX * 1.0f / label_indices[idx].indices.size();
      meanY = meanY * 1.0f / label_indices[idx].indices.size();
      meanZ = meanZ * 1.0f / label_indices[idx].indices.size();
      meanR = meanR * 1.0f / label_indices[idx].indices.size();
      meanG = meanG * 1.0f / label_indices[idx].indices.size();
      meanB = meanB * 1.0f / label_indices[idx].indices.size();
    }

    centroids.push_back(meanX);
    centroids.push_back(meanY);
    centroids.push_back(meanZ);
    centroids.push_back(meanR);
    centroids.push_back(meanG);
    centroids.push_back(meanB);
    centroids.push_back((float)label_indices[idx].indices.size());
  }

  //	std::cout << "Size of centroids: " << centroids.size() << std::endl;

  return centroids;
}

template <typename PointT>
std::vector<float> SegmentPropagation<PointT>::computeCentroids(
    CloudPtrVector label) {
  std::vector<float> centroids;

  float tempX, tempY, tempZ;
  // std::cout << "Size of label indices " << label_indices.size() << std::endl;

  for (std::size_t idx = 0; idx < label.size(); idx++) {
    float meanX = -1, meanY = -1, meanZ = -1;
    float meanR = -1, meanG = -1, meanB = -1;
    float minX = FLT_MAX, minY = FLT_MAX, minZ = FLT_MAX;
    float maxX = FLT_MIN, maxY = FLT_MIN, maxZ = FLT_MIN;

    /* find all segments greater than SEGMENT_SIZE */
    if (label[idx]->points.size() > SEGMENT_SIZE) {
      meanX = 0, meanY = 0, meanZ = 0;
      meanR = 0, meanG = 0, meanB = 0;

      if (debug_)
        std::cout << "Size of segment " << idx << " "
                  << label[idx]->points.size() << std::endl;

      // compute euclidean distance centroid and color centroid
      for (std::size_t i = 0; i < label[idx]->points.size(); i++) {
        int coord = i;

        meanR += (float)label[idx]->points[coord].r;
        meanG += (float)label[idx]->points[coord].g;
        meanB += (float)label[idx]->points[coord].b;

        tempX = (float)label[idx]->points[coord].x;
        meanX += tempX;
        if (tempX > maxX) maxX = tempX;
        if (tempX < minX) minX = tempX;

        tempY = (float)label[idx]->points[coord].y;
        meanY += tempY;
        if (tempY > maxY) maxY = tempY;
        if (tempY < minY) minY = tempY;

        tempZ = (float)label[idx]->points[coord].z;
        meanZ += tempZ;
        if (tempZ > maxZ) maxZ = tempZ;
        if (tempZ < minZ) minZ = tempZ;
      }

      meanX = meanX * 1.0f / label[idx]->points.size();
      meanY = meanY * 1.0f / label[idx]->points.size();
      meanZ = meanZ * 1.0f / label[idx]->points.size();
      meanR = meanR * 1.0f / label[idx]->points.size();
      meanG = meanG * 1.0f / label[idx]->points.size();
      meanB = meanB * 1.0f / label[idx]->points.size();
    }

    centroids.push_back(meanX);
    centroids.push_back(meanY);
    centroids.push_back(meanZ);
    centroids.push_back(meanR);
    centroids.push_back(meanG);
    centroids.push_back(meanB);
    centroids.push_back((float)label[idx]->points.size());
    centroids.push_back(minX);
    centroids.push_back(minY);
    centroids.push_back(minZ);
    centroids.push_back(maxX);
    centroids.push_back(maxY);
    centroids.push_back(maxZ);
  }

  //	std::cout << "Size of centroids: " << centroids.size() << std::endl;

  return centroids;
}

template <typename PointT>
std::vector<int> SegmentPropagation<PointT>::LinearMatch(
    std::vector<float> curr_centroids, std::vector<float> prev_centroids) {
  int num_edges = curr_centroids.size() /
                  NUM_PARAMS;  // number of edges in bipartite matching graph

  std::vector<int> neighbor_vec;
  neighbor_vec.resize(num_edges, -1);

  for (size_t idx = 0; idx < curr_centroids.size(); idx += NUM_PARAMS) {
    if (curr_centroids[idx] == -1) continue;
    float min = 1000000000, min_diff_num = 0;
    int nn_index = -1;
    for (std::size_t i = 0; i < prev_centroids.size(); i += NUM_PARAMS) {
      if (prev_centroids[i] == -1) continue;
      float diffX = curr_centroids[idx] - prev_centroids[i];
      float diffY = curr_centroids[idx + 1] - prev_centroids[i + 1];
      float diffZ = curr_centroids[idx + 2] - prev_centroids[i + 2];
      float diffR = curr_centroids[idx + 3] - prev_centroids[i + 3];
      float diffG = curr_centroids[idx + 4] - prev_centroids[i + 4];
      float diffB = curr_centroids[idx + 5] - prev_centroids[i + 5];
      float diff_num = fabs(curr_centroids[idx + 6] - prev_centroids[i + 6]);
      float jaccard_index =
          ComputeJaccardIndex(curr_centroids, prev_centroids, idx, i);

      float diff_eq = sqrtf(diffX * diffX + diffY * diffY + diffZ * diffZ);
      // float diff_eq = -jaccard_index;

      float diff_color = 0;  // sqrtf(diffR*diffR + diffG*diffG + diffB*diffB);
      if (((diff_eq + diff_color) / 2) < min) {
        min = ((diff_eq + diff_color) / 2);
        nn_index = i / NUM_PARAMS;
        min_diff_num = diff_num;
      }
    }

    // rev_neighbor_vec
    neighbor_vec[idx / NUM_PARAMS] = nn_index;

    std::cout << "IDX: " << idx / NUM_PARAMS << " NN index " << nn_index
              << " diff num " << min << std::endl;

    // compute intersection of boxes
    float jaccard_index = ComputeJaccardIndex(curr_centroids, prev_centroids,
                                              idx, nn_index * NUM_PARAMS);

    std::cout << "jaccard_index: " << jaccard_index << std::endl;

    if (jaccard_index < 0.3 || min > 0.7) {
      std::cout << "Min: " << min << std::endl;
      std::cout << "jaccard_index: " << jaccard_index << std::endl;
      neighbor_vec[idx / NUM_PARAMS] = -2;
    }
  }

  return neighbor_vec;
  //    return std::pair
}

template <typename PointT>
std::vector<int> SegmentPropagation<PointT>::TwoWayMatch(
    std::vector<float> prev_centroids, std::vector<float> curr_centroids,
    std::vector<int> neighbor_vec,
    std::map<int, std::vector<int> > rev_neighbor_vec, int label_size) {
  // neighbor_vec[i];
  std::vector<int> final_neighbor_vec;
  final_neighbor_vec.resize(neighbor_vec.size(), -1);

  for (size_t idx = 0; idx < neighbor_vec.size(); idx++) {
    if (neighbor_vec[idx] != -1 &&
        neighbor_vec[idx] !=
            -2) {  // size of the segment is less than SEGMENT_SIZE
                   // std::cout << "Index: " << i << " Neighbor: " <<
                   // neighbor_vec[i] << " Size of rev NN: " <<
                   // rev_neighbor_vec[neighbor_vec[i]].size() << std::endl;
      int nn_index = neighbor_vec[idx];

      if (rev_neighbor_vec[nn_index].size() == 1) {
        final_neighbor_vec[idx] = nn_index;
        active_label_indices_[nn_index] = 1;
        if (debug_)
          std::cout << "(0)Neighbor of " << idx << " " << nn_index << std::endl;
        continue;
      }

      // find the NN of i among rev_neighbor_vec[neighbor_vec[i]]
      float nn_nn_index = -1;
      float min = 1000000000;
      for (std::size_t j = 0; j < rev_neighbor_vec[nn_index].size(); j++) {
        int idy = rev_neighbor_vec[nn_index][j];
        float diffX = curr_centroids[idy * NUM_PARAMS] -
                      prev_centroids[nn_index * NUM_PARAMS];
        float diffY = curr_centroids[idy * NUM_PARAMS + 1] -
                      prev_centroids[nn_index * NUM_PARAMS + 1];
        float diffZ = curr_centroids[idy * NUM_PARAMS + 2] -
                      prev_centroids[nn_index * NUM_PARAMS + 2];
        float diffR = curr_centroids[idy * NUM_PARAMS + 3] -
                      prev_centroids[nn_index * NUM_PARAMS + 3];
        float diffG = curr_centroids[idy * NUM_PARAMS + 4] -
                      prev_centroids[nn_index * NUM_PARAMS + 4];
        float diffB = curr_centroids[idy * NUM_PARAMS + 5] -
                      prev_centroids[nn_index * NUM_PARAMS + 5];

        float jaccard_index =
            ComputeJaccardIndex(curr_centroids, prev_centroids,
                                idy * NUM_PARAMS, nn_index * NUM_PARAMS);

        float diff_eq = sqrtf(diffX * diffX + diffY * diffY + diffZ * diffZ);

        // float diff_eq = -jaccard_index;
        float diff_color = 0;  // sqrtf(diffR*diffR + diffG*diffG +
                               // diffB*diffB);
        // std::cout << i << " " << prev_centroids[i] << " " << diff_eq << " "
        // << min << " " << nn_index << std::endl;
        if (debug_)
          std::cout << nn_index << " " << idy << " " << diff_eq << std::endl;
        if (((diff_eq + diff_color) / 2) < min) {
          min = ((diff_eq + diff_color) / 2);
          nn_nn_index = idy;
          //       std::cout << "Index: " << idx << " " << nn_index << " " <<
          //       diff_eq << "Prev Centroids " << prev_centroids[i] << " " <<
          //       prev_centroids[i+1] << " " << prev_centroids[i+2] <<
          //       std::endl;
        }
      }
      if (debug_)
        std::cout << "(1)Neighbor of " << nn_nn_index << " " << nn_index
                  << std::endl;
      final_neighbor_vec[nn_nn_index] = nn_index;
      neighbor_vec[nn_nn_index] = nn_index;
      active_label_indices_[nn_index] = 1;

      /* for everything else assign a new label */
      for (std::size_t j = 0; j < rev_neighbor_vec[nn_index].size(); j++) {
        int idy = rev_neighbor_vec[nn_index][j];
        if (idy != nn_nn_index) {
          // find new label
          int flag_check = 0;
          for (int k = 0; k < label_size && flag_check == 0; k++) {
            // if label K in the previous frame didn't match with any of the
            // label in the current frame

            if (rev_neighbor_vec.find(k) == rev_neighbor_vec.end() &&
                active_label_indices_.find(k) == active_label_indices_.end()) {
              // found a label
              std::vector<int> temp_nn_index;
              temp_nn_index.push_back(idy);
              rev_neighbor_vec[k] = temp_nn_index;
              final_neighbor_vec[idy] = k;
              neighbor_vec[idy] = k;

              // new object found
              active_label_indices_[k] = 1;

              if (k >= max_size_) max_size_ = k + 1;

              if (debug_)
                std::cout << "(2)Neighbor of " << idy << " " << k << " "
                          << label_size << std::endl;
              flag_check = 1;
              break;
            } else {
              if (debug_) {
                std::cout << k << " is taken " << std::endl;
                if (rev_neighbor_vec.find(k) == rev_neighbor_vec.end())
                  std::cout << "0" << std::endl;
                else
                  std::cout << "1" << std::endl;
                if (active_label_indices_.find(k) ==
                    active_label_indices_.end())
                  std::cout << "0" << std::endl;
                else
                  std::cout << "1" << std::endl;
              }
            }
          }
        }
      }
      rev_neighbor_vec[nn_index].clear();
      rev_neighbor_vec[nn_index].push_back(nn_nn_index);

    } else if (neighbor_vec[idx] == -2) {
      int flag_check = 0;

      for (int k = 0; k < label_size && flag_check == 0; k++) {
        // if label K in the previous frame didn't match with any of the label
        // in the current frame

        if (rev_neighbor_vec.find(k) == rev_neighbor_vec.end() &&
            active_label_indices_.find(k) == active_label_indices_.end()) {
          if (debug_) {
            std::cout << "Merging with " << k
                      << "even though jaccard index is zero" << std::endl;
            std::cout << k << " is not taken " << std::endl;
            if (rev_neighbor_vec.find(k) == rev_neighbor_vec.end())
              std::cout << "0" << std::endl;
            else
              std::cout << "1" << std::endl;
            if (active_label_indices_.find(k) == active_label_indices_.end())
              std::cout << "0" << std::endl;
            else
              std::cout << "1" << std::endl;
          }
          // found a label
          std::vector<int> temp_nn_index;
          temp_nn_index.push_back(idx);
          rev_neighbor_vec[k] = temp_nn_index;
          final_neighbor_vec[idx] = k;
          neighbor_vec[idx] = k;

          // new object found
          active_label_indices_[k] = 1;

          if (k >= max_size_) max_size_ = k + 1;

          if (debug_)
            std::cout << "(-2)Neighbor of " << idx << " " << k << " "
                      << label_size << std::endl;
          flag_check = 1;
          break;
        }
      }
    }

    /*else {

     if (rev_neighbor_vec.find(idx) == rev_neighbor_vec.end()) {
     std::vector<int> temp_nn_index;
     temp_nn_index.push_back(idx);
     rev_neighbor_vec[idx] = temp_nn_index;
     final_neighbor_vec[idx] = -1;
     neighbor_vec[idx] = -1;
     //std::cout << "(else)Neighbor of " << idx << " " << idx << std::endl;

     }

     }*/
  }

  return final_neighbor_vec;
}

template <typename PointT>
std::vector<pcl::PointIndices> SegmentPropagation<PointT>::FindFinalLabels(
    std::vector<int> neighbor_vec,
    std::vector<pcl::PointIndices> euclidean_label_indices) {
  std::vector<pcl::PointIndices> final_label_indices;
  final_label_indices.resize(max_size_);
  back_label_.clear();
  for (size_t idx = 0; idx < neighbor_vec.size(); idx++) {
    int nn_index = neighbor_vec[idx];
    if (nn_index == -1) continue;

    if (verbose_)
      std::cout << "(inside find final labels)NN Index " << nn_index
                << " size of euclidean label indices: "
                << euclidean_label_indices[idx].indices.size() << " "
                << final_label_indices.size() << std::endl;
    for (std::size_t j = 0; j < euclidean_label_indices[idx].indices.size();
         j++) {
      final_label_indices[nn_index].indices.push_back(
          euclidean_label_indices[idx].indices[j]);
      //	euclidean_labels.points[euclidean_label_indices[idx].indices[j]].label
      //= 			nn_index;
    }
    back_label_.insert(std::pair<int, int>(nn_index, idx));
  }
  return final_label_indices;
}

template <typename PointT>
typename SegmentPropagation<PointT>::CloudPtrVector
SegmentPropagation<PointT>::findFinalLabels(
    std::vector<int> neighbor_vec, CloudPtrVector euclidean_label_indices) {
  CloudPtrVector final_label_indices;
  back_label_.clear();

  Cloud temp;
  for (int i = 0; i < max_size_; i++)
    final_label_indices.push_back(temp.makeShared());

  for (size_t idx = 0; idx < neighbor_vec.size(); idx++) {
    int nn_index = neighbor_vec[idx];
    if (nn_index == -1) continue;

    if (verbose_)
      std::cout << "(inside find final labels)NN Index " << idx << " "
                << nn_index << " size of euclidean label indices: "
                << euclidean_label_indices[idx]->points.size() << " "
                << final_label_indices.size() << " " << neighbor_vec.size()
                << std::endl;

    final_label_indices[nn_index] = euclidean_label_indices[idx];
    back_label_.insert(std::pair<int, int>(nn_index, idx));
  }
  return final_label_indices;
}

template <typename PointT>
std::vector<pcl::PointIndices> SegmentPropagation<PointT>::PropogateLabels(
    std::vector<pcl::PointIndices> prev_label_indices_,
    std::vector<pcl::PointIndices> curr_label_indices_,
    CloudConstPtr prev_cloud_ptr_, CloudConstPtr curr_cloud_ptr_) {
  max_size_ = (prev_label_indices_.size() > curr_label_indices_.size())
                  ? prev_label_indices_.size()
                  : curr_label_indices_.size();

  // std::cout << "Size of Prev label indices " << prev_label_indices_.size() <<
  // " Size of Current Label Indices: " << euclidean_label_indices.size() <<
  // std::endl;
  std::vector<float> prev_centroids;
  prev_centroids = computeCentroids(prev_label_indices_, prev_cloud_ptr_);

  if (verbose_) std::cout << "Prev Centroids Computed..." << std::endl;

  std::vector<float> curr_centroids;
  curr_centroids = computeCentroids(curr_label_indices_, curr_cloud_ptr_);

  if (verbose_) std::cout << "Current Centroids Computed..." << std::endl;

  // match the current centroids to previous centroids
  std::vector<int> neighbor_vec = LinearMatch(curr_centroids, prev_centroids);

  // match the previous centroids to current centroids
  // std::vector<int> neighbor_vec_forward = linearMatch(prev_centroids,
  // curr_centroids);

  //	std::cout << "Linear Matching ... " << std::endl;

  std::map<int, std::vector<int> > rev_neighbor_vec;
  for (size_t idx = 0; idx < neighbor_vec.size(); idx++) {
    int nn_index = neighbor_vec[idx];
    if (rev_neighbor_vec.find(nn_index) != rev_neighbor_vec.end()) {
      rev_neighbor_vec[nn_index].push_back(idx);
    } else {
      std::vector<int> temp_nn_index;
      temp_nn_index.push_back(idx);
      rev_neighbor_vec[nn_index] = temp_nn_index;
    }
  }

  if (verbose_) std::cout << "Created Reverse Neighbor vec..." << std::endl;

  // two way match

  final_neighbors_ = TwoWayMatch(prev_centroids, curr_centroids, neighbor_vec,
                                 rev_neighbor_vec, curr_label_indices_.size());

  if (verbose_) std::cout << "Two way match completed..." << std::endl;

  // find final labels
  std::vector<pcl::PointIndices> final_label_indices;
  final_label_indices = findFinalLabels(final_neighbors_, curr_label_indices_);

  if (debug_)
    for (size_t idx = 0; idx < neighbor_vec.size(); idx++) {
      int nn_index = neighbor_vec[idx];
      if (nn_index == -1) continue;

      float diff_num =
          fabs(curr_centroids[idx + 6] - prev_centroids[nn_index + 6]);

      std::cout << "IDX: " << idx << " NN index: " << nn_index << " size "
                << final_label_indices[nn_index].indices.size() << std::endl;
    }
  //	std::cout << "find Final Labels completed..." << std::endl;
  // getchar();

  return final_label_indices;
}

template <typename PointT>
typename SegmentPropagation<PointT>::CloudPtrVector
SegmentPropagation<PointT>::propagateLabels(
    CloudPtrVector prev_label_indices_, CloudPtrVector curr_label_indices_) {
  if (verbose_)
    std::cout << "Inside propagating labels: NUM_PARAMS " << NUM_PARAMS
              << std::endl;
  fflush(stdout);

  max_size_ = (prev_label_indices_.size() > curr_label_indices_.size())
                  ? prev_label_indices_.size()
                  : curr_label_indices_.size();
  if (verbose_)
    std::cout << "Size of Prev label indices " << prev_label_indices_.size()
              << " Size of Current Label Indices: "
              << curr_label_indices_.size() << std::endl;
  std::vector<float> prev_centroids;
  prev_centroids = computeCentroids(prev_label_indices_);

  if (verbose_) std::cout << "Prev Centroids Computed..." << std::endl;

  std::vector<float> curr_centroids;
  curr_centroids = computeCentroids(curr_label_indices_);

  if (verbose_) std::cout << "Current Centroids Computed..." << std::endl;

  // match the current centroids to previous centroids
  std::vector<int> neighbor_vec = LinearMatch(curr_centroids, prev_centroids);

  // match the previous centroids to current centroids
  // std::vector<int> neighbor_vec_forward = linearMatch(prev_centroids,
  // curr_centroids);

  //	std::cout << "Linear Matching ... " << std::endl;

  std::map<int, std::vector<int> > rev_neighbor_vec;
  for (size_t idx = 0; idx < neighbor_vec.size(); idx++) {
    int nn_index = neighbor_vec[idx];
    if (rev_neighbor_vec.find(nn_index) != rev_neighbor_vec.end()) {
      rev_neighbor_vec[nn_index].push_back(idx);
    } else {
      std::vector<int> temp_nn_index;
      temp_nn_index.push_back(idx);
      rev_neighbor_vec[nn_index] = temp_nn_index;
    }
  }

  if (verbose_) std::cout << "Created Reverse Neighbor vec..." << std::endl;

  // two way match
  std::vector<int> final_neighbor_vec =
      TwoWayMatch(prev_centroids, curr_centroids, neighbor_vec,
                  rev_neighbor_vec, curr_label_indices_.size());

  if (verbose_) std::cout << "Two way match completed..." << std::endl;

  // find final labels
  CloudPtrVector final_label_indices;
  final_label_indices =
      findFinalLabels(final_neighbor_vec, curr_label_indices_);

  if (debug_) {
    for (size_t idx = 0; idx < final_neighbor_vec.size(); idx++) {
      int nn_index = final_neighbor_vec[idx];
      if (nn_index == -1) continue;

      float diff_num =
          fabs(curr_centroids[idx + 6] - prev_centroids[nn_index + 6]);

      std::cout << "IDX: " << idx << " NN index: " << nn_index << " size "
                << final_label_indices[nn_index]->points.size() << std::endl;
    }

    std::cout << "Size of final label indices: " << final_label_indices.size()
              << std::endl;
    for (std::size_t i = 0; i < final_label_indices.size(); i++) {
      if (final_label_indices[i]->points.size() != 0)
        std::cout << "Index: " << i << " "
                  << final_label_indices[i]->points.size() << std::endl;
    }
  }

  // UNCOMMENT IT WHEN USING THE SEGMENTATION DEMO UI
  final_neighbors_.clear();
  final_neighbors_ = final_neighbor_vec;

  return final_label_indices;
}

template <typename PointT>
typename SegmentPropagation<PointT>::CloudPtrVector
SegmentPropagation<PointT>::propagateLabels(CloudPtrVector label,
                                            gtsam::Pose3 pose_,
                                            gtsam::Symbol sym) {
  std::cout << "Inside temporal propagating " << std::endl;
  CloudPtrVector final_label, final_untransformed_label;

  CloudPtrVector transformed_label;
  Eigen::Matrix4f map_tform = pose_.matrix().cast<float>();
  for (std::size_t i = 0; i < label.size(); i++) {
    pcl::PointCloud<PointT> cluster;
    pcl::transformPointCloud(
        *label[i], cluster,
        map_tform);  // this is transforming the main point cloud
    transformed_label.push_back(cluster.makeShared());
  }

  if (prev_cloud_vec_.size() == 0 || restart_flag_) {
    if (restart_flag_) {
      std::cout << "Restarting temporal propagation..." << std::endl;
      restart_flag_ = 0;
    }
    std::cout << "[initializing] PrevCloudPtrVector is empty:  "
              << prev_cloud_vec_.size()
              << " CurrentCloudPtrVector size: " << label.size() << std::endl;
    final_label = InitializeLabel(transformed_label);
    final_untransformed_label = InitializeLabel(label);

    prev_cloud_vec_ = final_label;

    std::cout << "[after initializing] Size of prev_cloud_vec_: "
              << prev_cloud_vec_.size() << std::endl;
    //	prev_pose_ = pose_;
  } else {
    std::cout << "[propagating labels] PrevCloudPtrVector size :  "
              << prev_cloud_vec_.size()
              << " CurrentCloudPtrVector size: " << label.size() << std::endl;

    final_label = propagateLabels(prev_cloud_vec_, transformed_label);
    final_untransformed_label = findFinalLabels(final_neighbors_, label);
    prev_cloud_vec_ = final_label;

    std::cout << "[labels propagated]" << std::endl;
  }

  observations_.insert(
      std::pair<gtsam::Symbol, CloudPtrVector>(sym, final_untransformed_label));

  return final_label;
}

template <typename PointT>
typename SegmentPropagation<PointT>::CloudPtrVector
SegmentPropagation<PointT>::PredictLabels(CloudPtrVector label,
                                          gtsam::Pose3 pose_,
                                          gtsam::Symbol sym) {
  std::cout << "Predicting label " << std::endl;
  CloudPtrVector final_label, final_untransformed_label;

  double start = pcl::getTime();
  CloudPtrVector transformed_label;
  Eigen::Matrix4f map_tform = pose_.matrix().cast<float>();
  for (std::size_t i = 0; i < label.size(); i++) {
    pcl::PointCloud<PointT> cluster;
    pcl::transformPointCloud(
        *label[i], cluster,
        map_tform);  // this is transforming the main point cloud
    transformed_label.push_back(cluster.makeShared());
  }

  if (prev_cloud_vec_.size() == 0 || restart_flag_) {
    if (restart_flag_) {
      std::cout << "Restarting temporal propagation..." << std::endl;
      restart_flag_ = 0;
    }
    std::cout << "[initializing] Current map is empty:  "
              << prev_cloud_vec_.size()
              << " CurrentCloudPtrVector size: " << label.size() << std::endl;
    final_label = InitializeLabel(transformed_label);
    final_untransformed_label = CopyLabel(label, final_label);

    std::cout << "Size of final_label = " << final_label.size()
              << " Size of untransformed label "
              << final_untransformed_label.size() << std::endl;
    prev_cloud_vec_ = final_label;

    std::cout << "[after initializing] Size of prev_cloud_vec_: "
              << prev_cloud_vec_.size() << std::endl;
    //	prev_pose_ = pose_;
  } else {
    std::cout << "[propagating labels] Map size :  " << final_map_cloud_.size()
              << " CurrentCloudPtrVector size: " << label.size() << std::endl;

    // TODO: final_map_cloud is an unoptimized cloud estimated using the poses
    // till that time, experiment with propagation against the optimized cloud

    final_label = propagateLabels(final_map_cloud_, transformed_label);
    final_untransformed_label = findFinalLabels(final_neighbors_, label);
    prev_cloud_vec_ = final_label;

    std::cout << "[labels propagated]" << std::endl;
  }
  double end = pcl::getTime();
  std::cout << "Label Propagation took: " << double(end - start) << std::endl;

  start = pcl::getTime();

  sym.print("Symbol Added");
  std::cout << "Adding symbol "
            << " and cloud of size " << final_untransformed_label.size()
            << std::endl;
  observations_.insert(
      std::pair<gtsam::Symbol, CloudPtrVector>(sym, final_untransformed_label));
  final_map_cloud_ = CreateFinalCloud(sym, pose_);
  end = pcl::getTime();
  std::cout << "Final cloud creation took: " << double(end - start)
            << std::endl;

  std::cout << "Size of final map cloud " << final_map_cloud_.size()
            << std::endl;

  return final_label;
}
template <typename PointT>
void SegmentPropagation<PointT>::SetActiveLabelIndices(int size) {
  for (int i = 0; i < size; i++) {
    active_label_indices_[i] = 1;
  }
}

template <typename PointT>
typename SegmentPropagation<PointT>::CloudPtrVector
SegmentPropagation<PointT>::CreateFinalCloud(gtsam::Symbol sym_latest,
                                             gtsam::Pose3 cloud_pose) {
  /*
   * OPTIMAL_CLOUD: After segments from the current frame is added
   * to the map frame, iterate over all the previous segments and
   * their respective poses and recreate the map cloud
   *
   * ELSE: Only add the segments corresponding to the current frame to
   * the cloud
   *
   */
#if OPTIMAL_CLOUD == 1
  typename std::vector<CloudPtr> final_cloud_opt_;
  typename std::map<gtsam::Symbol, std::vector<CloudPtr> >::iterator it;
  final_count.clear();

  for (it = observations_.begin(); it != observations_.end(); it++) {
    gtsam::Symbol sym = it->first;
    std::vector<CloudPtr> cloud = it->second;
    boost::optional<gtsam::Pose3> cloud_pose = mapper_->predictPose(sym);
    if (cloud_pose) {
      gtsam::Pose3 new_pose = *cloud_pose;
      Eigen::Matrix4f map_transform = new_pose.matrix().cast<float>();

      for (std::size_t j = 0; j < cloud.size(); j++) {
        CloudPtr map_cloud(new Cloud());
        if (final_cloud_opt_.size() <= j) final_cloud_opt_.push_back(map_cloud);
        if (cloud[j]->points.size() == 0) continue;
        pcl::transformPointCloud(*cloud[j], *map_cloud, map_transform);

        //	std::cout << "Using map transform" << std::endl;

        Cloud new_cloud = *final_cloud_opt_[j] + *map_cloud;
        final_cloud_opt_[j] = new_cloud.makeShared();

        if (final_count.find(j) == final_count.end())
          final_count[j] = 1;
        else
          final_count[j]++;
      }
    }
  }

  std::cout << "Outside create Final Cloud" << std::endl;

#else
  typename std::vector<CloudPtr> final_cloud_opt_;
  final_cloud_opt_ = final_map_cloud_;
  gtsam::Symbol sym = sym_latest;
  std::vector<CloudPtr> cloud = observations_.at(sym);
  sym.print("[Create Final Cloud] Symbol");
  std::cout << "Size of the cloud " << cloud.size() << std::endl;
  gtsam::Pose3 new_pose = cloud_pose;
  Eigen::Matrix4f map_transform = new_pose.matrix().cast<float>();

  for (std::size_t j = 0; j < cloud.size(); j++) {
    CloudPtr map_cloud(new Cloud());
    if (final_cloud_opt_.size() <= j) final_cloud_opt_.push_back(map_cloud);
    if (cloud[j]->points.size() == 0) continue;
    pcl::transformPointCloud(*cloud[j], *map_cloud, map_transform);

    //	std::cout << "Using map transform" << std::endl;

    Cloud new_cloud = *final_cloud_opt_[j] + *map_cloud;
    final_cloud_opt_[j] = new_cloud.makeShared();

    if (final_count_.find(j) == final_count_.end())
      final_count_[j] = 1;
    else
      final_count_[j]++;
  }

#endif

  return final_cloud_opt_;
}

#endif

// instantation
template class SegmentPropagation<pcl::PointXYZRGBA>;
template class SegmentPropagation<pcl::PointXYZRGB>;
