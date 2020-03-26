#pragma once

#include <gtsam/geometry/Pose3.h>
#include <omnimapper/object_recognition.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/surface/mls.h>

template <typename PointT>
class ObjectDiscovery {
 public:
  ObjectDiscovery();
  ~ObjectDiscovery(){};

  void CreateGraph();
  void LoadRepresentations(std::string dir);
  void CreateFinalCloud(std::string dir);
  void ReconstructSurface(typename pcl::PointCloud<PointT>::Ptr merged, int id);
  float ComputeJaccardIndex(Eigen::Vector4f min_pt_1, Eigen::Vector4f min_pt_2,
                            Eigen::Vector4f max_pt_1, Eigen::Vector4f max_pt_2);
  void MergeClouds();
  int FindLabel(int segment);
  int FindMin(int seg, std::vector<int> seg_vec);
  std::map<int, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> map_cloud_;

 protected:
  int max_object_size_, max_current_size_;

  boost::shared_ptr<ObjectRecognition<pcl::SHOT1344> > correspondence_estimator_;
  std::map<int, std::vector<int> > graph_;
  std::map<int, std::vector<int> > match_graph_;
  std::vector<int> segment_arr_;
  std::string object_dir_;
};
