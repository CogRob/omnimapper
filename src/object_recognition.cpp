#ifndef OBJECT_RECOGNITION_IMPL_
#define OBJECT_RECOGNITION_IMPL_

#include <omnimapper/object_recognition.h>
#include <pcl/common/concatenate.h>
#include <pcl/common/io.h>
#include <map>
#include <string>
#define USE_UNIFORM_SAMPLING 1

template <typename FeatureType>
ObjectRecognition<FeatureType>::ObjectRecognition(
    boost::shared_ptr<pcl::Keypoint<pcl::PointXYZRGBA, pcl::PointXYZI> >
        keypoint_detector,
    typename pcl::Feature<pcl::PointXYZRGBA, FeatureType>::Ptr
        feature_extractor)
    : correspondences_(new pcl::Correspondences),
      source_features_(new pcl::PointCloud<FeatureType>),
      pose_map(),
      source_keypoints_(new pcl::PointCloud<pcl::PointXYZI>()),
      target_keypoints_(new pcl::PointCloud<pcl::PointXYZI>()),
      keypoint_detector_(keypoint_detector),
      feature_extractor_(feature_extractor),
      target_features_(new pcl::PointCloud<FeatureType>),
      show_source2target_(false),
      show_target2source_(false),
      show_correspondences(false),
      verbose_(true),
      debug_(true),
      object_centroid_map() {
  std::cout << "Constructing Feature Matches" << std::endl;
#if USE_UNIFORM_SAMPLING == 1
  uniform_sampler_.reset(new pcl::UniformSampling<pcl::PointXYZRGBA>);
#endif
}

template <typename FeatureType>
void ObjectRecognition<FeatureType>::matchClouds(
    typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr source,
    typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr target)

{
  source_ = source;
  target_ = target;
  detectKeypoints(source_, source_keypoints_);
  detectKeypoints(target_, target_keypoints_);

  extractDescriptors(source_, source_keypoints_, source_features_);
  extractDescriptors(target_, target_keypoints_, target_features_);

  findCorrespondences(source_features_, target_features_, source2target_);
  findCorrespondences(target_features_, source_features_, target2source_);

  filterCorrespondences();

  // determineInitialTransformation ();
  // determineFinalTransformation ();
}

template <typename FeatureType>
void ObjectRecognition<FeatureType>::matchToStoredTemplate(
    typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr source,
    typename pcl::PointCloud<FeatureType> target_feature)

{
  source_ = source;
  detectKeypoints(source_, source_keypoints_);
  extractDescriptors(source_, source_keypoints_, source_features_);
  findCorrespondences(source_features_, target_feature.makeShared(),
                      source2target_);
  findCorrespondences(target_feature.makeShared(), source_features_,
                      target2source_);
  filterCorrespondences();

  // determineInitialTransformation ();
  // determineFinalTransformation ();
}

template <typename FeatureType>
int ObjectRecognition<FeatureType>::matchToFile(
    typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr source,
    typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr target) {
  int max_num_corr = 0;
  typename pcl::PointCloud<FeatureType> target_feature;

  source_ = source;
  detectKeypoints(source_, source_keypoints_);
  if (source_keypoints_->size() == 0)
    return -1;  // -2 = cannot find keypoints in the source cloud

  extractDescriptors(source_, source_keypoints_, source_features_);

  target_ = target;
  detectKeypoints(target_, target_keypoints_);
  extractDescriptors(target_, target_keypoints_, target_features_);

  findCorrespondences(source_features_, target_features_, source2target_);
  findCorrespondences(target_features_, source_features_, target2source_);
  if (verbose_)
    std::cout << "found correspondences [source2target_] "
              << source2target_.size() << " [target2source_] "
              << target2source_.size() << std::endl;
  filterCorrespondences();
  if (verbose_)
    std::cout << "[After filtering]#Corr: " << correspondences_->size()
              << std::endl;

  if (static_cast<int>(correspondences_->size()) > max_num_corr) {
    max_num_corr = correspondences_->size();
  }

  return max_num_corr;
}

template <typename FeatureType>
std::pair<int, int> ObjectRecognition<FeatureType>::matchToFeatureFile(
    typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr source, int label,
    gtsam::Matrix C) {
  int max_num_corr = 0, matching_object = -1;
  source_ = source;
  detectKeypoints(source_, source_keypoints_);

  std::cout << "Size of Source: " << source->points.size() << std::endl;
  std::cout << "Size of Keypoints: " << source_keypoints_->size() << std::endl;
  if (source_keypoints_->size() == 0)
    return std::make_pair(
        max_num_corr, -2);  // -2 = cannot find keypoints in the source cloud

  extractDescriptors(source_, source_keypoints_, source_features_);

  typename std::map<int, std::vector<pcl::PointCloud<FeatureType> > >::iterator
      feature_map_it;

  typename pcl::PointCloud<FeatureType> target_feature;

#if USE_KDTREE == 1
  // iterate over KDTree of centroids
  Eigen::Vector4f feature_centroid;
  pcl::compute3DCentroid(*source_keypoints_, feature_centroid);
  std::vector<float> query_vec;
  query_vec.push_back((float)feature_centroid[0]);
  query_vec.push_back((float)feature_centroid[1]);
  query_vec.push_back((float)feature_centroid[2]);
  flann::Matrix<float> query(&query_vec[0], 1, 3);
  query.cols = 3;
  std::vector<std::vector<int> > indices;
  std::vector<std::vector<float> > dists;

  if (object_centroid_map.size() != 0)
    index->knnSearch(query, indices, dists, object_centroid_map.size(),
                     flann::SearchParams(flann::FLANN_CHECKS_UNLIMITED));

  // Rebuild index
  std::cout << "Rebuilding index " << std::endl;
  centroid_data.push_back((float)feature_centroid[0]);
  centroid_data.push_back((float)feature_centroid[1]);
  centroid_data.push_back((float)feature_centroid[2]);
  object_centroid_map.push_back(label);
  flann::Matrix<float> dataset(&centroid_data[0], object_centroid_map.size(),
                               3);
  index = new flann::Index<flann::L2<float> >(dataset,
                                              flann::KDTreeSingleIndexParams());
  index->buildIndex();
  std::cout << "Index built " << std::endl;

  float dist_thresh = 0;  // covX*covX + covY*covY + covZ*covZ;
  if (indices.size() != 0) {
    for(std::size_t i = 0; i < indices[0].size(); i++) {
      int index = indices[0][i];
      int obj_id = object_centroid_map[index];

      if (obj_id == label) continue;
      // TODO: remove the below line if it's not loop closure
      // compute the mahalanobis distance
      gtsam::Point3 matching_pt(centroid_data[3 * index],
                                centroid_data[3 * index + 1],
                                centroid_data[3 * index + 2]);
      gtsam::Point3 object_pt(feature_centroid[0], feature_centroid[1],
                              feature_centroid[2]);
      gtsam::Point3 D = object_pt - matching_pt;
      gtsam::Point3 Dcov(D.x() * C(0, 0) + D.y() * C(1, 0) + D.z() * C(2, 0),
                         D.x() * C(0, 1) + D.y() * C(1, 1) + D.z() * C(2, 1),
                         D.x() * C(0, 2) + D.y() * C(1, 2) + D.z() * C(2, 2));

      matching_pt.print("Matching point\n");
      object_pt.print("Object point\n");
      gtsam::Point3 final_pt(Dcov.x() * D.x(), Dcov.y() * D.y(),
                             Dcov.z() * D.z());
      double final_val = Dcov.dot(D);
      final_pt.print("Final point\n");
      std::cout << "Covaraince inverse: " << C << std::endl;
      std::cout << "Final Val: " << final_val << std::endl;

      if (final_val > 4) break;

      std::cout << "Distance to " << obj_id << " is " << dists[0][i]
                << std::endl;
      typename std::vector<pcl::PointCloud<FeatureType> > obj_desc =
          feature_map[obj_id];
#else

  for (feature_map_it = feature_map.begin();
       feature_map_it != feature_map.end(); feature_map_it++) {
    int obj_id = feature_map_it->first;
    typename std::vector<pcl::PointCloud<FeatureType> > obj_desc =
        feature_map_it->second;

#endif

      for(std::size_t j = 0; j < obj_desc.size(); j++) {
        target_feature = feature_map[obj_id][j];
        target_keypoints_ = keypoint_map[obj_id][j].makeShared();

        if (verbose_)
          std::cout << "finding correspondence between source " << label
                    << " and feature file " << obj_id << " out of "
                    << obj_desc.size() << " and " << indices[0].size()
                    << std::endl;

        //	std::cout << "source feature size: " << source_features_->size()
        //<< " target_feature: " << target_feature.size() << std::endl;
        findCorrespondences(source_features_, target_feature.makeShared(),
                            source2target_);
        findCorrespondences(target_feature.makeShared(), source_features_,
                            target2source_);
        if (verbose_)
          std::cout << "found correspondences [source2target_] "
                    << source2target_.size() << " [target2source_] "
                    << target2source_.size() << std::endl;
        filterCorrespondences();
        if (verbose_)
          std::cout << "[After filtering]#Corr: " << correspondences_->size()
                    << std::endl;
        if (correspondences_->size() < 3) continue;

        if (static_cast<int>(correspondences_->size()) > max_num_corr) {
          max_num_corr = correspondences_->size();
          matching_object = obj_id;
        }
      }
    }
  }
  std::cout << "Max Matching Object for  " << label
            << " is: " << matching_object << " with: " << max_num_corr
            << std::endl;
  return std::make_pair(max_num_corr, matching_object);
}

template <typename FeatureType>
pcl::PointCloud<FeatureType> ObjectRecognition<FeatureType>::computeDescriptor(
    typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr source)

{
  /* detect keypoints */
  typename pcl::PointCloud<FeatureType>::Ptr feature_desc;
  feature_desc.reset(new pcl::PointCloud<FeatureType>());
  detectKeypoints(source, source_keypoints_);
  if (debug_)
    std::cout << "[computeDescriptor] size of keypoints: "
              << source_keypoints_->size() << std::endl;
  if (source_keypoints_->size() == 0) return *feature_desc;

  /* extract descriptors */
  extractDescriptors(source, source_keypoints_, feature_desc);
  if (debug_)
    std::cout << "[computeDescriptor] size of features: "
              << feature_desc->size() << std::endl;
  return *feature_desc;
}

template <typename FeatureType>
std::pair<pcl::PointCloud<FeatureType>, pcl::PointCloud<pcl::PointXYZI> >
ObjectRecognition<FeatureType>::computeAndStoreDescriptor(
    typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr source,
    std::string filename) {
  pcl::PointCloud<FeatureType> feature = computeDescriptor(source);
  if (debug_) std::cout << "Size of feature " << feature.size() << std::endl;

  if (feature.size() > 0) {
    std::string desc_file = filename;
    desc_file.replace(desc_file.length(), 9, "_desc");
    storeDescriptor(feature.makeShared(), desc_file);

    // store keypoints
    std::string keypoint_file = filename;
    keypoint_file.replace(keypoint_file.length(), 9, "_keypoint");
    pcl::io::savePCDFileASCII(keypoint_file, *source_keypoints_);
  }

  return std::make_pair(feature, *source_keypoints_);
}

template <typename FeatureType>
void ObjectRecognition<FeatureType>::storeDescriptor(
    typename pcl::PointCloud<FeatureType>::Ptr features, std::string filename)

{
  pcl::io::savePCDFileASCII(filename, *features);
}

template <typename FeatureType>
std::pair<pcl::PointCloud<FeatureType>, pcl::PointCloud<pcl::PointXYZI> >
ObjectRecognition<FeatureType>::loadDescriptor(const std::string filename)

{
  pcl::PointCloud<FeatureType> feature_desc;
  pcl::PointCloud<pcl::PointXYZI> keypoints;
  if (pcl::io::loadPCDFile<FeatureType>(filename, feature_desc) ==
      -1)  //* load the file
  {
    PCL_ERROR("Couldn't read file %s\n", filename.c_str());
    //       return (-1);
  }

  std::string keypoint_file = filename;
  keypoint_file.replace(keypoint_file.length() - 5, 9, "_keypoint");
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(keypoint_file, keypoints) == -1) {
    PCL_ERROR("Couldn't read file %s\n", keypoint_file.c_str());
    //       return (-1);
  }

  return std::make_pair(feature_desc, keypoints);
}

template <typename FeatureType>
void ObjectRecognition<FeatureType>::detectKeypoints(
    typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr input,
    pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints) const {
#if USE_UNIFORM_SAMPLING == 1
  uniform_sampler_->setRadiusSearch(0.03f);
  uniform_sampler_->setInputCloud(input);

  /* The following code was changed from these during PCL 1.7.1 -> 1.7.2
          pcl::PointCloud<int> sampled_indices;
          uniform_sampler_->compute(sampled_indices);
          pcl::copyPointCloud(*input, sampled_indices.points, *keypoints);
  */
  pcl::PointCloud<pcl::PointXYZRGBA> sampled_cloud;
  uniform_sampler_->filter(sampled_cloud);
  pcl::copyPointCloud<pcl::PointXYZRGBA, pcl::PointXYZI>(sampled_cloud,
                                                         *keypoints);

#else
  if (debug_) std::cout << "keypoint detection..." << std::flush;
  keypoint_detector_->setInputCloud(input);
  keypoint_detector_->compute(*keypoints);
  if (debug_)
    std::cout << "OK. keypoints found: " << keypoints->points.size()
              << std::endl;

#endif
}

template <typename FeatureType>
void ObjectRecognition<FeatureType>::extractDescriptors(
    typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr input,
    typename pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints,
    typename pcl::PointCloud<FeatureType>::Ptr features) {
  typename pcl::PointCloud<pcl::PointXYZRGBA>::Ptr kpts(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  kpts->points.resize(keypoints->points.size());

  pcl::copyPointCloud(*keypoints, *kpts);

  typename pcl::FeatureFromNormals<pcl::PointXYZRGBA, pcl::Normal,
                                   FeatureType>::Ptr feature_from_normals =
      boost::dynamic_pointer_cast<pcl::FeatureFromNormals<
          pcl::PointXYZRGBA, pcl::Normal, FeatureType> >(feature_extractor_);

  feature_extractor_->setSearchSurface(input);
  feature_extractor_->setInputCloud(kpts);

  if (feature_from_normals)
  // if (boost::dynamic_pointer_cast<typename
  // pcl::FeatureFromNormals<pcl::PointXYZRGBA, pcl::Normal, FeatureType> >
  // (feature_extractor_))
  {
    if (debug_)
      std::cout << "normal estimation..." << input->height << " "
                << input->width << " " << input->size() << std::flush;
    typename pcl::PointCloud<pcl::Normal>::Ptr normals(
        new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> normal_estimation;
    normal_estimation.setSearchMethod(
        pcl::search::Search<pcl::PointXYZRGBA>::Ptr(
            new pcl::search::KdTree<pcl::PointXYZRGBA>));
    normal_estimation.setRadiusSearch(0.01);
    normal_estimation.setInputCloud(input);
    normal_estimation.compute(*normals);
    feature_from_normals->setInputNormals(normals);

    if (debug_) std::cout << "OK" << std::endl;
  }

  if (debug_) std::cout << "descriptor extraction..." << std::flush;
  feature_extractor_->compute(*features);

  if (debug_) {
    std::cout << "[extractDescriptor] size of features " << features->size()
              << std::endl;
    std::cout << "OK" << std::endl;
  }
}

template <typename FeatureType>
void ObjectRecognition<FeatureType>::findCorrespondences(
    typename pcl::PointCloud<FeatureType>::Ptr source,
    typename pcl::PointCloud<FeatureType>::Ptr target,
    std::vector<int>& correspondences) const {
  if (debug_) std::cout << "correspondence assignment..." << std::flush;
  correspondences.resize(source->size());

  // Use a KdTree to search for the nearest matches in feature space
  pcl::KdTreeFLANN<FeatureType> descriptor_kdtree;
  descriptor_kdtree.setInputCloud(target);

  // Find the index of the best match for each keypoint, and store it in
  // "correspondences_out"
  const int k = 1;
  std::vector<int> k_indices(k);
  std::vector<float> k_squared_distances(k);
  for(std::size_t i = 0; i < source->size(); ++i) {
    descriptor_kdtree.nearestKSearch(*source, i, k, k_indices,
                                     k_squared_distances);
    correspondences[i] = k_indices[0];
  }
  if (debug_) std::cout << "OK" << std::endl;
}

template <typename FeatureType>
void ObjectRecognition<FeatureType>::filterCorrespondences() {
  if (debug_) std::cout << "correspondence rejection..." << std::flush;
  std::vector<std::pair<unsigned, unsigned> > correspondences;
  for (unsigned cIdx = 0; cIdx < source2target_.size(); ++cIdx)
    if (target2source_[source2target_[cIdx]] == static_cast<int>(cIdx))
      correspondences.push_back(std::make_pair(cIdx, source2target_[cIdx]));

  if (debug_)
    std::cout << "size of corr: " << correspondences.size() << std::endl;

  correspondences_->resize(correspondences.size());
  for (unsigned cIdx = 0; cIdx < correspondences.size(); ++cIdx) {
    (*correspondences_)[cIdx].index_query = correspondences[cIdx].first;
    (*correspondences_)[cIdx].index_match = correspondences[cIdx].second;
  }

  if (debug_)
    std::cout << "size of corr: " << correspondences_->size() << std::endl;
  if (correspondences_->size() < 5) return;
  if (debug_)
    std::cout << "[filterCorrespondences] #corr: " << correspondences_->size()
              << std::endl;

  pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZI>
      rejector;
  rejector.setInputSource(source_keypoints_);
  rejector.setInputTarget(target_keypoints_);
  rejector.setInputCorrespondences(correspondences_);
  rejector.getCorrespondences(*correspondences_);

  if (debug_) {
    std::cout << "Size of filtered correspondences "
              << correspondences_->size();
    std::cout << std::endl;
    std::cout << "OK" << std::endl;
  }
}

template <typename FeatureType>
void ObjectRecognition<FeatureType>::determineInitialTransformation() {
  std::cout << "initial alignment..." << std::flush;
  pcl::registration::TransformationEstimation<pcl::PointXYZI,
                                              pcl::PointXYZI>::Ptr
      transformation_estimation(
          new pcl::registration::TransformationEstimationSVD<pcl::PointXYZI,
                                                             pcl::PointXYZI>);

  transformation_estimation->estimateRigidTransformation(
      *source_keypoints_, *target_keypoints_, *correspondences_,
      initial_transformation_matrix_);

  pcl::transformPointCloud(*source_, *source_transformed_,
                           initial_transformation_matrix_);
  std::cout << "OK" << std::endl;
}

template <typename FeatureType>
void ObjectRecognition<FeatureType>::determineFinalTransformation() {
  std::cout << "final registration..." << std::flush;
  pcl::Registration<pcl::PointXYZRGBA, pcl::PointXYZRGBA>::Ptr registration(
      new pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA>);
  registration->setInputSource(source_transformed_);
  // registration->setInputSource (source_segmented_);
  registration->setInputTarget(target_);
  registration->setMaxCorrespondenceDistance(0.05);
  registration->setRANSACOutlierRejectionThreshold(0.05);
  registration->setTransformationEpsilon(0.000001);
  registration->setMaximumIterations(1000);
  registration->align(*source_registered_);
  transformation_matrix_ = registration->getFinalTransformation();
  std::cout << "OK" << std::endl;
}

template <typename FeatureType>
std::pair<int, int> ObjectRecognition<FeatureType>::matchToDatabase(
    typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cluster,
    std::vector<gtsam::Pose3> pose_array, int label, gtsam::Matrix covariance) {
  // label = label+ used_label_threshold_;
  typename std::pair<pcl::PointCloud<FeatureType>,
                     pcl::PointCloud<pcl::PointXYZI> >
      feature_keypoint;
  int max_num_corr = 0, matching_object = -1;

  std::cout << "Matching to feature file " << std::endl;
  std::cout << "Original Covariance: " << covariance << std::endl;
  gtsam::Matrix covariance_mat = covariance.inverse();
  std::cout << "Original Covariance Inverse : " << covariance_mat << std::endl;
  std::pair<int, int> corr_object =
      matchToFeatureFile(cluster, label, covariance_mat);
  std::cout << "Matching done " << std::endl;

  max_num_corr = corr_object.first;
  matching_object = corr_object.second;
  if (matching_object == -2)
    return std::make_pair(-2, -1);  // cannot find keypoints in the source cloud

  PoseVector pose_vec;
  pose_vec.pose_vector = pose_array;
  std::string pose_filename =
      pose_directory_ + "/object_" + boost::lexical_cast<std::string>(label);
  savePoseArray(pose_filename, pose_vec);
  pose_map.insert(std::pair<int, PoseVector>(label, pose_vec));

  if (max_num_corr < 12) /* store in the file */
  {
    if (feature_map.find(label) == feature_map.end()) {
      typename std::vector<pcl::PointCloud<FeatureType> > temp;
      feature_map[label] = temp;

      std::vector<pcl::PointCloud<pcl::PointXYZI> > temp_keypoint;
      keypoint_map[label] = temp_keypoint;

      std::vector<Eigen::Vector4f> temp_centroid;
      centroid_map[label] = temp_centroid;
    }
    std::string output_filename =
        feature_directory_ + "/object_" +
        boost::lexical_cast<std::string>(label) + "_" +
        boost::lexical_cast<std::string>(feature_map[label].size());

    //		pcl::PointCloud<pcl::FPFHSignature33> feature =
    feature_keypoint = computeAndStoreDescriptor(cluster, output_filename);
    std::cout << "Descriptor computed and stored" << std::endl;
    Eigen::Vector4f feature_centroid;
    pcl::compute3DCentroid(feature_keypoint.second, feature_centroid);
    std::cout << "Centroid computed" << std::endl;

    if (feature_keypoint.first.size() > 0) {
      feature_map[label].push_back(feature_keypoint.first);
      keypoint_map[label].push_back(feature_keypoint.second);
      centroid_map[label].push_back(feature_centroid);
    }

    // put in object segment map
    // known segment temporally but didn't match any object in this frame
    if (segment_object.find(label) == segment_object.end()) {
      // segment not found in the database
      std::map<int, int> temp_object;
      temp_object[label] = 1;
      segment_object[label] = temp_object;
    } else {
      segment_object[label][label]++;
    }

    // pcl::PointCloud < pcl::SHOT1344 > feature = feature_keypoint.first;
    // pcl::PointCloud < pcl::PointXYZI > keypoint = feature_keypoint.second;
    // std::cout << "Size of feature " << feature.size() << std::endl;

    /*
     std::string pcd_filename = "/home/siddharth/kinect/pcd_files/feature_"
     + boost::lexical_cast < std::string
     > (feature_files.size()) + ".pcd";
     pcl::io::savePCDFile(pcd_filename, cluster);
     */

    return std::make_pair(-1, max_num_corr);
  } else {
    std::string output_filename =
        feature_directory_ + "/object_" +
        boost::lexical_cast<std::string>(label) + "_" +
        boost::lexical_cast<std::string>(feature_map[label].size());

    //		pcl::PointCloud<pcl::FPFHSignature33> feature =
    //	feature_keypoint = computeAndStoreDescriptor(cluster, output_filename);

    // put in object segment map
    // known segment temporally and matched matching_object in this frame
    if (segment_object.find(label) != segment_object.end()) {
      if (segment_object[label].find(matching_object) ==
          segment_object[label].end()) {
        // segment not found in the database
        segment_object[label][matching_object] = 1;
      } else {
        segment_object[label][matching_object]++;
      }

    } else {
      // matching_object and label
      std::map<int, int> temp_object;
      temp_object[matching_object] = 1;
      segment_object[label] = temp_object;
    }
    return std::make_pair(matching_object, max_num_corr);
  }
}

template <typename FeatureType>
void ObjectRecognition<FeatureType>::savePoseArray(std::string filename,
                                                   PoseVector pose_vector) {
  std::ofstream ofs(filename.c_str());
  boost::archive::text_oarchive oa(ofs);
  oa << pose_vector;

  ofs.close();
}

template <typename FeatureType>
PoseVector ObjectRecognition<FeatureType>::loadPoseArray(std::string filename) {
  std::ifstream ifs(filename.c_str());
  boost::archive::text_iarchive ia(ifs);

  PoseVector return_val;
  ia >> return_val;

  return return_val;
}

template <typename FeatureType>
int ObjectRecognition<FeatureType>::loadDatabase(std::string dir) {
  std::cout << "load directory_ " << dir << std::endl;
  base_directory_ = dir;
  feature_directory_ = base_directory_ + "/object_templates";
  pose_directory_ = base_directory_ + "/pose_templates";
  model_directory_ = base_directory_ + "/object_models";

  typename std::pair<pcl::PointCloud<FeatureType>,
                     pcl::PointCloud<pcl::PointXYZI> >
      feature_keypoint;
  boost::filesystem::directory_iterator end_itr;
  int feature_count = 0;

  for (boost::filesystem::directory_iterator itr(feature_directory_);
       itr != end_itr; ++itr) {
    /* load object templates and save it in feature_desc vector */
    // pcl::PointCloud<pcl::FPFHSignature33>  feature_desc =
    std::size_t string_find = itr->path().string().find("keypoint");
    if (string_find != std::string::npos) continue;
    feature_keypoint = loadDescriptor(itr->path().string());

    // find the centroid of the keypoints
    Eigen::Vector4f feature_centroid;
    pcl::compute3DCentroid(feature_keypoint.second, feature_centroid);
    //	std::cout << feature_centroid[0] << " " << feature_centroid[1] << " "
    //			<< feature_centroid[2] << std::endl;
    centroid_data.push_back(feature_centroid[0]);
    centroid_data.push_back(feature_centroid[1]);
    centroid_data.push_back(feature_centroid[2]);
    feature_count++;

    // std::cout << "Feature Centroid of " << itr->path().string() << "  is: "
    // << feature_centroid << std::endl;

    // parsing the string
    std::vector<std::string> parsed_str;
    boost::split(parsed_str, itr->path().string(), boost::is_any_of("_"));
    int obj_id_val = boost::lexical_cast<int>(parsed_str[2]);
    // std::cout << "Pushing object " << obj_id_val << std::endl;
    object_centroid_map.push_back(obj_id_val);

    if (feature_map.find(obj_id_val) != feature_map.end()) {
      feature_map[obj_id_val].push_back(feature_keypoint.first);
      keypoint_map[obj_id_val].push_back(feature_keypoint.second);
      centroid_map[obj_id_val].push_back(feature_centroid);

    } else {
      typename std::vector<pcl::PointCloud<FeatureType> > temp;
      feature_map[obj_id_val] = temp;

      std::vector<pcl::PointCloud<pcl::PointXYZI> > temp_keypoint;
      keypoint_map[obj_id_val] = temp_keypoint;

      std::vector<Eigen::Vector4f> temp_centroid;
      centroid_map[obj_id_val] = temp_centroid;

      feature_map[obj_id_val].push_back(feature_keypoint.first);
      keypoint_map[obj_id_val].push_back(feature_keypoint.second);
      centroid_map[obj_id_val].push_back(feature_centroid);
    }
  }

  /// 	Create Flann Matrix and Tree
  if (feature_count != 0) {
    flann::Matrix<float> dataset(&centroid_data[0], feature_count, 3);
    index = new flann::Index<flann::L2<float> >(
        dataset, flann::KDTreeSingleIndexParams());
    index->buildIndex();
    std::cout << "Tree constructed" << std::endl;
  }

  std::cout << "Loading poses" << std::endl;
  for (boost::filesystem::directory_iterator itr(pose_directory_);
       itr != end_itr; ++itr) {
    /* load object templates and save it in feature_desc vector */
    PoseVector pose_vec = loadPoseArray(itr->path().string());
    std::vector<std::string> parsed_str;
    boost::split(parsed_str, itr->path().string(), boost::is_any_of("_"));
    int obj_id_val = boost::lexical_cast<int>(parsed_str[2]);
    pose_map.insert(std::pair<int, PoseVector>(obj_id_val, pose_vec));
  }

  /*	std::cout << "Loading models" << std::endl;
          for (boost::filesystem::directory_iterator itr(model_directory_); itr
     != end_itr;
                          ++itr) {
                   load object templates and save it in feature_desc vector
                  PoseVector pose_vec = loadPCD(itr->path().string());
                  std::vector<std::string> parsed_str;
                                  boost::split(parsed_str, itr->path().string(),
     boost::is_any_of("_")); int obj_id_val =
     boost::lexical_cast<int>(parsed_str[2]); pose_map.insert(std::pair<int,
     PoseVector>(obj_id_val, pose_vec));

          }*/

  std::string mapping_file = base_directory_ + "/mapping.txt";
  int max_segment = loadMapping(mapping_file);

  return max_segment;
}

template <typename FeatureType>
int ObjectRecognition<FeatureType>::loadMapping(std::string filename) {
  std::string line;

  std::ifstream file(filename.c_str());
  int max_segment = -1;
  int segment_id, object_id, object_count;
  if (file.is_open()) {
    while (file.good()) {
      std::getline(file, line);
      std::stringstream s(line);

      s >> segment_id;
      if (!s) break;
      //  std::cout << "Line: " << line << std::endl;
      std::map<int, int> temp_map;
      //   std::cout << "Segment: " << segment_id;
      while (s) {
        s >> object_id;
        s >> object_count;
        temp_map[object_id] = object_count;
        // std::cout << object_id << " " << object_count << std::endl;
      }
      segment_object[segment_id] = temp_map;

      //   for(std::map<int,int>::iterator it =
      //   segment_object[segment_id].begin(); it !=
      //   segment_object[segment_id].end(); it++){
      //  	  std::cout << it->first << " " << it->second << std::endl;
      //    }

      if (segment_id > max_segment) max_segment = segment_id;

      //    std::cout << "Line: " << segment_id << " " << line << std::endl;
      //  std::cout <<" Max segment " << max_segment << std::endl;
    }
    file.close();
  }

  else
    std::cout << "Unable to open file";

  return max_segment;
}

template <typename FeatureType>
void ObjectRecognition<FeatureType>::saveMapping(std::string filename) {
  std::string line;
  std::istringstream s;
  std::ofstream file(filename.c_str());

  std::map<int, std::map<int, int> >::iterator it;
  std::map<int, int>::iterator it_1;

  for (it = segment_object.begin(); it != segment_object.end(); it++) {
    file << it->first;
    for (it_1 = it->second.begin(); it_1 != it->second.end(); it_1++) {
      file << " " << it_1->first << " " << it_1->second;
    }
    file << std::endl;
  }

  file.close();
}

template <typename FeatureType>
void ObjectRecognition<FeatureType>::saveObjectStats(std::string filename) {
  std::string line;
  std::istringstream s;
  std::ofstream file(filename.c_str());

  std::map<int, std::map<int, int> >::iterator it;
  std::map<int, int>::iterator it_1;

  for (it = segment_object.begin(); it != segment_object.end(); it++) {
    file << it->first;
    int obj_id = it->first;
    file << " " << pose_map.at(obj_id).pose_vector.size();
    file << std::endl;
  }

  file.close();
}

#if 0
template<typename FeatureType>
void ObjectRecognition<FeatureType>::reconstructSurface ()
{
	std::cout << "surface reconstruction..." << std::flush;
	// merge the transformed and the target point cloud
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr merged (new pcl::PointCloud<pcl::PointXYZRGBA>);
	*merged = *source_registered_;
	*merged += *target_segmented_;

	// apply grid filtering to reduce amount of points as well as to make them uniform distributed
	pcl::VoxelGrid<pcl::PointXYZRGBA> voxel_grid;
	voxel_grid.setInputCloud(merged);
	voxel_grid.setLeafSize (0.002f, 0.002f, 0.002f);
	voxel_grid.setDownsampleAllData(true);
	voxel_grid.filter(*merged);

	pcl::PointCloud<pcl::PointXYZRGBANormal>::Ptr vertices (new pcl::PointCloud<pcl::PointXYZRGBANormal>);
	pcl::copyPointCloud(*merged, *vertices);

	pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::PointXYZRGBANormal> normal_estimation;
	normal_estimation.setSearchMethod (pcl::search::Search<pcl::PointXYZRGBA>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGBA>));
	normal_estimation.setRadiusSearch (0.01);
	normal_estimation.setInputCloud (merged);
	normal_estimation.compute (*vertices);

	pcl::search::KdTree<pcl::PointXYZRGBANormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBANormal>);
	tree->setInputCloud (vertices);

	surface_reconstructor_->setSearchMethod(tree);
	surface_reconstructor_->setInputCloud(vertices);
	surface_reconstructor_->reconstruct(surface_);
	std::cout << "OK" << std::endl;
}

template<typename FeatureType>
void ObjectRecognition<FeatureType>::run()
{
	visualizer_.spin ();
}

template<typename FeatureType>
void ObjectRecognition<FeatureType>::keyboard_callback (const pcl::visualization::KeyboardEvent& event, void*)
{
	if (event.keyUp())
	{
		switch (event.getKeyCode())
		{
			case '1':
			if (!visualizer_.removePointCloud("source_points"))
			{
				visualizer_.addPointCloud(source_, "source_points");
			}
			break;

			case '2':
			if (!visualizer_.removePointCloud("target_points"))
			{
				visualizer_.addPointCloud(target_, "target_points");
			}
			break;

			case '3':
			if (!visualizer_.removePointCloud("source_segmented"))
			{
				visualizer_.addPointCloud(source_segmented_, "source_segmented");
			}
			break;

			case '4':
			if (!visualizer_.removePointCloud("target_segmented"))
			{
				visualizer_.addPointCloud(target_segmented_, "target_segmented");
			}
			break;

			case '5':
			if (!visualizer_.removePointCloud("source_keypoints"))
			{
				pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> keypoint_color (source_keypoints_, 0, 0, 255);
				//pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> keypoint_color (source_keypoints_, "intensity");
				visualizer_.addPointCloud(source_keypoints_, keypoint_color, "source_keypoints");
			}
			break;

			case '6':
			if (!visualizer_.removePointCloud("target_keypoints"))
			{
				//pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> keypoint_color (target_keypoints_, "intensity");
				pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> keypoint_color (target_keypoints_, 255, 0, 0);
				visualizer_.addPointCloud(target_keypoints_, keypoint_color, "target_keypoints");
			}
			break;

			case '7':
			if (!show_source2target_)
			visualizer_.addCorrespondences<pcl::PointXYZI>(source_keypoints_, target_keypoints_, source2target_, "source2target");
			else
			visualizer_.removeCorrespondences("source2target");

			show_source2target_ = !show_source2target_;
			break;

			case '8':
			if (!show_target2source_)
			visualizer_.addCorrespondences<pcl::PointXYZI>(target_keypoints_, source_keypoints_, target2source_, "target2source");
			else
			visualizer_.removeCorrespondences("target2source");

			show_target2source_ = !show_target2source_;
			break;

			case '9':
			if (!show_correspondences)
			visualizer_.addCorrespondences<pcl::PointXYZI>(source_keypoints_, target_keypoints_, *correspondences_, "correspondences");
			else
			visualizer_.removeCorrespondences("correspondences");
			show_correspondences = !show_correspondences;
			break;

			case 'i':
			case 'I':
			if (!visualizer_.removePointCloud("transformed"))
			visualizer_.addPointCloud(source_transformed_, "transformed");
			break;

			case 'r':
			case 'R':
			if (!visualizer_.removePointCloud("registered"))
			visualizer_.addPointCloud(source_registered_, "registered");
			break;

			case 't':
			case 'T':
			visualizer_.addPolygonMesh(surface_, "surface");
			break;
		}
	}
}

#endif

#endif /* OBJECT_RECOGNITION_IMPL_ */

// Instantiate
template class ObjectRecognition<pcl::SHOT1344>;
template class ObjectRecognition<pcl::PFHRGBSignature250>;
template class ObjectRecognition<pcl::VFHSignature308>;
