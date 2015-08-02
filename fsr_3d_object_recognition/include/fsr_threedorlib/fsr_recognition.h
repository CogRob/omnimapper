#ifndef FSR_RECOGNITION
#define FSR_RECOGNITION

#include <pcl/range_image/range_image_planar.h>
#include <pcl/features/normal_3d.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/octree/octree_search.h>
#include <pcl/common/time.h>
#include <tbb/concurrent_vector.h>
#include <boost/unordered_set.hpp>
#include <boost/thread.hpp>

#include <fsr_threedorlib/fsr_types.h>
#include <fsr_threedorlib/fsr_conflictgraph.h>

namespace fsr_or
{
  template <typename PointT>
  class FSRRecognition
  {
    typedef pcl::PointCloud<PointT> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    typedef pcl::PointCloud<pcl::Normal> NormalCloud;
    typedef typename NormalCloud::Ptr NormalCloudPtr;
    typedef typename NormalCloud::ConstPtr NormalCloudConstPtr;

    typedef pcl::RangeImagePlanar RangeImage;
    typedef RangeImage::Ptr RangeImagePtr;

    typedef typename FeatureHashMap<PointT>::Ptr FeatureHashMapPtr;

    typedef boost::shared_ptr<Eigen::Affine3f> AffineT;

    typedef std::vector<std::pair<CloudPtr, AffineT> > RegistrationSolutions;
    typedef boost::shared_ptr<RegistrationSolutions> RegistrationSolutionsPtr;

    typedef boost::posix_time::ptime Time;

    protected:
    private:
      typedef tbb::concurrent_hash_map <OMKey, std::pair<CloudPtr, AffineT>, HashCompare<OMKey> > RegistrationHypotheses;
      typedef boost::shared_ptr<RegistrationHypotheses> RegistrationHypothesesPtr;

      typedef typename ObjectMap<std::vector<PointPairSystem<PointT> > >::iterator OMPPSIterator;
      typedef typename std::vector<PointPairSystem<PointT> >::iterator PPSIterator;

      /// the distance between point pairs in model
      float d_;
      /// the tolerance in deviation from d
      float delta_d_;
      /// the size of the octree leaves
      float L_;
      /// the desired probability of success for RANSAC
      float P_s_;
      /// percentage of points kept during feature description
      float K_;
      /// average number of points in a model
      int m_;
      /// the probability a sampled point is from an object in the scene that is in the database
      float C_;
      /// number of iterations
      int N_;
      /// min proportion of points of the tranfored model that must be visible to be a good hypothesis
      float t_V_;
      /// max proportion of points of the tranfored model that can occlude scene points and still be a good hypothesis
      float t_P_;
      /// focal length of the capture device used to create pcd files
      float focallength_;
      /// the density of the points in the scene
      float scene_resolution_;

      typename FeatureHashMap<PointT>::Ptr H_;
      ObjectMap<float>::Ptr modelDiams_;
      ObjectMap<int>::Ptr modelSizes_;

      CloudConstPtr cloud_sensor_;
      boost::optional<CloudConstPtr> cloud_input_;
      boost::optional<CloudConstPtr> cloud_scene_;
      boost::optional<CloudPtr> cloud_scene_reduced_;

      boost::optional<RangeImagePtr> range_image_scene_;

      void (*getRangeImage)(CloudPtr&, RangeImagePtr&, Eigen::Affine3f&, float);

      boost::optional<Eigen::Affine3f> sensor_pose_;

      typename pcl::NormalEstimation<PointT, pcl::Normal>::Ptr ne_;

      typename pcl::octree::OctreePointCloudVoxelCentroid<PointT>::Ptr oct_centroid_;
      typename pcl::octree::OctreePointCloudSearch<PointT>::Ptr oct_search_;

      boost::optional<boost::shared_ptr<ConflictGraph> > conflict_graph_;

      RegistrationHypothesesPtr hypotheses_;
      RegistrationSolutionsPtr solutions_;

      bool updated_cloud_;
      bool ready_;

      boost::mutex cloud_mutex_;

      boost::thread spin_thread;

      boost::optional<CloudPtr> pub_scene_reduced_;
      boost::optional<boost::shared_ptr<ConflictGraph> > pub_conflict_graph_;
      boost::optional<RegistrationSolutionsPtr> pub_solutions_;

      std::vector<boost::function<void (const CloudConstPtr)> > callbacks_scene_reduced_;
      std::vector<boost::function<void (const boost::shared_ptr<ConflictGraph>)> > callbacks_conflict_graph_;
      std::vector<boost::function<void (const RegistrationSolutionsPtr, Time t)> > callbacks_solutions_;

      void spinThread ();
      void intializeRecognition ();
      void generateHypotheses ();
      bool acceptHypothesis (CloudPtr &cloud, int m, OMKey id);
      void removeConflictingHypotheses ();
      void cleanup ();

    public:
      typedef boost::shared_ptr<FSRRecognition> Ptr;

      FSRRecognition (float d = 0.08f, float delta_d = 0.005f,
                      float L = 0.05f, float P_s = 0.95f,
                      float K = 0.5f, float C = 0.25f,
                      float t_V = 0.25f, float t_P = 0.05f,
                      int gri = 0, float fl = 525.0f);

      void spin ();
      void spinOnce ();
      void cloudCallback (const CloudConstPtr &cloud);
      void publish ();

      void setDescriptionInfo (float d, float delta_d, float K, int m) { d_ = d; delta_d_ = d; K_ = K; m_ = m; }
      void setHashMap (FeatureHashMapPtr fhmp) { H_ = fhmp; };
      void setModelDiams (ObjectMap<float>::Ptr omd) { modelDiams_ = omd; }
      void setModelSizes (ObjectMap<int>::Ptr oms) { modelSizes_ = oms; }

      void setSceneReducedCallback (boost::function<void (const CloudConstPtr)> &f)
      {
        callbacks_scene_reduced_.push_back(f);
      }

      void setConflictGraphCallback (boost::function<void (const boost::shared_ptr<ConflictGraph>)> &f)
      {
        callbacks_conflict_graph_.push_back(f);
      }
      void setRegistrationSolutionsCallback (boost::function<void (const RegistrationSolutionsPtr, Time t)> &f)
      {
        callbacks_solutions_.push_back(f);
      }
  };
}

#endif // FSR_RECOGNITION
