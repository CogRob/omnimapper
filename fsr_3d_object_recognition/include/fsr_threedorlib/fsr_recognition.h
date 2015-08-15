#ifndef FSR_RECOGNITION
#define FSR_RECOGNITION

#include <pcl/range_image/range_image_planar.h>
#include <pcl/features/normal_3d.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/octree/octree_search.h>
#include <pcl/common/time.h>
#include <tbb/concurrent_unordered_map.h>
#include <tbb/concurrent_vector.h>
#include <tbb/task_group.h>
#include <boost/unordered_set.hpp>
#include <boost/thread.hpp>

/// need this include to prevent linker errors associated with pcl not precompiling
/// everything needed for octree in headers that define the octree classes used
#include <pcl/octree/octree_impl.h>

#include <fsr_threedorlib/fsr_types.h>
#include <fsr_threedorlib/fsr_conflictgraph.h>
#include <fsr_threedorlib/fsr_recognition_helper.h>


namespace fsr_or
{

  template <typename PointT>
  class FSRRecognition
  {
    enum Device {FSR_DEV_NONE, FSR_DEV_OPENNI};

    typedef pcl::PointCloud<PointT> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    typedef pcl::PointCloud<pcl::Normal> NormalCloud;
    typedef typename NormalCloud::Ptr NormalCloudPtr;
    typedef typename NormalCloud::ConstPtr NormalCloudConstPtr;

    typedef pcl::RangeImagePlanar RangeImage;
    typedef RangeImage::Ptr RangeImagePtr;

    typedef typename FeatureHashMap<PointT>::Ptr FeatureHashMapPtr;
    typedef typename FeatureHashMap<PointT>::iterator FeatureHashMapIterator;

    typedef boost::posix_time::ptime Time;

    typedef std::vector<RegSolEntry<PointT> > RegistrationSolutions;
    typedef boost::shared_ptr<RegistrationSolutions> RegistrationSolutionsPtr;

    protected:
    private:
      typedef tbb::concurrent_unordered_multimap <OMKey::Ptr, std::pair<CloudPtr, MatrixT>, om_hash> RegistrationHypotheses;
      typedef boost::shared_ptr<RegistrationHypotheses> RegistrationHypothesesPtr;
      typedef typename RegistrationHypotheses::iterator RegistrationHypothesesIterator;

      typedef std::vector<OMKey::Ptr>::iterator KeyBoxIterator;
      typedef typename std::vector<PointPairSystem<PointT> >::iterator PPSIterator;

      typedef tbb::spin_mutex TBBMutex;
      typedef boost::mutex BoostMutex;

      /// the distances between point pairs in model in the interval [d - err, d + err]
      float d_min_;
      float d_max_;
      float d_min_sq_;
      float d_max_sq_;
      /// discretization of angles in feature
      float angle_step_;
      /// the size of the octree leaves
      float L_;
      /// distances from the front and back of voxels, used for hypothesis acceptance
      float L_space_eps_;
      /// the desired probability of success for RANSAC
      float P_s_;
      /// percentage of points kept during feature description
      float K_;
      /// average number of points in a model
      float m_;
      /// the probability a sampled point is from an object in the scene that is in the database
      float C_;
      /// number of iterations
      int N_;
      /// min proportion of points of the tranfored model that must be visible to be a good hypothesis
      float t_V_;
      /// max proportion of points of the tranfored model that can occlude scene points and still be a good hypothesis
      float t_P_;
      /// how the point clouds were created
      Device device_;
      /// focal length of the capture device used to create pcd files
      float focallength_;
      /// the density of the points in the scene
      float scene_resolution_;

      FeatureHashMapPtr H_;
      std::vector<OMKey::Ptr> omkey_box_;

      ObjectMap<int>::Ptr modelSizes_;

      CloudConstPtr cloud_sensor_;
      boost::optional<CloudConstPtr> cloud_input_;
      boost::optional<CloudConstPtr> cloud_scene_;
      boost::optional<CloudPtr> cloud_scene_reduced_;

      boost::optional<RangeImagePtr> range_image_scene_;

      void (*getRangeImage)(CloudPtr&, RangeImagePtr&, Eigen::Affine3f&, float);

      boost::optional<Eigen::Affine3f> sensor_pose_;

      typename pcl::octree::OctreePointCloudVoxelCentroid<PointT>::Ptr oct_centroid_;
      typename pcl::octree::OctreePointCloudSearch<PointT>::Ptr oct_search_;

      boost::optional<boost::shared_ptr<ConflictGraph> > conflict_graph_;

      RegistrationHypothesesPtr hypotheses_;
      boost::optional<RegistrationSolutionsPtr> solutions_;

      typename ModelCache<PointT>::Ptr model_cache_;

      bool updated_cloud_;
      bool ready_;

      BoostMutex cloud_mutex_;
      TBBMutex io_mutex_;

      boost::thread spin_thread_;

      tbb::task_group background_group_;

      boost::optional<CloudConstPtr> pub_scene_;
      boost::optional<CloudPtr> pub_scene_reduced_;
      boost::optional<RangeImagePtr> pub_range_image_;
      boost::optional<Eigen::Affine3f> pub_sensor_pose_;
      boost::optional<boost::shared_ptr<ConflictGraph> > pub_conflict_graph_;
      boost::optional<RegistrationSolutionsPtr> pub_solutions_;

      std::vector<boost::function<void (const CloudConstPtr&, CloudPtr&, Time)> > callbacks_scene_reduced_;
      std::vector<boost::function<void (RangeImagePtr&, Eigen::Affine3f, Time)> > callbacks_range_image_;
      std::vector<boost::function<void (boost::shared_ptr<ConflictGraph>&, Time)> > callbacks_conflict_graph_;
      std::vector<boost::function<void (RegistrationSolutionsPtr&, Time)> > callbacks_solutions_;

      void spinThread ();
      void intializeRecognition ();
      void generateHypotheses (size_t iter_id);
      bool acceptHypothesis (Cloud &model, float m, OMKey::Ptr &id);
      void removeConflictingHypotheses ();
      void cleanup ();
      Time stamp2ptime (uint64_t stamp);
      uint64_t getStamp ();

    public:
      typedef boost::shared_ptr<FSRRecognition> Ptr;

      FSRRecognition (float L = 0.01f, float L_eps = 0.005f,
                      float P_s = 0.95f, float C = 0.25f,
                      float t_V = 0.25f, float t_P = 0.05f,
                      Device dev = FSR_DEV_NONE, float fl = 525.0f);

      void spin ();
      void spinOnce ();
      void cloudCallback (const CloudConstPtr &cloud);
      void publish ();
      bool ready ();

      void setDescriptionInfo (const float &d_min, const float &d_max, const float &angle_step, const float &K, const int &m)
      {
        d_min_ = d_min;
        d_max_ = d_max;
        d_min_sq_ = pow (d_min, 2);
        d_max_sq_ = pow (d_max, 2);
        angle_step_ = angle_step;
        K_ = K;
        m_ = static_cast<float> (m);
      }

      void setHashMap (const FeatureHashMapPtr &fhmp) { H_ = fhmp; }
      void setModelSizes (const ObjectMap<int>::Ptr &oms) { modelSizes_ = oms; }
      void setModelKeyBox (const ObjectMap<int>::Ptr &om)
      {
        for (typename ObjectMap<int>::iterator it = om->begin (); it != om->end (); ++it)
        {
          omkey_box_.push_back((it->first).makeShared ());
        }
      }

      void setSceneReducedCallback (boost::function<void (const CloudConstPtr&, CloudPtr&, Time)> &f)
      {
        callbacks_scene_reduced_.push_back(f);
      }

      void setRangeImageCallback (boost::function<void (RangeImagePtr&, Eigen::Affine3f, Time)> &f)
      {
        callbacks_range_image_.push_back(f);
      }

      void setConflictGraphCallback (boost::function<void (boost::shared_ptr<ConflictGraph>&, Time)> &f)
      {
        callbacks_conflict_graph_.push_back(f);
      }

      void setRegistrationSolutionsCallback (boost::function<void (RegistrationSolutionsPtr&, Time)> &f)
      {
        callbacks_solutions_.push_back(f);
      }
  };

}

#endif // FSR_RECOGNITION
