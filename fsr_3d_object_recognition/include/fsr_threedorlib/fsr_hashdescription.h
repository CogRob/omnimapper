#ifndef _FSR_HASH_DESCRIPTION
#define _FSR_HASH_DESCRIPTION

#include <tbb/concurrent_unordered_map.h>
#include <tbb/concurrent_unordered_set.h>
#include <tbb/spin_mutex.h>
#include <fstream>
#include <istream>

#include <fsr_threedorlib/fsr_types.h>

namespace fsr_or
{

  template <typename PointT>
  class FSRHashMapDescription
  {
    typedef pcl::PointCloud<PointT> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    typedef pcl::PointCloud<pcl::Normal> NormalCloud;
    typedef typename NormalCloud::Ptr NormalCloudPtr;
    typedef typename NormalCloud::ConstPtr NormalCloudConstPtr;

    typedef typename FeatureHashMap<PointT>::Ptr FeatureHashMapPtr;

    protected:
    private:
      struct OrientedPointCloud
      {
        std::vector<OrientedPoint<PointT> > points;

        OrientedPointCloud () {}
        OrientedPointCloud (const CloudPtr &cloud,
                            const NormalCloudPtr &normals,
                            const std::vector<int> &norm_inds)
        {
          for (size_t i = 0; i < norm_inds.size (); ++i)
          {
            points.push_back(OrientedPoint<PointT> (cloud->points[norm_inds[i]], normals->points[i]));
          }
        }
      };

      typedef tbb::concurrent_unordered_set<FSRFeature, boost::hash<FSRFeature> > FeatureTracker;
      typedef boost::shared_ptr<FeatureTracker> FeatureTrackerPtr;

      typedef tbb::concurrent_unordered_multimap<FSRFeature, PointPairSystem<PointT>, boost::hash<FSRFeature> > FeatureHashMapSingle;
      typedef boost::shared_ptr<FeatureHashMapSingle> FeatureHashMapSinglePtr;
      typedef typename FeatureHashMapSingle::const_iterator FeatureHashMapSingleIterator;

      typedef tbb::spin_mutex TBBMutex;

      float d_min_;
      float d_max_;
      float d_min_sq_;
      float d_max_sq_;
      float K_;
      float angle_step_;
      int m_;
      FeatureHashMapPtr H_;
      ObjectMap<int>::Ptr modelSizes_;

      bool readModel (std::istream &mfile);
      void writeModel (const FeatureHashMapSingle &singlemap, const FeatureTracker &tracker, std::ofstream &mfile) const;
      void trimFeaturesFromSingleTable (FeatureHashMapSingle &singlemap, const FeatureTracker &tracker, int &sum1, int &sum2) const;
      bool validateCloudForProcessing (const CloudPtr &cloud, std::vector<int> &pt_indices) const;

    public:
      typedef boost::shared_ptr<FSRHashMapDescription> Ptr;

      FSRHashMapDescription (float d = 0.1f,
                             float delta_d = 0.001f / 2.0f,
                             float K = 0.2f,
                             float angle_step = 3.0f * 180.0f / (static_cast<float> (M_PI)));

      void addModelToFile (CloudPtr &cloud,
                            std::string model_name, int model_type,
                            std::ofstream &mfile) const;

      void readModelsFromFile (std::string &fname);

      float getMinDistance () const { return d_min_; }
      float getMaxDistance () const { return d_max_; }
      float getFractionKeptOfOriginalModel () const { return K_; }
      float getAngleStep () const { return angle_step_; }
      int getAverageModelSize () const { return m_; }
      FeatureHashMapPtr getHashMap () const { return H_->makeShared (); }
      ObjectMap<int>::Ptr getModelSizes () const { return modelSizes_->makeShared (); }
  };

}

#endif
