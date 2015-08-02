#ifndef _FSR_HASH_DESCRIPTION
#define _FSR_HASH_DESCRIPTION

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
      typedef tbb::concurrent_hash_map<FSRFeature, std::vector<PointPairSystem<PointT> >, HashCompare<FSRFeature> > FeatureHashMapSingle;
      typedef boost::shared_ptr<FeatureHashMapSingle> FeatureHashMapSinglePtr;

      typedef tbb::spin_mutex DiamMutex;

      float d_;
      float delta_d_;
      float K_;
      float angle_step_;
      int m_;
      FeatureHashMapPtr H_;
      ObjectMap<float>::Ptr modelDiams_;
      ObjectMap<int>::Ptr modelSizes_;
      DiamMutex dmutex_;

      bool readModel (std::istream &mfile);
      void writeModel (FeatureHashMapSingle &singlemap, std::ofstream &mfile);
      void trimFeaturesFromSingleTable (FeatureHashMapSingle &singlemap);
      bool validateCloudForProcessing (CloudPtr &cloud);

    public:
      typedef boost::shared_ptr<FSRHashMapDescription> Ptr;

      FSRHashMapDescription (float d = 0.08f,
                             float delta_d = 0.005f,
                             float K = 0.5f,
                             float angle_step = 6.0f / 180.0f * static_cast<float> (M_PI));

      void addModelToFile (CloudPtr &cloud_input,
                            std::string model_name, int model_type, int model_view,
                            std::ofstream &mfile);

      void readModelsFromFile (std::istream &mfile);

      float getPairDistance () { return d_; }
      float getDistanceTolerance () { return delta_d_; }
      float getFractionKeptOfOriginalModel () { return K_; }
      float getAngleStep () { return angle_step_; }
      int getAverageModelSize () { return m_; }
      FeatureHashMapPtr getHashMap () { return H_->makeShared (); }
      ObjectMap<float>::Ptr getModelDiams () { return modelDiams_->makeShared (); }
      ObjectMap<int>::Ptr getModelSizes () { return modelSizes_->makeShared (); }
  };
}

#endif
