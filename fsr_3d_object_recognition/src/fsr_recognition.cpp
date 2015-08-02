#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <tbb/tbb.h>

#include <fsr_threedorlib/fsr_recognition.h>

namespace fsr_or
{
  template <typename PointT>
  FSRRecognition<PointT>::FSRRecognition (float d, float delta_d,
                                               float L, float P_s,
                                               float K, float C,
                                               float t_V, float t_P,
                                               int gri, float fl)
  : d_ (d),
    delta_d_ (delta_d),
    L_ (L),
    P_s_ (P_s),
    K_ (K),
    m_ (0),
    C_ (C),
    N_ (0),
    t_V_ (t_V),
    t_P_ (t_P),
    focallength_ (fl),
    H_ (new FeatureHashMap<PointT>),
    modelDiams_ (new ObjectMap<float>),
    modelSizes_ (new ObjectMap<int>),
    cloud_sensor_ (new Cloud),
    cloud_input_ (boost::none),
    cloud_scene_ (boost::none),
    cloud_scene_reduced_ (boost::none),
    range_image_scene_ (boost::none),
    sensor_pose_ (boost::none),
    ne_ (new pcl::NormalEstimation<PointT, pcl::Normal> ()),
    oct_centroid_ (new pcl::octree::OctreePointCloudVoxelCentroid<PointT> (L)),
    oct_search_(new pcl::octree::OctreePointCloudSearch<PointT> (L)),
    conflict_graph_ (boost::none),
    hypotheses_ (new RegistrationHypotheses ()),
    solutions_ (new RegistrationSolutions ()),
    updated_cloud_ (false),
    ready_ (true),
    pub_scene_reduced_ (boost::none),
    pub_conflict_graph_ (boost::none),
    pub_solutions_ (boost::none)
  {
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    ne_.setRadiusSearch (0.03);
    ne_.setSearchMethod (tree);

    hypotheses_->clear ();
    solutions_->clear ();

    static bool seeded =  false;
    if (!seeded)
    {
        srand ((unsigned int) pcl::getTime ());
        seeded = true;
    }

    if (gri == 0)
    {
      getRangeImage = [] (CloudPtr &cloud, RangeImagePtr &ri, Eigen::Affine3f &pose, float f)
      {
        ri->createFromPointCloudWithFixedSize(*cloud,
                                              640, 480,
                                              320.0f, 240.0f,
                                              f, f,
                                              pose, pcl::RangeImage::CAMERA_FRAME,
                                              0.0f, 0.0f);
      };
    }
    /// TODO: add range image creation using openni depth image
    //else if (gri == 1}) {}
  }

  template <typename PointT>
  void FSRRecognition<PointT>::spin ()
  {
    spin_thread = boost::thread (&FSRRecognition<PointT>::spinThread, this);
  }

  template <typename PointT>
  void FSRRecognition<PointT>::spinThread ()
  {
    while (true)
    {
      boost::this_thread::sleep (boost::posix_time::milliseconds (30));
      spinOnce ();
    }
  }

  template <typename PointT>
  void FSRRecognition<PointT>::spinOnce ()
  {
    /// TODO: add resolution invariance
    if (cloud_mutex_.try_lock())
    {
      if (updated_cloud_)
      {
        *cloud_input_= cloud_sensor_;
        updated_cloud_ = false;
        ready_ = false;
      }
      cloud_mutex_.unlock();
    }

    /// recognition
    intializeRecognition ();
    tbb::parallel_for(0, N_, 1,
                     [&] (int i)
    {
      generateHypotheses ();
    });
    removeConflictingHypotheses ();

    /// give user results
    pub_scene_reduced_ = cloud_scene_reduced_;
    pub_conflict_graph_ = conflict_graph_
    *pub_solutions_ = solutions_;
    publish ();

    /// cleanup
    cleanup ();
  }

  template <typename PointT>
  void FSRRecognition<PointT>::intializeRecognition ()
  {
    if (!cloud_input_)
    {
      N_ = 0;
      return;
    }

    cloud_scene_ = cloud_input_;
    scene_resolution_ = computeCloudResolution(*cloud_scene_);

    /// compute the range image used for evaluating hypotheses
    *sensor_pose_ = Eigen::Affine3f (Eigen::Translation3f((*cloud_scene_)->sensor_origin_[0],
								    (*cloud_scene_)->sensor_origin_[1],
								    (*cloud_scene_)->sensor_origin_[2])) *
								    Eigen::Affine3f((*cloud_scene_)->sensor_orientation_);
    getRangeImage(*cloud_scene_, *range_image_scene_, *sensor_pose_, focallength_);

    /// reduce the scene cloud
    *cloud_scene_reduced_ (new Cloud);

    oct_centroid_->setInputCloud (*cloud_scene_);
    oct_centroid_->addPointsFromInputCloud ();

    typename pcl::octree::OctreePointCloud<PointT>::AlignedPointTVector centroids;
    (*cloud_scene_reduced_)->width = oct_centroid_->getVoxelCentroids (centroids);
    (*cloud_scene_reduced_)->height = 1;
    (*cloud_scene_reduced_)->points.resize (cloud_scene_reduced_->width * cloud_scene_reduced_->height);
    for (size_t i = 0; i < (*cloud_scene_reduced_)->width; i++)
    {
      (*cloud_scene_reduced_)->points[i] = centroids[i];
    }
    oct_centroid_->deleteTree();

    /// create the search tree for reduced scene
    oct_search_->setInputCloud (*cloud_scene_reduced_);
    oct_search_->addPointsFromInputCloud ();

    /// calcuate number of iterations
    N_ = static_cast<int> (ceil (-(static_cast<float> ((*cloud_scene_reduced_)->width) * log (1.0f - P_s_)) / (m_ * K_ * C_)));

    ne_->setInputCloud (*cloud_scene_reduced_);
  }

  template <typename PointT>
  void FSRRecognition<PointT>::generateHypotheses ()
  {
    if (N_ == 0)
    {
      return;
    }

    int d = d_ + delta_d_;
    /// sample p_u
    int u = rand() % static_cast<int> ((*cloud_scene_reduced_)->points.size ());
    PointT pu = (*cloud_scene_reduced_)->points[u];

    /// sample p_v from around p_u
    std::vector<int> uball;
    std::vector<float> uSqDists;
    oct_search_->radiusSearch(pu, d, uball, uSqDists);
    int v = uball[rand() % static_cast<int> (uball.size ())];
    PointT pv = (*cloud_scene_reduced_)->points[v];

    /// compute the normals for p_u and p_v
    boost::shared_ptr<std::vector<int> > indexptr (new std::vector<int> ());
    NormalCloudPtr normal_u (new NormalCloud ());
    indexptr->push_back (u);
    ne_->setIndices (indexptr);
    ne_->compute (*normal_u);

    NormalCloudPtr normal_v (new NormalCloud ());
    (*indexptr)[0] = v;
    ne_->setIndices (indexptr);
    ne_->compute (*normal_v);

    /// compute the feature for the sampled point pair
    float f1;
    FSRFeature feature;
    PointPairSystem<PointT> pps (PointPair<PointT> (pu, normal_u->points[0], pv, normal_v->points[0]), Eigen::Matrix4f ());
    if (!computeOPFFeature (pps.opp.first.p.getVector4fMap (),
                            pps.opp.first.n.getNormalVector4fMap (),
                            pps.opp.second.p.getVector4fMap (),
                            pps.opp.second.n.getNormalVector4fMap (),
                            f1, feature.first, feature.second.first, feature.second.second,
                            d))
    {
      return;
    }

    /// get point pairs with same feature as pair (u,v)
    ObjectMap<std::vector<PointPairSystem<PointT> > > allpairs;
    if (!(H_->at_all (feature, allpairs)))
    {
      return;
    }

    /// compute the local coordinate system of F for (u,v)
    if (!computeF(pps.opp, pps.F))
    {
      return;
    }

    /// generate and test hypotheses
    tbb::parallel_for (tbb::blocked_range<OMPPSIterator> (allpairs.begin (), allpairs.end ()),
                       [&] (const tbb::blocked_range<OMPPSIterator> &r)
    {
      OMKey key;
      std::vector<PointPairSystem<PointT> > mpairs;
      std::string mname, mtype, mview;
      int m;
      CloudPtr cloud_model (new pcl::PointCloud<PointT> ());
      /// iterate over all models with computed feature
      for (OMPPSIterator it = r.begin (); it != r.end (); ++it)
      {
        key = it->first;

        mpairs = *(it->find_model (key));
        m = *(modelSizes_->find_model (key));

        cloud_model->clear ();
        /// TODO add repository for models that are in use by user
        pcl::io::loadPCDFile ((it->getObjectFileName (key)).c_str (), *cloud_model);

        /// iterate over all point pairs in the model that have this feature
        tbb::parallel_for (tbb::blocked_range<PPSIterator> (mpairs.begin (), mpairs.end (), 800),
                           [&] (const tbb::blocked_range<PPSIterator> &r2)
        {
          CloudPtr cloud_transformed (new pcl::PointCloud<PointT> ());
          CloudPtr cloud_scene_hypothesis (new pcl::PointCloud<PointT> ());
          AffineT T;
          for (PPSIterator it2 = r2.begin (); it2 != r2.end (); ++it2)
          {
            cloud_transformed->clear ();
            cloud_scene_hypothesis->clear ();

            *T = pps.F * (it2->F.inverse ());
            pcl::transformPointCloud (*cloud_model, *cloud_transformed, T);
            pcl::copyPointCloud (*(*cloud_scene_reduced_), cloud_scene_hypothesis);
            *(cloud_scene_hypothesis) += *cloud_model;

            if (acceptHypothesis (cloud_scene_hypothesis, m, key))
            {
              typename RegistrationHypotheses::accessor a;
              hypotheses_->insert (a, key);
              a->second.first = cloud_model;
              a->second.second = T;
            }
          }
        }, tbb::simple_partitioner ());
      }
    });
  }

  template <typename PointT>
  bool FSRRecognition<PointT>::acceptHypothesis (CloudPtr &cloud, int m, OMKey id)
  {
    RangeImagePtr range_hypothesis (new RangeImage);
    getRangeImage(cloud, range_hypothesis, *sensor_pose_, focallength_);

    /// calculate the nmber of points that are visible and
    /// the number of points that occlude valid points
    int m_v = 0, m_p = 0;
    float xf, yf, range;
    int x, y;
    float veps = 0.01f;
    float peps = 0.01f;
    float mf = static_cast<float> (m);
    pcl::PointWithRange s, h;
    boost::unordered_set<CNPoint> explainedpts;
    for (size_t i = 0; cloud->points.size (); ++i)
    {
      range_hypothesis->getImagePoint (cloud->points[i].getVector4fMap (), xf, yf, range);
      range_hypothesis->real2DToInt2D (xf, yf, x, y);
      s = (*range_image_scene_)->getPoint (x, y);
      h = range_hypothesis->getPoint (x, y);

      /// count hypothesis points that align with scene points
      if (h.z < s.z + veps && h.z > s.z - veps)
      {
        ++m_v;
        explainedpts.insert (CNPoint (x, y));
      }
      /// count hypothesis points that don't occlude other points
      m_p += (h.range < s.range - peps) ? 1 : 0;
    }

    /// add hypothesis to conflict graph for later
    (*conflict_graph_)->addNode (id, m_v, m_p, explainedpts);

    float u_V = static_cast<float> (m_v) / mf;
    float u_P = static_cast<float> (m_p) / mf;
    return u_V > t_V_ && u_P < t_P_;
  }

  template <typename PointT>
  void FSRRecognition<PointT>::removeConflictingHypotheses ()
  {
    if (N_ == 0)
    {
      return;
    }

    /// build conflict graph
    tbb::parallel_for (size_t(0), (*conflict_graph_)->size (),
                       [&] (size_t i)
    {
      tbb::parallel_for (tbb::blocked_range<size_t> (i + 1, (*conflict_graph_)->size (), 1000),
                                                     [&] (const tbb::blocked_range<size_t> &r)
      {
        for (size_t j = r.begin (); j < r.end (); ++j)
        {
          (*conflict_graph_)->addEdge (i, j);
        }
      }, tbb::simple_partitioner ());
    });

    /// suppress hypotheses that conflict with better hypotheses
    tbb::parallel_for (tbb::blocked_range<size_t> (0, (*conflict_graph_)->edgeCapacity (), 1000),
                                                  [&] (const tbb::blocked_range<size_t> &r)
    {
      for (size_t i = r.begin (); i < r.end (); ++i)
      {
        (*conflict_graph_)->suppressNode (i);
      }
    }, tbb::simple_partitioner ());

    /// add the all the unsupressed hypotheses to the solution container
    std::pair<CloudPtr, AffineT> sol;
    OMKey key;
    for (size_t i = 0; i < (*conflict_graph_)->size (); i++)
    {
      if (!(*conflict_graph_)->nodeSuppressed (i))
      {
        key = (*conflict_graph_)->getNodeID (i);
        typename RegistrationHypotheses::accessor a;
        hypotheses_->find (a, key);
        solutions_->push_back (a->second);
      }
    }
  }

  template <typename PointT>
  void FSRRecognition<PointT>::cloudCallback (const CloudConstPtr& cloud)
  {
    boost::lock_guard<boost::mutex> (cloud_mutex);
    cloud_sensor_ = cloud;
    updated_cloud_ = true;
    ready_ = false;
  }

  template <typename PointT>
  void FSRRecognition<PointT>::publish ()
  {
   if (pub_scene_reduced_)
   {
     for (size_t i = 0; i < callbacks_scene_reduced_.size (); i++)
     {
       callbacks_scene_reduced_[i](*pub_scene_reduced_);
     }
   }

   if (pub_conflict_graph_)
   {
     for (size_t i = 0; i < callbacks_conflict_graph_.size (); i++)
     {
       callbacks_conflict_graph_[i](*pub_conflict_graph_);
     }
   }

   if (pub_solutions_)
   {
     for (size_t i = 0; i < callbacks_solutions_.size (); i++)
     {
       callbacks_solutions_[i](*pub_solutions_);
     }
   }
  }

  template <typename PointT>
  void FSRRecognition<PointT>::cleanup ()
  {
    cloud_input_ = boost::none;
    cloud_scene_ = boost::none;
    cloud_scene_reduced_ = boost::none;
    range_image_scene_ = boost::none;
    sensor_pose_ = boost::none;
    oct_centroid_->deleteTree ();
    oct_search_->deleteTree ();
    conflict_graph_ = boost::none;
    hypotheses_->clear ();
    solutions_->clear ();
  }
}
