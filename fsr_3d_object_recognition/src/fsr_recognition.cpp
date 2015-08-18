#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <tbb/tbb.h>

#include <fsr_threedorlib/fsr_recognition.h>

#define FSR_RECOGNITION_DEBUG 1
#define FSR_RECOGNITION_VERBOSE 1

#if FSR_RECOGNITION_DEBUG || FSR_RECOGNITION_VERBOSE
typedef tbb::spin_mutex DBGMutex;
static DBGMutex dbg_out_mutex;
#endif // FSR_RECOGNITION_DEBUG

#if FSR_RECOGNITION_DEBUG
static std::ofstream dbg_file;
#endif // FSR_RECOGNITION_DEBUG

#include <vtkSmartPointer.h>
#include <vtkImageData.h>
#include <vtkPNGWriter.h>

namespace fsr_or
{
  template <typename PointT>
  FSRRecognition<PointT>::FSRRecognition (float L, float L_eps,
                                          float P_s,float C,
                                          float t_V, float t_P,
                                          Device dev, float fl)
  : d_min_ (0.0f),
    d_max_ (0.0f),
    d_min_sq_ (0.0f),
    d_max_sq_ (0.0f),
    angle_step_ (0.0f),
    L_ (L),
    L_eps_ (L_eps),
    P_s_ (P_s),
    K_ (0.0f),
    m_ (0.0f),
    C_ (C),
    N_ (0),
    t_V_ (t_V),
    t_P_ (t_P),
    device_ (dev),
    focallength_ (fl),
    H_ (new FeatureHashMap<PointT>),
    cloud_sensor_ (new Cloud),
    cloud_input_ (boost::none),
    cloud_scene_ (boost::none),
    cloud_scene_reduced_ (boost::none),
    range_image_scene_ (boost::none),
    sensor_pose_ (boost::none),
    oct_centroid_ (new pcl::octree::OctreePointCloudVoxelCentroid<PointT> (L)),
    oct_search_(new pcl::octree::OctreePointCloudSearch<PointT> (L)),
    conflict_graph_ (boost::none),
    hypotheses_ (new RegistrationHypotheses ()),
    solutions_ (boost::none),
    model_cache_ (new ModelCache<PointT> ()),
    updated_cloud_ (false),
    ready_ (true),
    pub_scene_ (boost::none),
    pub_scene_reduced_ (boost::none),
    pub_range_image_ (boost::none),
    pub_sensor_pose_ (boost::none),
    pub_conflict_graph_ (boost::none),
    pub_solutions_ (boost::none)
  {
    hypotheses_->clear ();

    static bool seeded =  false;
    if (!seeded)
    {
        srand ((unsigned int) pcl::getTime ());
        seeded = true;
    }

    if (device_ == FSR_DEV_NONE)
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
    //else if (dev == FSR_DEV_OPENNI}) {}
  }

  template <typename PointT>
  void FSRRecognition<PointT>::spin ()
  {
    spin_thread_ = boost::thread (&FSRRecognition<PointT>::spinThread, this);
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
    if (cloud_mutex_.try_lock ())
    {
      if (updated_cloud_)
      {
        cloud_input_= cloud_sensor_;
        updated_cloud_ = false;
        ready_ = false;
      }
      cloud_mutex_.unlock ();
    }

    /// recognition
    background_group_.wait ();
    intializeRecognition ();
    tbb::parallel_for(size_t(0), static_cast<size_t>(N_),
                     [&] (size_t i)
    {
      generateHypotheses (i);
    });
    removeConflictingHypotheses ();

    /// give user results, cleanup, and get ready for the next run
    if (N_ > 0)
    {
      pub_scene_ = cloud_scene_;
      pub_scene_reduced_ = cloud_scene_reduced_;
      pub_range_image_ = range_image_scene_;
      pub_sensor_pose_ = sensor_pose_;
      pub_conflict_graph_ = conflict_graph_;
      pub_solutions_ = solutions_;
      publish ();

      cleanup();
      background_group_.run ([&]
      {
        model_cache_->clean ();
        model_cache_->record ();
      });

      #if FSR_RECOGNITION_DEBUG
      dbg_file.close();
      #endif // FSR_RECOGNITION_DEBUG

      boost::lock_guard<BoostMutex> lock (cloud_mutex_);
      ready_ = true;
    }
  }

  template <typename PointT>
  void FSRRecognition<PointT>::intializeRecognition ()
  {
    if (!cloud_input_)
    {
      return;
    }

    cloud_scene_ = cloud_input_;
    CloudPtr temp_scene (new Cloud);
    pcl::copyPointCloud (*(*cloud_scene_), *temp_scene);

    scene_resolution_ = computeCloudResolution<PointT> (temp_scene);

    /// compute the range image used for evaluating hypotheses
    range_image_scene_ = RangeImagePtr (new RangeImage ());
    sensor_pose_ = Eigen::Affine3f (Eigen::Translation3f((*cloud_scene_)->sensor_origin_[0],
                                    (*cloud_scene_)->sensor_origin_[1],
                                    (*cloud_scene_)->sensor_origin_[2])) *
                                    Eigen::Affine3f((*cloud_scene_)->sensor_orientation_);
    getRangeImage(temp_scene, *range_image_scene_, *sensor_pose_, focallength_);

    /// reduce the scene cloud for point sampling
    cloud_scene_reduced_ = CloudPtr (new Cloud ());
    oct_centroid_->setInputCloud (*cloud_scene_);
    oct_centroid_->addPointsFromInputCloud ();
    typename pcl::octree::OctreePointCloud<PointT>::AlignedPointTVector centroids;
    (*cloud_scene_reduced_)->is_dense = true;
    (*cloud_scene_reduced_)->width = oct_centroid_->getVoxelCentroids (centroids);
    (*cloud_scene_reduced_)->height = 1;
    (*cloud_scene_reduced_)->points.resize ((*cloud_scene_reduced_)->width);
    for (size_t i = 0; i < (*cloud_scene_reduced_)->width; ++i)
    {
      (*cloud_scene_reduced_)->points[i] = centroids[i];
    }
    /// set voxel half-length for hypothesis acceptance
    float L_space = sqrt (oct_centroid_->getVoxelSquaredSideLen ()) / 2.0;
    L_front_ = L_space - L_eps_;
    L_back_ = L_space + L_eps_;

    oct_centroid_->deleteTree();

    /// create the search tree from reduced scene
    oct_search_->setInputCloud (*cloud_scene_reduced_);
    oct_search_->addPointsFromInputCloud ();

    /// calcuate number of iterations
    N_ = static_cast<int> (ceil (-(static_cast<float> ((*cloud_scene_reduced_)->points.size ()) * log (1.0f - P_s_)) / (m_ * K_ * C_)));

    /// prepare structures for recognition
    conflict_graph_ = boost::shared_ptr<ConflictGraph> (new ConflictGraph ());
    solutions_ = RegistrationSolutionsPtr (new RegistrationSolutions ());

    #if FSR_RECOGNITION_DEBUG
    static int runs = -1;
    std::stringstream ss;
    ss << "fsr_recognition_dbginfo_" << ++runs << ".txt";
    dbg_file.open (ss.str());

    ss << "The scene cloud has " << (*cloud_scene_)->points.size () << " points.\n";
    ss << "The reduced scene cloud has " << (*cloud_scene_reduced_)->points.size () << " points.\n";
    ss << "It will take " << N_ << " RANSAC iterations to register the objects.\n\n";
    dbg_file << ss.str ();
    # endif // FSR_RECOGNITION_DEBUG

    #if FSR_RECOGNITION_VERBOSE
    std::cout << "[fsr_or::FSRRecognition::intializeRecognition()]: The scene cloud has " << (*cloud_scene_)->points.size () << " points." << std::endl;
    std::cout << "                                                  The reduced scene cloud has " << (*cloud_scene_reduced_)->points.size () << " points." << std::endl;
    std::cout << "                                                  It will take " << N_ << " RANSAC iterations to register the objects." << std::endl;
    std::cout << std::endl;
    #endif // FSR_RECOGNITION_VERBOSE
  }

  template <typename PointT>
  void FSRRecognition<PointT>::generateHypotheses (size_t iter_id)
  {
    if (N_ == 0)
    {
      return;
    }

    float f1, f2, f3, f4;

    int u, v, v_ind, n, tried;
    PointT pu, pv;
    std::vector<int> ushell;
    std::vector<float> uSqDists;
    bool foundpair = false, foundone = false;
    int poolnum = static_cast<int> ((*cloud_scene_reduced_)->points.size ());
    int chosen = 0;
    while (!foundpair && chosen < poolnum)
    {
      ushell.clear ();
      uSqDists.clear ();

      /// sample p_u
      u = rand() % poolnum;
      pu = (*cloud_scene_reduced_)->points[u];

      /// sample p_v from around p_u with distance [d_min_, d_max_]
      n = oct_search_->radiusSearch(pu, static_cast<double>(d_max_), ushell, uSqDists);
      tried = 0;
      while (!foundone && tried < n)
      {
        v_ind = rand() % n;
        v = ushell[v_ind];
        pv = (*cloud_scene_reduced_)->points[v];
        f1 = uSqDists[v_ind];
        foundone = (f1 < d_min_sq_ || f1 > d_max_sq_) ? false : true;
        ++tried;
      }

      foundpair = foundone;
      ++chosen;
    }

    #if FSR_RECOGNITION_DEBUG
    if (foundpair)
    {
      {
        DBGMutex::scoped_lock lock (dbg_out_mutex);
        std::stringstream ss;
        ss << "[fsr_or::FSRRecognition::generateHypotheses()] ID : " << iter_id  << " :\n"
           << chosen << " pair(s) were tried to find a point pair.\n";
        ss << "There were " << n << " other points around point u.\n";
        ss << "Point (" << pu.x << "," << pu.y << "," << pu.z << ")"
           << " was found around point (" << pv.x << "," << pv.y << "," << pv.z << ")"
           << " at a distance " << f1 << ".\n\n";
        dbg_file << ss.str ();
      }
    }
    else
    {
      {
        DBGMutex::scoped_lock lock (dbg_out_mutex);
        std::stringstream ss;
        ss << "[fsr_or::FSRRecognition::generateHypotheses()] ID : " << iter_id << " :\n"
           << chosen << " pairs were tried and no point pair was found.\n\n";
        dbg_file << ss.str ();
      }
    }
    #endif // FSR_RECOGNITION_DEBUG

    if (!foundpair)
    {
      /// TODO : wrap this in a mutex
      std::cout << "[fsr_or::FSRRecognition::generateHypotheses()]: WARNING! The whole scene cloud was sampled to find one good pair for RANSAC." << std::endl;
      std::cout << "                                                Check the distance between point pairs (d_min/d_max member variables) and make"  << std::endl;
      std::cout << "                                                sure points in the reduced scene cloud are closer than that distance." << std::endl;
      std::cout << std::endl;
      return;
    }

    /// compute the normals for p_u and p_v
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    ne.setInputCloud (*cloud_scene_reduced_);
    ne.setRadiusSearch (0.03);
    ne.setSearchMethod (tree);

    boost::shared_ptr<std::vector<int> > indexptr (new std::vector<int> ());
    NormalCloud normals_uv;
    indexptr->push_back (u);
    indexptr->push_back (v);
    ne.setIndices (indexptr);
    ne.compute (normals_uv);

    /// compute the feature for the sampled point pair
    PointPairSystem<PointT> pps_scene (PointPair<PointT> (pu, normals_uv[0], pv, normals_uv[1]));
    if (!computeOPFeature<PointT> (pps_scene.opp.pu, pps_scene.opp.pv,
                                   f1, f2, f3, f4,
                                   d_min_sq_, d_max_sq_))
    {
      #if FSR_RECOGNITION_DEBUG
      {
        DBGMutex::scoped_lock lock (dbg_out_mutex);
        std::stringstream ss;
        ss << "[fsr_or::FSRRecognition::generateHypotheses()] ID : " << iter_id  << " :\n"
           << "Something went wrong computing the feature for point pair " << pps_scene.opp.display() << ".\n\n";
        dbg_file << ss.str ();
      }
      #endif // FSR_RECOGNITION_DEBUG
      return;
    }
    int d2, d3, d4;
    discretizeOPFeatureVals (f2, f3, f4,
                             d2, d3, d4,
                             angle_step_);
    const FSRFeature feature (d2, d3, d4);

    /// get point pairs with same feature as pair (u,v)
    FeatureHashMapIterator omforfit = H_->find (feature);
    if (omforfit == H_->end ())
    {
      #if FSR_RECOGNITION_DEBUG
      {
        DBGMutex::scoped_lock lock (dbg_out_mutex);
        std::stringstream ss;
        ss << "[fsr_or::FSRRecognition::generateHypotheses()] ID : " << iter_id  << " :\n"
           << "There were no models with feature " << feature.print () << ".\n\n";
        dbg_file << ss.str ();
      }
      #endif // FSR_RECOGNITION_DEBUG
      return;
    }
    ObjectMap<std::vector<PointPairSystem<PointT> > > om_pairs (omforfit->second);

    /// compute the local coordinate system of F for (u,v)
    if (!computeF<PointT>(pps_scene.opp, pps_scene.F))
    {
      #if FSR_RECOGNITION_DEBUG
      {
        DBGMutex::scoped_lock lock (dbg_out_mutex);
        std::stringstream ss;
        ss << "[fsr_or::FSRRecognition::generateHypotheses()] ID : " << iter_id  << " :\n"
           << "F could not be computed for feature " << feature.print () << ".\n\n";
        dbg_file << ss.str ();
      }
      #endif // FSR_RECOGNITION_DEBUG
      return;
    }

    /// generate and test hypotheses
    tbb::parallel_for (tbb::blocked_range<KeyBoxIterator> (omkey_box_.begin (), omkey_box_.end ()),
                       [&] (const tbb::blocked_range<KeyBoxIterator> &r)
    {
      /// iterate over all models with computed feature
      float m;
      OMKey::Ptr key;
      std::vector<PointPairSystem<PointT> > mpairs;
      typename ObjectMap<std::vector<PointPairSystem<PointT> > >::iterator ompps_it;
      for (KeyBoxIterator it = r.begin (); it != r.end (); ++it)
      {
        key = (*it);

        ompps_it = om_pairs.find_model (*key);
        if (ompps_it == om_pairs.end ()) { continue; }

        mpairs = ompps_it->second;

        CloudPtr cloud_model (new pcl::PointCloud<PointT> ());
        {
          TBBMutex::scoped_lock lock (io_mutex_);
          m = model_cache_->getModel (key, cloud_model, scene_resolution_);
        }

        /// iterate over all point pairs in the model that have this feature
        tbb::parallel_for (tbb::blocked_range<PPSIterator> (mpairs.begin (), mpairs.end ()),
                           [&] (const tbb::blocked_range<PPSIterator> &r2)
        {
          MatrixT T;
          Cloud cloud_transformed;
          for (PPSIterator it2 = r2.begin (); it2 != r2.end (); ++it2)
          {
            /// transorm the model and add it to the scene
            T = MatrixT (pps_scene.F * (it2->F.inverse ()));
            pcl::transformPointCloud(*cloud_model, cloud_transformed, T);

            if (acceptHypothesis (cloud_transformed, m, key))
            {
              hypotheses_->insert (std::pair<OMKey::Ptr, std::pair<CloudPtr, MatrixT> > (key, std::pair<CloudPtr, MatrixT> (cloud_model, T)));
            }

            #if FSR_RECOGNITION_DEBUG
            {
              DBGMutex::scoped_lock lock (dbg_out_mutex);
              std::stringstream ss;
              ss << "[fsr_or::FSRRecognition::generateHypotheses()] ID : " << iter_id  << " :\n"
                 << "Model " << key->print() << " has " << m << " points.\n"
                 << "The computed transform for feature " << feature.print() << " is:\n"
                 << T.format(MatrixDisplayFmt)
                 << "\n\n";
              dbg_file << ss.str ();
            }
              #endif // FSR_RECOGNITION_DEBUG
          }
        });
      }
    });

    #if FSR_RECOGNITION_VERBOSE
    {
      DBGMutex::scoped_lock lock (dbg_out_mutex);
      std::cout << "Finished iteration " << iter_id << "." << std::endl << std::endl;
    }
    #endif // FSR_RECOGNITION_VERBOSE
  }

  template <typename PointT>
  bool FSRRecognition<PointT>::acceptHypothesis (Cloud &model, float m, OMKey::Ptr &id)
  {
    ConflictNode node (id);

    /// calculate the nmber of points that are visible and
    /// the number of points that occlude valid points
    int x, y;
    float xf, yf, range;
    int &m_v = node.m_v;
    int &m_p = node.m_p;
    boost::unordered_set<CNPoint> &explainedpts = node.explainedPoints;
    explainedpts.reserve (m/8);
    RangeImagePtr &ris = (*range_image_scene_);
    for (size_t i = 0; i < model.points.size (); ++i)
    {
      PointT &h = model.points[i];

      ris->getImagePoint (h.getVector3fMap (), xf, yf, range);
      ris->real2DToInt2D (xf, yf, x, y);

      if (!ris->isInImage (x,y)) { continue; }

      pcl::PointWithRange &s = ris->getPoint (x, y);

      if (!pcl_isfinite (s.range)) { continue; }

      if (range < s.range - L_front_)
      {
        ++m_p;
      }
      else if (range < s.range + L_back_)
      {
        ++m_v;
        explainedpts.insert (CNPoint (x, y));
      }
    }

    float u_V = static_cast<float> (m_v) / m;
    float u_P = static_cast<float> (m_p) / m;

    #if FSR_RECOGNITION_DEBUG
    {
        DBGMutex::scoped_lock lock (dbg_out_mutex);
        std::stringstream ss;
        ss << "[fsr_or::FSRRecognition::acceptHypothesis()] : \n"
           << "Model " << id->print() << " had " << u_V << " visibility and "
           << u_P << " occlusion of the scene.\n\n";
        dbg_file << ss.str ();
    }
    #endif // FSR_RECOGNITION_DEBUG

    bool accept = u_V > t_V_ && u_P < t_P_;
    if (accept)
    {
      (*conflict_graph_)->addNode (node);
    }
    return accept;
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
      tbb::parallel_for (tbb::blocked_range<size_t> (i + 1, (*conflict_graph_)->size (), 100),
                                                     [&] (const tbb::blocked_range<size_t> &r)
      {
        for (size_t j = r.begin (); j < r.end (); ++j)
        {
          (*conflict_graph_)->addEdge (i, j);
        }
      }, tbb::simple_partitioner ());
    });

    /// suppress hypotheses that conflict with better hypotheses
    /// check the left side of every edge in a parallel loop and then the right side of every edge in
    /// another parallel because of data races and every node is in an edge with another node only once
    tbb::parallel_for (tbb::blocked_range<size_t> (0, (*conflict_graph_)->edgeCapacity (), 10000),
                                                   [&] (const tbb::blocked_range<size_t> &r)
    {
      for (size_t i = r.begin (); i < r.end (); ++i)
      {
        (*conflict_graph_)->suppressNodeOnEdgeFrom (i);
      }
    }, tbb::simple_partitioner ());
    tbb::parallel_for (tbb::blocked_range<size_t> (0, (*conflict_graph_)->edgeCapacity (), 10000),
                                                   [&] (const tbb::blocked_range<size_t> &r)
    {
      for (size_t i = r.begin (); i < r.end (); ++i)
      {
        (*conflict_graph_)->suppressNodeOnEdgeTo (i);
      }
    }, tbb::simple_partitioner ());

    /// add the all the unsupressed hypotheses to the solution container
    std::pair<RegistrationHypothesesIterator, RegistrationHypothesesIterator> rhitr;
    for (size_t i = 0; i < (*conflict_graph_)->size (); ++i)
    {
      if (!((*conflict_graph_)->nodeSuppressed (i)))
      {
        OMKey::Ptr key;
        (*conflict_graph_)->getNodeID (i, key);
        rhitr = hypotheses_->equal_range (key);
        for (; rhitr.first != rhitr.second; ++rhitr.first)
        {
          (*solutions_)->push_back (RegSolEntry<PointT> (key,
                                                         rhitr.first->second.first,
                                                         rhitr.first->second.second));
        }
      }
    }

    #if FSR_RECOGNITION_DEBUG
    std::stringstream ss;
    ss << "There were " << (*conflict_graph_)->size () << " hypotheses.\n";
    ss << "Of these " << (*solutions_)->size () << " were chosen.\n";
    dbg_file << ss.str ();
    {
      DBGMutex::scoped_lock lock (dbg_out_mutex);
      std::cout << "There were " << (*conflict_graph_)->size () << " hypotheses." << std::endl;
      std::cout << "Of these " << (*solutions_)->size () << " were chosen." << std::endl;
    }
    #endif // FSR_RECOGNITION_DEBUG
  }

  template <typename PointT>
  void FSRRecognition<PointT>::cloudCallback (const CloudConstPtr &cloud)
  {
    boost::lock_guard<BoostMutex> lock (cloud_mutex_);
    cloud_sensor_ = cloud;
    updated_cloud_ = true;
    ready_ = false;
  }

  template <typename PointT>
  void FSRRecognition<PointT>::publish ()
  {
    if (pub_scene_reduced_)
    {
      Time timestamp = stamp2ptime ((*cloud_scene_)->header.stamp);
      for (size_t i = 0; i < callbacks_scene_reduced_.size (); ++i)
      {
        callbacks_scene_reduced_[i](*pub_scene_, *pub_scene_reduced_, timestamp);
      }
    }

    if (pub_range_image_)
    {
      Time timestamp = stamp2ptime ((*cloud_scene_)->header.stamp);
      for (size_t i = 0; i < callbacks_range_image_.size (); ++i)
      {
        callbacks_range_image_[i](*pub_range_image_, *pub_sensor_pose_, timestamp);
      }
    }

    if (pub_conflict_graph_)
    {
      Time timestamp = stamp2ptime ((*cloud_scene_)->header.stamp);
      for (size_t i = 0; i < callbacks_conflict_graph_.size (); ++i)
      {
        callbacks_conflict_graph_[i](*pub_conflict_graph_, timestamp);
      }
    }

    if (pub_solutions_)
    {
      Time timestamp = stamp2ptime ((*cloud_scene_)->header.stamp);
      for (size_t i = 0; i < callbacks_solutions_.size (); ++i)
      {
        callbacks_solutions_[i](*pub_solutions_, timestamp);
      }
    }
  }

  template <typename PointT>
  void FSRRecognition<PointT>::cleanup ()
  {
    N_ = 0;

    cloud_input_ = boost::none;
    cloud_scene_ = boost::none;
    cloud_scene_reduced_ = boost::none;
    range_image_scene_ = boost::none;
    sensor_pose_ = boost::none;
    oct_centroid_->deleteTree ();
    oct_search_->deleteTree ();
    conflict_graph_ = boost::none;
    hypotheses_->clear ();
    solutions_ = boost::none;
    pub_scene_ = boost::none;
    pub_scene_reduced_ = boost::none;
    pub_range_image_ = boost::none;
    pub_sensor_pose_ = boost::none;
    pub_conflict_graph_ = boost::none;
    pub_solutions_ = boost::none;
  }

  template <typename PointT>
  bool FSRRecognition<PointT>::ready ()
  {
    boost::lock_guard<BoostMutex> lock (cloud_mutex_);
    return ready_;
  }

  template <typename PointT>
  typename FSRRecognition<PointT>::Time FSRRecognition<PointT>::stamp2ptime (uint64_t stamp)
  {
    Time time_t_epoch (boost::gregorian::date (1970,1,1));
    return (time_t_epoch + boost::posix_time::microseconds (stamp));
  }

  template <typename PointT>
  uint64_t FSRRecognition<PointT>::getStamp ()
  {
    Time epoch (boost::gregorian::date(1970,1,1));
    Time current = boost::posix_time::microsec_clock::local_time ();
    return (static_cast<uint64_t> ((current - epoch).total_microseconds ()));
  }

}

template class fsr_or::FSRRecognition<pcl::PointXYZ>;
template class fsr_or::FSRRecognition<pcl::PointXYZRGBA>;
template class fsr_or::FSRRecognition<pcl::PointXYZRGB>;
