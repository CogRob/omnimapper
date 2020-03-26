#include <omnimapper/geometry.h>
#include <pcl/surface/convex_hull.h>

template <typename PointT>
bool omnimapper::fusePlanarPolygonsXY(const pcl::PointCloud<PointT>& poly1,
                                      const pcl::PointCloud<PointT>& poly2,
                                      pcl::PointCloud<PointT>& poly_out) {
  PointVector poly1vec(poly1.points);
  PointVector poly2vec(poly2.points);

  // Load these into pointvectors
  /*
  PointVector poly1vec;
  poly1vec.resize (poly1.points.size ());
  PointVector poly2vec;
  poly2vec.resize (poly2.points.size ());




  for (size_t i = 0 ; i < poly1.points.size (); i++)
  {
    poly1vec[i].x = poly1.points[i].x;
    poly1vec[i].y = poly1.points[i].y;
    poly1vec[i].z = 0.0;
  }

  for (size_t i = 0 ; i < poly2.points.size (); i++)
  {
    poly2vec[i].x = poly2.points[i].x;
    poly2vec[i].y = poly2.points[i].y;
    poly2vec[i].z = 0.0;
  }

    for (size_t i = 1; i < poly1vec.size ()-1; i++)
  {
    pcl::PointXYZRGBA pt1 = poly1vec[i];
    for (size_t j = i+1; j < poly1vec.size (); j++)
    {
      if((poly1vec[j].x == pt1.x) && (poly1vec[j].y == pt1.y) && (poly1vec[j].z
  == pt1.z))
      {
        poly1vec.erase (poly1vec.begin ()+i);
        i = 1;
        j = i+1;
        pt1 = poly1vec[1];
      }
    }
  }

  //printf ("Checking polygon of size: %d\n",poly2vec.size ());
  for (size_t i = 1; i < poly2vec.size ()-1; i++)
  {
    pcl::PointXYZRGBA pt1 = poly2vec[i];
    for (size_t j = i+1; j < poly2vec.size (); j++)
    {
      if((poly2vec[j].x == pt1.x) && (poly2vec[j].y == pt1.y) && (poly2vec[j].z
  == pt1.z))
      {
        poly2vec.erase (poly2vec.begin ()+i);
        i = 1;
        j = i+1;
        pt1 = poly2vec[1];
      }
    }
  }
  */

  boost::geometry::correct(poly1vec);
  boost::geometry::correct(poly2vec);

  PointVector simple1;
  boost::geometry::simplify(poly1vec, simple1, 0.01);
  bool simple1_intersect = boost::geometry::intersects(simple1);
  if (simple1_intersect)
    printf("Simple1 intersects!\n");
  else
    printf("Simple1 no intersects\n");

  PointVector simple2;
  boost::geometry::simplify(poly2vec, simple2, 0.01);
  bool simple2_intersect = boost::geometry::intersects(simple2);
  if (simple2_intersect)
    printf("Simple2 intersects!\n");
  else
    printf("Simple2 no intersects\n");

  // Check for self intersection
  bool intersect1 = boost::geometry::intersects(poly1vec);
  bool intersect2 = boost::geometry::intersects(poly2vec);
  bool intersect3 = boost::geometry::intersects(poly1vec, poly2vec);
  bool i1 = boost::geometry::intersects(poly1.points);
  bool i2 = boost::geometry::intersects(poly2.points);

  if (i1) printf("I1\n");
  if (i2) printf("I2\n");
  if (intersect1) printf("Intersect1\n");
  if (intersect2) printf("Intersect2\n");
  if (intersect3) printf("Intersect3\n");

  std::vector<PointVector> union_pts;
  try {
    boost::geometry::union_(poly1vec, poly2vec, union_pts);
    // boost::geometry::union_ (poly1.points, poly2.points, union_pts);
  } catch (boost::geometry::overlay_invalid_input_exception& e) {
    printf("PCL Polymerge: Exception!\n");
    union_pts.push_back(poly2vec);
    return (false);
  }

  if (union_pts.size() != 1) {
    printf("PolyMerge: More than one output polygon!\n");
    // exit(1);
  }

  // pcl::PointCloud<PointT> output_cloud;
  // pcl::PointCloud<PointT> output_cloud_transformed;

  // output_cloud.points.resize (union_pts[0].size ());
  // output_cloud_transformed.points.resize (union_pts[0].size ());
  poly_out.resize(union_pts[0].size());

  PointT pt;
  pt.z = 0.0;
  for (size_t i = 0; i < union_pts[0].size(); i++) {
    pt.x = (union_pts[0])[i].x;
    pt.y = (union_pts[0])[i].y;
    // pt.z = 0.0;
    poly_out.points[i] = pt;
  }

  return (true);
}

template <typename PointT>
bool omnimapper::fusePlanarPolygonsConvexXY(
    const pcl::PointCloud<PointT>& poly1, const pcl::PointCloud<PointT>& poly2,
    pcl::PointCloud<PointT>& poly_out) {
  boost::shared_ptr<pcl::PointCloud<PointT> > merged_cloud(
      new pcl::PointCloud<PointT>());
  *merged_cloud += poly1;
  *merged_cloud += poly2;
  pcl::ConvexHull<PointT> hull;
  hull.setInputCloud(merged_cloud);
  hull.reconstruct(poly_out);
  return (true);
}
