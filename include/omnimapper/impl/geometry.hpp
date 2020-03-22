#include <omnimapper/geometry.h>

template <typename PointT>
bool omnimapper::fusePlanarPolygonsXY(const pcl::PointCloud<PointT>& poly1,
                                      const pcl::PointCloud<PointT>& poly2,
                                      pcl::PointCloud<PointT>& poly_out) {
  // Load these into pointvectors
  PointVector poly1vec;
  poly1vec.resize(poly1.points.size());
  PointVector poly2vec;
  poly2vec.resize(poly2.points.size());

  for (size_t i = 0; i < poly1.points.size(); i++) {
    poly1vec[i].x = poly1.points[i].x;
    poly1vec[i].y = poly1.points[i].y;
    poly1vec[i].z = 0.0;
  }

  for (size_t i = 0; i < poly2.points.size(); i++) {
    poly2vec[i].x = poly2.points[i].x;
    poly2vec[i].y = poly2.points[i].y;
    poly2vec[i].z = 0.0;
  }

  for (size_t i = 1; i < poly1vec.size() - 1; i++) {
    pcl::PointXYZRGB pt1 = poly1vec[i];
    for (size_t j = i + 1; j < poly1vec.size(); j++) {
      if ((poly1vec[j].x == pt1.x) && (poly1vec[j].y == pt1.y) &&
          (poly1vec[j].z == pt1.z)) {
        poly1vec.erase(poly1vec.begin() + i);
        i = 1;
        j = i + 1;
        pt1 = poly1vec[1];
      }
    }
  }

  // printf ("Checking polygon of size: %d\n",poly2vec.size ());
  for (size_t i = 1; i < poly2vec.size() - 1; i++) {
    pcl::PointXYZRGB pt1 = poly2vec[i];
    for (size_t j = i + 1; j < poly2vec.size(); j++) {
      if ((poly2vec[j].x == pt1.x) && (poly2vec[j].y == pt1.y) &&
          (poly2vec[j].z == pt1.z)) {
        poly2vec.erase(poly2vec.begin() + i);
        i = 1;
        j = i + 1;
        pt1 = poly2vec[1];
      }
    }
  }

  boost::geometry::correct(poly1vec);
  boost::geometry::correct(poly2vec);

  // Check for self intersection
  // bool intersect1 = boost::geometry::intersects (poly1vec);
  // bool intersect2 = boost::geometry::intersects (poly2vec);
  // bool intersect3 = boost::geometry::intersects (poly1vec, poly2vec);

  std::vector<PointVector> union_pts;
  try {
    boost::geometry::union_(poly1vec, poly2vec, union_pts);
  } catch (boost::geometry::overlay_invalid_input_exception& e) {
    printf("PCL Polymerge: Exception!\n");
    union_pts.push_back(poly2vec);
    return (false);
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
