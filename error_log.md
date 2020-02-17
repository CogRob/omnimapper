rename organized_feature_extraction with organized_segmentation_tbb
rename transform_helpers with transform_tools

> error: cannot find <gtsam/geometry/Sphere2.h>
	--> rename all Sphere2 with Unit3

> error: no member mutex in boost ... boost::mutex
	--> add #include <boost/thread/mutex.hpp> as a header file.

> error: no matching function for call to ‘gtsam::Unit3::retract(gtsam::Vector2&, gtsam::Unit3::CoordinatesMode) const’
   gtsam::Unit3 n_retracted = n_.retract (n_v, gtsam::Unit3::EXPMAP);
	--> gtsam::Unit3 n_retracted = n_.retract (n_v)
	--> (delete gtsam::Unit3::EXPMAP)

> error: could not convert ‘(& Eigen::DenseBase<Derived>::operator<<(const Scalar&) [
	with Derived = Eigen::Matrix<double, -1, 1>; Eigen::DenseBase<Derived>::Scalar = double](
		(*(const Scalar*)(&((Eigen::DenseCoeffsBase<Eigen::Matrix<double, -1, 1>, 1>*)(
			(Eigen::DenseBase<Eigen::Matrix<double, -1, 1> >*)(& n_error)))
			->Eigen::DenseCoeffsBase<Eigen::Matrix<double, -1, 1>, 1>::operator()(0))))
			.Eigen::CommaInitializer<Eigen::Matrix<double, -1, 1> >::operator,((*(const Scalar*)
			(&((Eigen::DenseCoeffsBase<Eigen::Matrix<double, -1, 1>, 1>*)
			((Eigen::DenseBase<Eigen::Matrix<double, -1, 1> >*)(& n_error)))
			->Eigen::DenseCoeffsBase<Eigen::Matrix<double, -1, 1>, 1>::operator()(1)))))
			->Eigen::CommaInitializer<Eigen::Matrix<double, -1, 1> >::operator,(d_error)’ 
			from 
			‘Eigen::CommaInitializer<Eigen::Matrix<double, -1, 1> >’ to ‘gtsam::Vector {aka Eigen::Matrix<double, -1, 1>}’
   return (gtsam::Vector (3) << n_error (0), n_error (1), d_error)
	--> change "return (gtsam::Vector (3) << n_error (0), n_error (1), d_error)" to 
		"gtsam::Vector g_v;
		 g_v << n_error (0), n_error (1), d_error;
		 return g_v;"

> error: invalid use of incomplete type ‘class gtsam::Plane<Point>’
	--> because the Plane is inherint using DerivedValue, but it is deleted in gtsam, so we need to add our own DerivedValue.h
	--> add #inclue <omnimapper/DerivedValue.h>

> error: XnOS.h: No such file or directory
	--> add ${OPENNI_INCLUDE_DIRS} to include_directories
	--> add ${OPENNI_LIBRARIES} to target_link_libraries

> error: incomplete type ‘gtsam::traits<gtsam::Plane<pcl::PointXYZRGBA> >’ used in nested name specifier
     return traits<T>::Equals(this->value_, genericValue2.value_, tol);
	--> 