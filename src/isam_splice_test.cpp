#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

int main(int argc, char** argv) {
  gtsam::ISAM2 isam2;

  gtsam::Values new_values;
  gtsam::NonlinearFactorGraph new_graph;

  // Initial Pose and prior
  gtsam::Symbol init_sym('x', 0);
  gtsam::Pose3 init_pose = gtsam::Pose3(gtsam::Rot3::ypr(0.0, 0.0, 0.0),
                                        gtsam::Point3(0.0, 0.0, 0.0));
  gtsam::PriorFactor<gtsam::Pose3> pose_prior(
      init_sym, init_pose,
      gtsam::noiseModel::Diagonal::Sigmas(
          (gtsam::Vector(6) << 0.001, 0.001, 0.001, 0.001, 0.001, 0.001)));
  new_values.insert(init_sym, init_pose);
  new_graph.add(pose_prior);

  // First measurement, 1m in x
  gtsam::Symbol sym1('x', 1);
  gtsam::Pose3 movement1(gtsam::Rot3::identity(), gtsam::Point3(1.0, 0.0, 0.0));
  gtsam::SharedDiagonal noise1 = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0));
  gtsam::Pose3 pose1 = gtsam::Pose3(gtsam::Rot3::ypr(0.0, 0.0, 0.0),
                                    gtsam::Point3(1.0, 0.0, 0.0));
  gtsam::BetweenFactor<gtsam::Pose3> between1(init_sym, sym1, movement1,
                                              noise1);
  new_values.insert(sym1, pose1);
  new_graph.add(between1);

  // Add something to pull it out of line
  gtsam::Pose3 movement2(gtsam::Rot3::identity(), gtsam::Point3(1.1, 0.0, 0.0));
  gtsam::SharedDiagonal noise2 = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1));
  gtsam::BetweenFactor<gtsam::Pose3> between2(init_sym, sym1, movement2,
                                              noise2);
  new_graph.add(between2);

  new_values.print("Initial problem: ");

  // Optimize
  gtsam::ISAM2Result result = isam2.update(new_graph, new_values);
  gtsam::Values result_values = isam2.calculateEstimate();
  gtsam::NonlinearFactorGraph result_graph = isam2.getFactorsUnsafe();

  result_values.print("Optimized problem: ");
  printf("\n");
  result_graph.print("Result graph: ");
  printf("\n\n");

  // Test splice
  // Set up the new factors
  new_values.clear();
  new_graph = gtsam::NonlinearFactorGraph();
  gtsam::Symbol sym2('x', 2);
  gtsam::Pose3 movement02(gtsam::Rot3::identity(),
                          gtsam::Point3(0.5, 0.0, 0.0));
  gtsam::SharedDiagonal noise02 = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 0.5, 0.5, 0.5, 0.5, 0.5, 0.5));
  gtsam::Pose3 pose2 = gtsam::Pose3(gtsam::Rot3::ypr(0.0, 0.0, 0.0),
                                    gtsam::Point3(0.5, 0.0, 0.0));
  gtsam::BetweenFactor<gtsam::Pose3> between02(init_sym, sym2, movement02,
                                               noise02);
  gtsam::BetweenFactor<gtsam::Pose3> between21(sym2, sym1, movement02, noise02);
  new_values.insert(sym2, pose2);
  new_graph.add(between02);
  new_graph.add(between21);
  // Remove Factor with index 1
  gtsam::FastVector<size_t> to_remove;
  to_remove.push_back(1);

  gtsam::ISAM2Result result2 = isam2.update(new_graph, new_values, to_remove);
  gtsam::Values result_values2 = isam2.calculateEstimate();
  gtsam::NonlinearFactorGraph result_graph2 = isam2.getFactorsUnsafe();

  result_values2.print("Optimized problem2: ");
  printf("\n");
  result_graph2.print("Result graph2: ");
}
