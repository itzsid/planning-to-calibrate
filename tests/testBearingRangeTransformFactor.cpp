/**
 *  @file  TestBearingRangeTransformFactor.cpp
 *  @brief Unit tests for BearingRangeTransformFactor Class
 *  @author Siddharth Choudhary
 */

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/TestableAssertions.h>
#include <BearingRangeTransformFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/bind.hpp>

using namespace std;
using namespace gtsam;

// make a realistic calibration matrix

// Create a noise model for the pixel error
static SharedNoiseModel model(noiseModel::Unit::Create(2));

// Convenience for named keys

typedef BearingRangeTransformFactor<Pose2, Point2, Pose2> TestBearingRangeTransformFactor;

/* ************************************************************************* */
TEST( BearingRangeTransformFactor, Constructor) {
  Key poseKey(1);
  Key pointKey(2);
  Key transformKey(3);

  gtsam::Rot2 bearing(0);
double range = 20;

  TestBearingRangeTransformFactor factor(poseKey, pointKey, transformKey, bearing, range, model);

  gtsam::Values initial;
  initial.insert(poseKey, gtsam::Pose2(0,0,0));
  initial.insert(transformKey, gtsam::Pose2(20,0,0));
  initial.insert(pointKey, gtsam::Point2(10,0));

  gtsam::NonlinearFactorGraph nlfg;

  noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas((Vector(3) << 0.2, 0.2, 0.1).finished());
  nlfg.add(PriorFactor<Pose2>(poseKey, Pose2(0, 0, 0), priorNoise));
  nlfg.add(factor);

  //nlfg.print("Factor graph\n");
  //initial.print("Initial values:\n");
  Values result = LevenbergMarquardtOptimizer(nlfg, initial).optimize();
  //result.print("\nFinal result:\n");

}

/* ************************************************************************* */

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */


