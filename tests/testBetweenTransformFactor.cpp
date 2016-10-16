/**
 *  @file  TestBetweenTransformFactor.cpp
 *  @brief Unit tests for TestBetweenTransformFactor Class
 *  @author Ruffin White
 *  @date Jan 2015
 */
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/TestableAssertions.h>
#include <BetweenTransformFactor.h>
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

// Create a noise model for the pixel error
static SharedNoiseModel model(noiseModel::Unit::Create(3));
static SharedNoiseModel model3(noiseModel::Unit::Create(6));

// Convenience for named keys

typedef BetweenTransformFactor<Pose2, Pose2> TestBetweenTransformFactor;
typedef BetweenTransformFactor<Pose3, Pose3> TestBetweenTransformFactor3;

//typedef NoiseModelFactor3<Pose2, Pose2, Pose2> TestNoiseModelFactor3;

/* ************************************************************************* */
Vector factorError2D(const Pose2& poseA, const Pose2& poseB, const Pose2& transform, const TestBetweenTransformFactor& factor) {
  return factor.evaluateError(poseA, poseB, transform);
}

/* ************************************************************************* */
TEST( BetweenTransformFactor, Constructor ) {
  // Create a factor
  Key poseAKey(1);
  Key poseBKey(2);
  Key transformKey(3);

  Pose2 value(1, 1, 0);

  TestBetweenTransformFactor factor(poseAKey, poseBKey, transformKey, value, model);
}

/* ************************************************************************* */
TEST( BetweenTransformFactor, Equals ) {
  // Create two identical factors and make sure they're equal
  Key poseAKey(1);
  Key poseBKey(2);
  Key transformKey(3);

  Pose2 value(1, 1, 0);
  Pose3 valueT(value);

  TestBetweenTransformFactor3 factor_1(poseAKey, poseBKey, transformKey, valueT, model);
  TestBetweenTransformFactor3 factor_2(poseAKey, poseBKey, transformKey, valueT, model);
}

/* ************************************************************************* */
TEST( BetweenTransformFactor, Error ) {
  // Create a factor
  Key poseAKey(1);
  Key poseBKey(2);
  Key transformKey(3);

  Pose2 pose1(0,0,0);
  Pose2 pose2(1.0,0,0);
  Pose2 transformVal(0,3,0);
  Pose2 measuredVal(1, 0, 0);

  Rot3 R    = eye(3);
  Point3 t1(0.0, 0.0, 0.0);
  Point3 t2(1.0, 0.0, 0.0);
  Point3 t3(5.0, 0.0, 0.0);
  Point3 m1(1.0, 0.0, 0.0);

  Pose3 pose1T(R, t1);
  Pose3 pose2T(R, t2);
  Pose3 transformValT(R, t3);
  Pose3 measuredValT(R, m1);

  TestBetweenTransformFactor factor(poseAKey, poseBKey, transformKey, measuredVal, model);
  TestBetweenTransformFactor3 factor3(poseAKey, poseBKey, transformKey, measuredValT, model);

  // Set the linearization point

  // Use the factor to calculate the error
  Vector actualError(factor.evaluateError( pose1, pose2, transformVal));

  // The expected error is 0.5 degree / UnitCovariance
  // The expected error is sqrt(2) = 1.414213562 meter / UnitCovariance
  Vector expectedError = (Vector(3) << 0, 0, 0).finished();

  // Verify we get the expected error
  CHECK(assert_equal(expectedError, actualError, 1e-9));

  Vector expectedErrorT(6);
  expectedErrorT << 0, 0, 0, 0, 0, 0;
  //Vector expectedErrorT (Vector << 0, 0, 0, 0, 0, 0);
  Vector actualErrorT(factor3.evaluateError( pose1T, pose2T, transformValT));

  CHECK(assert_equal(expectedErrorT, actualErrorT, 1e-9));
}

Pose2 poseA(0,0,0);
Pose2 poseB(1,1,1);
Pose2 transformValue(2,2,0.5);
Pose2 measuredValue(-0.560694, 2.31586, 1);

Pose3 poseAT = Pose3(Rot3(), Point3());
Pose3 poseBT = Pose3(Rot3(), Point3(1,1,1));
Pose3 transformValueT = Pose3(Rot3(), Point3(1,1,1));
Pose3 measuredValueT = Pose3(Rot3(), Point3(1,1,1));

Vector factorError(const Pose2& p0, const Pose2& p1, const Pose2& transform)
{
Pose2 poseTrans0 = p0.compose(transform);
Pose2 poseTrans1 = p1.compose(transform);
Pose2 hx = poseTrans0.between(poseTrans1);
return measuredValue.localCoordinates(hx);
}

Vector factorError3(const Pose3& p0, const Pose3& p1, const Pose3& transform)
{
    Pose3 poseTrans0 = p0.compose(transform);
    Pose3 poseTrans1 = p1.compose(transform);
    Pose3 hx = poseTrans0.between(poseTrans1);
    return measuredValueT.localCoordinates(hx);
}


//Point3 transform_to_(const Pose3& pose, const Point3& point) { return pose.transform_to(point); }
TEST(BetweenTransformFactor, Dfactor1)
{

    Key poseAKey(1);
    Key poseBKey(2);
    Key transformKey(3);
    TestBetweenTransformFactor factor1(poseAKey, poseBKey, transformKey, measuredValue, model);


    Matrix computed;
    factor1.evaluateError( poseA, poseB, transformValue, computed, boost::none, boost::none);
    Matrix numerical = numericalDerivative31(factorError, poseA, poseB, transformValue);
    EXPECT(assert_equal(numerical,computed,1e-8));

    TestBetweenTransformFactor3 factor2(poseAKey, poseBKey, transformKey, measuredValueT, model3);
    Matrix computedT;
    factor2.evaluateError( poseAT, poseBT, transformValueT, computedT, boost::none, boost::none);
    Matrix numericalT = numericalDerivative31(factorError3, poseAT, poseBT, transformValueT);
    EXPECT(assert_equal(numericalT,computedT,1e-8));

}

TEST(BetweenTransformFactor, Dfactor2)
{

    Key poseAKey(1);
    Key poseBKey(2);
    Key transformKey(3);
    TestBetweenTransformFactor factor1(poseAKey, poseBKey, transformKey, measuredValue, model);


    Matrix computed;
    factor1.evaluateError( poseA, poseB, transformValue, boost::none, computed, boost::none);
    Matrix numerical = numericalDerivative32(factorError, poseA, poseB, transformValue);
    EXPECT(assert_equal(numerical,computed,1e-8));

    TestBetweenTransformFactor3 factor2(poseAKey, poseBKey, transformKey, measuredValueT, model3);

    Matrix computedT;
    factor2.evaluateError( poseAT, poseBT, transformValueT, boost::none, computedT,  boost::none);
    Matrix numericalT = numericalDerivative32(factorError3, poseAT, poseBT, transformValueT);
    EXPECT(assert_equal(numericalT,computedT,1e-8));
}



TEST(BetweenTransformFactor, Dfactor3)
{

    Key poseAKey(1);
    Key poseBKey(2);
    Key transformKey(3);
    TestBetweenTransformFactor factor1(poseAKey, poseBKey, transformKey, measuredValue, model);


    Matrix computed;
    factor1.evaluateError( poseA, poseB, transformValue, boost::none, boost::none, computed);
    Matrix numerical = numericalDerivative33(factorError, poseA, poseB, transformValue);
    std::cout << computed << " " << numerical << std::endl;
    EXPECT(assert_equal(numerical,computed,1e-8));

    TestBetweenTransformFactor3 factor2(poseAKey, poseBKey, transformKey, measuredValueT, model3);

    Matrix computedT;
    factor2.evaluateError( poseAT, poseBT, transformValueT, boost::none, boost::none, computedT);
    Matrix numericalT = numericalDerivative33(factorError3, poseAT, poseBT, transformValueT);
    EXPECT(assert_equal(numericalT,computedT,1e-8));
}


//TEST( BetweenTransformFactor, Jacobian2D ) {
//  // Create a factor
//  Key poseKey(1);
//  Key pointKey(2);
//  Key transformKey(3);

//  gtsam::Rot2 bearing(-1.57079632679);
//  double range = 20;

//  TestBetweenTransformFactor factor(poseKey, pointKey, transformKey, bearing, range, model);


//  // Set the linearization point
//  Pose2 pose(-5.0, 5.0, 1.57079632679); //90 degrees
//  Point2 point(10.0, 10.0);
//  Pose2 transform(-5.0 + 1, 5.0 + 1, 0.01745329251); //one unit and degree offset

//  // Use the factor to calculate the Jacobians
//  Matrix H1Actual, H2Actual, H3Actual;
//  factor.evaluateError(pose, point, transform, H1Actual, H2Actual, H3Actual);

//  // Use numerical derivatives to calculate the Jacobians
//  Matrix H1Expected, H2Expected, H3Expected;
//  H1Expected = numericalDerivative11<Vector, Pose2>(boost::bind(&factorError2D, _1, point, transform, factor), pose);
//  H2Expected = numericalDerivative11<Vector, Point2>(boost::bind(&factorError2D, pose, _1, transform, factor), point);
//  H3Expected = numericalDerivative11<Vector, Pose2>(boost::bind(&factorError2D, pose, point, _1, factor), transform);

//  // Verify the Jacobians are correct
//  CHECK(assert_equal(H1Expected, H1Actual, 1e-9));
//  CHECK(assert_equal(H2Expected, H2Actual, 1e-9));
//  CHECK(assert_equal(H3Expected, H3Actual, 1e-9));
//}

///* ************************************************************************* */
//TEST( BetweenTransformFactor, Optimize2D ) {
//  // Create a factor
//  Key poseKey(1);
//  Key pointKey(2);
//  Key transformKey(3);

//  gtsam::Rot2 bearing(-1.57079632679);
//  double range = 20;

//  TestBetweenTransformFactor factor(poseKey, pointKey, transformKey, bearing, range, model);


//  // Set the linearization point
//  Pose2 pose(-5.0, 5.0, 1.57079632679); //90 degrees
//  Point2 point(10.0, 10.0);
//  Pose2 transform(-5.0 + 1, 5.0 + 1, 0.01745329251); //one unit and degree offset

//  gtsam::Values initial;
//  initial.insert(poseKey, pose);
//  initial.insert(transformKey, point);
//  initial.insert(pointKey, transform);

//  gtsam::NonlinearFactorGraph nlfg;

//  noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas((Vector(3) << 0.2, 0.2, 0.1));
//  nlfg.add(PriorFactor<Pose2>(poseKey, Pose2(-5.1, 5.1, 1.57079632679 + 0.01745329251), priorNoise));
//  nlfg.add(factor);

//  //nlfg.print("Factor graph\n");
//  //initial.print("Initial values:\n");
//  Values result = LevenbergMarquardtOptimizer(nlfg, initial).optimize();
//  //result.print("\nFinal result:\n");
//}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
