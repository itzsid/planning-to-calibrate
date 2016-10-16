/**
 *  @file  TestBetweenLandmarkTransformFactor.cpp
 *  @brief Unit tests for TestBetweenLandmarkTransformFactor Class
 *  @author Varun Murali
 *  @date Feb 2016
 */
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/TestableAssertions.h>
#include <BetweenLandmarkTransformFactor.h>
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

typedef BetweenLandmarkTransformFactor<Pose2, Pose2> TestBetweenLandmarkTransformFactor;
typedef BetweenLandmarkTransformFactor<Pose3, Pose3> TestBetweenLandmarkTransformFactor3;

TestBetweenLandmarkTransformFactor factor;

//typedef NoiseModelFactor3<Pose2, Pose2, Pose2> TestNoiseModelFactor3;

/* ************************************************************************* */
Vector factorError2D(const Pose2& poseA, const Pose2& poseB, const Pose2& transform, const TestBetweenLandmarkTransformFactor& factor) {
  return factor.evaluateError(poseA, poseB, transform);
}


/* ************************************************************************* */
TEST( BetwennLandmarkTransformFactor, Constructor) {
  Key poseAKey(1);
  Key poseBKey(2);
  Key transformKey(3);

  Pose2 value(1,1,0);

  TestBetweenLandmarkTransformFactor factor(poseAKey, poseBKey, transformKey, value, model);
}

/* ************************************************************************* */
TEST( BetweenLandmarkTransformFactor, Equals ) {
  // Create two identical factors and make sure they're equal
  Key poseAKey(1);
  Key poseBKey(2);
  Key transformKey(3);

  Pose2 value(1, 1, 0);
  Pose3 valueT(value);

  TestBetweenLandmarkTransformFactor3 factor_1(poseAKey, poseBKey, transformKey, valueT, model);
  TestBetweenLandmarkTransformFactor3 factor_2(poseAKey, poseBKey, transformKey, valueT, model);
}

/* ************************************************************************* */
TEST( BetweenLandmarkTransformFactor, Error ) {
  // Create a factor
  Key poseAKey(1);
  Key poseBKey(2);
  Key transformKey(3);

  Pose2 pose1(0,0,0);
  Pose2 pose2(1,0,0);
  Pose2 transformVal(0,3,0);
  Pose2 measuredVal(1, -3, 0);

  Rot3 R    = eye(3);
  Point3 t1(0.0, 0.0, 0.0);
  Point3 t2(1.0, 0.0, 0.0);
  Point3 t3(0.0, 3.0, 0.0);
  Point3 m1(1.0, -3.0, 0.0);

  Pose3 pose1T(R, t1);
  Pose3 pose2T(R, t2);
  Pose3 transformValT(R, t3);
  Pose3 measuredValT(R, m1);

  TestBetweenLandmarkTransformFactor factor(poseAKey, poseBKey, transformKey, measuredVal, model);
  TestBetweenLandmarkTransformFactor3 factor3(poseAKey, poseBKey, transformKey, measuredValT, model);

  // Set the linearization point

  // Use the factor to calculate the error
  Vector actualError(factor.evaluateError( pose1, pose2, transformVal));

  // The expected error is 0.5 degree / UnitCovariance
  // The expected error is sqrt(2) = 1.414213562 meter / UnitCovariance
  Vector expectedError = (Vector(3) << 0, 0, 0).finished();

  // Verify we get the expected error
  CHECK(assert_equal(expectedError, actualError, 1e-9));

  //Vector expectedErrorT(6);
  //expectedErrorT << 0, 0, 0, 0, 0, 0;
  Vector expectedErrorT = (Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  Vector actualErrorT(factor3.evaluateError( pose1T, pose2T, transformValT));

  CHECK(assert_equal(expectedErrorT, actualErrorT, 1e-9));
}

Pose2 poseA(0,0,0);
Pose2 poseB(1,0,0);
Pose2 transformValue(0,3.0,0.0);
Pose2 measuredValue(1.0, -3.0, 0.0);

Pose3 poseAT = Pose3(Rot3(), Point3(0,0,0));
Pose3 poseBT = Pose3(Rot3(), Point3(1,0,0));
Pose3 transformValueT = Pose3(Rot3(), Point3(0,3,0));
Pose3 measuredValueT = Pose3(Rot3(), Point3(1,-3,0));

Vector factorError(const Pose2& p0, const Pose2& p1, const Pose2& transform)
{
Pose2 poseTrans0 = transform.compose(p0);
Pose2 pose1      = p1;
Pose2 hx = poseTrans0.between(pose1);
return measuredValue.localCoordinates(hx);
}

Vector factorError3(const Pose3& p0, const Pose3& p1, const Pose3& transform)
{
    Pose3 poseTrans0 = transform.compose(p0);
    Pose3 pose1 = p1;
    Pose3 hx = poseTrans0.between(pose1);
    return measuredValueT.localCoordinates(hx);
}

TEST(BetweenLandmarkTransformFactor, Dfactor1)
{

    Key poseAKey(1);
    Key poseBKey(2);
    Key transformKey(3);
    TestBetweenLandmarkTransformFactor factor1(poseAKey, poseBKey, transformKey, measuredValue, model);


    Matrix computed;
    factor1.evaluateError( poseA, poseB, transformValue, computed, boost::none, boost::none);
    Matrix numerical = numericalDerivative31(factorError, poseA, poseB, transformValue);
    EXPECT(assert_equal(numerical,computed,1e-8));

    TestBetweenLandmarkTransformFactor3 factor2(poseAKey, poseBKey, transformKey, measuredValueT, model3);
    Matrix computedT;
    factor2.evaluateError( poseAT, poseBT, transformValueT, computedT, boost::none, boost::none);
    Matrix numericalT = numericalDerivative31(factorError3, poseAT, poseBT, transformValueT);
    EXPECT(assert_equal(numericalT,computedT,1e-8));
}


TEST(BetweenLandmarkTransformFactor, Dfactor2)
{

    Key poseAKey(1);
    Key poseBKey(2);
    Key transformKey(3);
    TestBetweenLandmarkTransformFactor factor1(poseAKey, poseBKey, transformKey, measuredValue, model);


    Matrix computed;
    factor1.evaluateError( poseA, poseB, transformValue, boost::none, computed, boost::none);
    Matrix numerical = numericalDerivative32(factorError, poseA, poseB, transformValue);
    EXPECT(assert_equal(numerical,computed,1e-8));

    TestBetweenLandmarkTransformFactor3 factor2(poseAKey, poseBKey, transformKey, measuredValueT, model3);
    Matrix computedT;
    factor2.evaluateError( poseAT, poseBT, transformValueT, boost::none, computedT, boost::none);
    Matrix numericalT = numericalDerivative32(factorError3, poseAT, poseBT, transformValueT);
    EXPECT(assert_equal(numericalT,computedT,1e-8));
}

TEST(BetweenLandmarkTransformFactor, Dfactor3)
{

    Key poseAKey(1);
    Key poseBKey(2);
    Key transformKey(3);
    TestBetweenLandmarkTransformFactor factor1(poseAKey, poseBKey, transformKey, measuredValue, model);


    Matrix computed;
    factor1.evaluateError( poseA, poseB, transformValue, boost::none, boost::none, computed);
    Matrix numerical = numericalDerivative33(factorError, poseA, poseB, transformValue);
    EXPECT(assert_equal(numerical,computed,1e-8));

    TestBetweenLandmarkTransformFactor3 factor2(poseAKey, poseBKey, transformKey, measuredValueT, model3);
    Matrix computedT;
    factor2.evaluateError( poseAT, poseBT, transformValueT, boost::none, boost::none, computedT);
    Matrix numericalT = numericalDerivative33(factorError3, poseAT, poseBT, transformValueT);
    EXPECT(assert_equal(numericalT,computedT,1e-8));
}


int main() { TestResult tr; return TestRegistry::runAllTests(tr); }


