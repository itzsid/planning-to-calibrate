/**
 * Matlab toolbox interface definition for gtsam_unstable
 */

// specify the classes from gtsam we are using
class gtsam::Vector6;
class gtsam::Vector3;
class gtsam::Point3;
class gtsam::Pose3;
class gtsam::Point2;
class gtsam::Pose2;
class gtsam::Rot2;
class gtsam::Rot3;

class gtsam::GaussianFactorGraph;
class gtsam::Values;
virtual class gtsam::noiseModel::Base;
virtual class gtsam::NonlinearFactor;
virtual class gtsam::NonlinearFactorGraph;
virtual class gtsam::NoiseModelFactor;

#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>

namespace gtsam{
#include <BearingRangeTransformFactor.h>

template<POSE, LANDMARK, TRANSFORM, ROTATION>
virtual class BearingRangeTransformFactor : gtsam::NoiseModelFactor {
  BearingRangeTransformFactor(size_t poseKey, size_t pointKey,  size_t transformKey, const ROTATION& measuredBearing, double measuredRange, const gtsam::noiseModel::Base* noiseModel);

  pair<ROTATION, double> measured() const;
  void print(string s) const;

  // enabling serialization functionality
  void serialize() const;
};

typedef gtsam::BearingRangeTransformFactor<gtsam::Pose2, gtsam::Point2, gtsam::Pose2, gtsam::Rot2> BearingRangeTransformFactor2D;


#include <BetweenTransformFactor.h>

template<POSE, TRANSFORM>
virtual class BetweenTransformFactor : gtsam::NoiseModelFactor {
  BetweenTransformFactor(size_t pose1Key, size_t pose2Key,  size_t transformKey, const POSE& measuredTransform,  gtsam::noiseModel::Base* noiseModel);

  POSE measured() const;
  void print(string s) const;

  // enabling serialization functionality
  void serialize() const;
};

typedef gtsam::BetweenTransformFactor<gtsam::Pose2, gtsam::Pose2> BetweenTransformFactorPose2;
typedef gtsam::BetweenTransformFactor<gtsam::Pose3, gtsam::Pose3> BetweenTransformFactorPose3;


#include <BetweenLandmarkTransformFactor.h>

template<POSE, TRANSFORM>
virtual class BetweenLandmarkTransformFactor : gtsam::NoiseModelFactor {
  BetweenLandmarkTransformFactor(size_t pose1Key, size_t pose2Key,  size_t transformKey, const POSE& measuredTransform,  gtsam::noiseModel::Base* noiseModel);

  POSE measured() const;
  void print(string s) const;
  // enabling serialization functionality
  void serialize() const;
};
typedef gtsam::BetweenLandmarkTransformFactor<gtsam::Pose2, gtsam::Pose2> BetweenLandmarkTransformFactorPose2;
typedef gtsam::BetweenLandmarkTransformFactor<gtsam::Pose3, gtsam::Pose3> BetweenLandmarkTransformFactorPose3;
}
