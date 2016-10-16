#ifndef BearingRangeTransformFACTOR_H
#define BearingRangeTransformFACTOR_H

/**
 * @file BearingRangeTransform.h
 * @brief Derived from ProjectionFactor, estimates the range and bearing of point in base frame in addition to transform to sensor pose
 * @author Varun Murali
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/concepts.h>
#include <gtsam/base/Testable.h>
#include <boost/optional.hpp>

namespace gtsam {

  /**
   * Non-linear factor for a constraint derived from a 2D measurement. The calibration is known here.
   * i.e. the main building block for visual SLAM.
   * @addtogroup SLAM
   */

template<class POSE, class LANDMARK, class TRANSFORM, class ROTATION = typename POSE::Rotation>
  class BearingRangeTransformFactor: public NoiseModelFactor3<POSE, LANDMARK, TRANSFORM> {
  protected:
    typedef POSE Pose;
    typedef ROTATION Rot;
    typedef LANDMARK Landmark;
    typedef TRANSFORM Transform;
    
    Rot measuredBearing_;
    double measuredRange_;
    // Keep a copy of measurement and calibration for I/O
    //Point2 measured_;                    ///< 2D measurement
    //boost::shared_ptr<CALIBRATION> K_;  ///< shared pointer to calibration object

  public:

    /// shorthand for base class type
    typedef NoiseModelFactor3<POSE, LANDMARK, TRANSFORM> Base;

    /// shorthand for this class
    typedef BearingRangeTransformFactor<POSE, LANDMARK, TRANSFORM>  This;

    /// shorthand for a smart pointer to a factor
    typedef boost::shared_ptr<This> shared_ptr;

    /// Default constructor

    BearingRangeTransformFactor() {}

    /**
     * Constructor
     * TODO: Mark argument order standard (keys, measurement, parameters)
     * @param measured is the 2 dimensional location of point in image (the measurement)
     * @param model is the standard deviation
     * @param poseKey is the index of the camera
     * @param pointKey is the index of the landmark
     * @param transformKey is the index of the body-camera transform
     */
    BearingRangeTransformFactor(Key poseKey, Key pointKey, Key transformKey,
                                const Rot& measuredBearing, const double measuredRange, const SharedNoiseModel& model):
          Base(model, poseKey, pointKey, transformKey), measuredBearing_(measuredBearing), measuredRange_(measuredRange)
           {}


    /** Virtual destructor */
    virtual ~BearingRangeTransformFactor() {}


    /// @return a deep copy of this factor
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

    /**
     * print
     * @param s optional string naming the factor
     * @param keyFormatter optional formatter useful for printing Symbols
     */

    /** Print */
    virtual void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
      std::cout << s << "BearingRangeTransformFactor("
          << keyFormatter(this->key1()) << ","
          << keyFormatter(this->key2()) << ","
          << keyFormatter(this->key3()) <<")\n";
      measuredBearing_.print("measured bearing: ");
      std::cout << "measured range: " << measuredRange_ << std::endl;
      this->noiseModel_->print("noise model:\n");
    }

    /// equals
    virtual bool equals(const NonlinearFactor& expected, double tol = 1e-9) const {
        const This *e =  dynamic_cast<const This*> (&expected);
        return e != NULL && Base::equals(*e, tol) &&
            fabs(this->measuredRange_ - e->measuredRange_) < tol &&
            this->measuredBearing_.equals(e->measuredBearing_, tol);
    }


    /// Evaluate error h(x)-z and optionally derivatives
    Vector evaluateError(const Pose& pose, const Landmark& point, const Transform& transform,
                         boost::optional<Matrix&> H1 = boost::none,
                         boost::optional<Matrix&> H2 = boost::none,
                         boost::optional<Matrix&> H3 = boost::none) const
    {
        if (H1 || H2 || H3)
        {
            Matrix H11, H21, H12, H13, H22, H23 ,H0, H02;
            boost::optional<Matrix&> H11_ = H1 ? boost::optional<Matrix&>(H11) : boost::optional<Matrix&>();
            boost::optional<Matrix&> H21_ = H1 ? boost::optional<Matrix&>(H21) : boost::optional<Matrix&>();
            boost::optional<Matrix&> H12_ = H2 ? boost::optional<Matrix&>(H12) : boost::optional<Matrix&>();
            boost::optional<Matrix&> H22_ = H2 ? boost::optional<Matrix&>(H22) : boost::optional<Matrix&>();
            boost::optional<Matrix&> H13_ = H3 ? boost::optional<Matrix&>(H13) : boost::optional<Matrix&>();
            boost::optional<Matrix&> H23_ = H3 ? boost::optional<Matrix&>(H23) : boost::optional<Matrix&>();

            //d(poseTransformed)/d(transform) = H02;
            //d(poseTransformed)/d(pose) = H0;
            Pose poseTransformed = pose.compose(transform, H0, H02);

            //d(y1)/d(pose_transformed) = H11_;
            //d(y1)/d(point) = H12_;
            Rot y1 = poseTransformed.bearing(point, H11_, H12_);
            Vector e1 = Rot::Logmap(measuredBearing_.between(y1));

            //d(y2)/d(pose_transformed) = H21_;
            //d(y2)/d(point) = H22_;
            double y2 = poseTransformed.range(point, H21_, H22_);
            Vector e2 = (Vector(1) << y2 - measuredRange_).finished();


            //d(y1)/d(pose) = d(y1)/d(pose_transformed) * d(poseTransformed)/d(pose) = H11_ * H0;
            //d(y1)/d(point) = H12_;
            //d(y1)/d(transform) = d(y1)/d(pose_transformed) *  d(poseTransformed)/d(transform) = H11_ * H02;
            H11 = *H11_ * H0;
            H12 = *H12_;
            H13 = *H11_ * H02;


            //d(y2)/d(pose) = d(y2)/d(pose_transformed) * d(poseTransformed)/d(pose) = H21_ * H0;
            //d(y2)/d(point) = H22_;
            //d(y2)/d(transform) = d(y2)/d(pose_transformed) *  d(poseTransformed)/d(transform) = H21_ * H02;
            H21 = *H21_ * H0;
            H22 = *H22_;
            H23 = *H21_ * H02;

            if (H1) *H1 = gtsam::stack(2, &H11, &H21);
            if (H2) *H2 = gtsam::stack(2, &H12, &H22);
            if (H3) *H3 = gtsam::stack(2, &H13, &H23);

            return concatVectors(2, &e1, &e2);

        }
        else
        {
            Pose poseTransformed = pose.compose(transform);
            Rot y1 = poseTransformed.bearing(point);
            Vector e1 = Rot::Logmap(measuredBearing_.between(y1));
            double y2 = poseTransformed.range(point);
            Vector e2 = (Vector(1) << y2 - measuredRange_).finished();
            return concatVectors(2, &e1, &e2);
         }
    }

    /** return the measurement */
    const std::pair<Rot, double> measured() const {
      return std::make_pair(measuredBearing_,measuredRange_);
    }

  private:


    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
      ar & boost::serialization::make_nvp("NoiseModelFactor3",
          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(measuredBearing_);
      ar & BOOST_SERIALIZATION_NVP(measuredRange_);
    }


  };
} // \ namespace gtsam

#endif // BearingRangeTransformFACTOR_H
