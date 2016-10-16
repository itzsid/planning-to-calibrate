#ifndef BetweenLandmarkTransformFACTOR_H
#define BetweenLandmarkTransformFACTOR_H

/**
 * @file BetweenLandmarkTransformFactor.h
 * @brief Derived from BetweenFactor, estimates the between transform in addition to transform to sensor pose
 * @author Varun Murali
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Lie.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Testable.h>
#include <boost/optional.hpp>
#include <ostream>

namespace gtsam {

/**
   * A class for a measurement predicted by "between(config[key1],config[key2])"
   * @tparam VALUE the Value type
   * @addtogroup SLAM
   */
template<class VALUE, class TRANSFORM>
class BetweenLandmarkTransformFactor: public NoiseModelFactor3<VALUE, VALUE, TRANSFORM> {

public:

    typedef VALUE T;


private:

    typedef BetweenLandmarkTransformFactor<VALUE, TRANSFORM> This; //Pose, Transform
    typedef NoiseModelFactor3<VALUE, VALUE, TRANSFORM> Base;

    VALUE measured_; /** The measurement */

    /** concept check by type */
    GTSAM_CONCEPT_LIE_TYPE(T)
    GTSAM_CONCEPT_TESTABLE_TYPE(T)

    public:

        // shorthand for a smart pointer to a factor
        typedef typename boost::shared_ptr<BetweenLandmarkTransformFactor> shared_ptr;

    /** default constructor - only use for serialization */
    BetweenLandmarkTransformFactor() {}

    /** Constructor */
    BetweenLandmarkTransformFactor(Key key1, Key key2, Key key3, const VALUE& measured,
                           const SharedNoiseModel& model) :
        Base(model, key1, key2, key3), measured_(measured) {
    }

    virtual ~BetweenLandmarkTransformFactor() {}

    /// @return a deep copy of this factor
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                    gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

    /** implement functions needed for Testable */

    /** print */
    virtual void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
        std::cout << s << "BetweenLandmarkTransformFactor("
                  << keyFormatter(this->key1()) << ","
                  << keyFormatter(this->key2()) << ","
                  << keyFormatter(this->key3()) << ")\n";
        measured_.print("  measured: ");
        // Omnimapper complains about this line occasionally
        this->noiseModel_->print("  noise model: ");
    }

    virtual void printError(const T& p0, const T& p1, const T& transform) const
    {
      Vector v = evaluateError(p0, p1, transform);
      std::cout << "Error: " << v << std::endl;
    }

    /** equals */
    virtual bool equals(const NonlinearFactor& expected, double tol=1e-9) const {
        const This *e =  dynamic_cast<const This*> (&expected);
        return e != NULL && Base::equals(*e, tol) && this->measured_.equals(e->measured_, tol);
    }

    /** implement functions needed to derive from Factor */

    /** vector of errors */
    Vector evaluateError(const T& p0, const T& p1, const TRANSFORM& transform,
                         boost::optional<Matrix&> H1 = boost::none,
                         boost::optional<Matrix&> H2 = boost::none,
                         boost::optional<Matrix&> H3 = boost::none) const
    {

        if(H1 || H2 || H3){
            Matrix H00, H01, H10, H11, H20, H21;
            boost::optional<Matrix&> H00_ = H1 ? boost::optional<Matrix&>(H00) : boost::optional<Matrix&>();
            boost::optional<Matrix&> H10_ = H1 ? boost::optional<Matrix&>(H10) : boost::optional<Matrix&>();
            boost::optional<Matrix&> H01_ = H2 ? boost::optional<Matrix&>(H01) : boost::optional<Matrix&>();
            boost::optional<Matrix&> H11_ = H2 ? boost::optional<Matrix&>(H11) : boost::optional<Matrix&>();
            boost::optional<Matrix&> H20_ = H3 ? boost::optional<Matrix&>(H20) : boost::optional<Matrix&>();
            boost::optional<Matrix&> H21_ = H3 ? boost::optional<Matrix&>(H21) : boost::optional<Matrix&>();

            //d(poseTrans0)/d(p0) = H00;
            //d(poseTrans0)/d(transform) = H01;
            T poseTrans0 = transform.compose (p0, H00, H01);

            //d(pose1)/d(p1) = H10;
            //d(pose1)/d(transform) = H11;
            T dPose1 = p1;

            //d(hx)/d(poseTrans0) = H20;
            //d(hx)/d(pose1) = H21;
            T hx = poseTrans0.between(dPose1, H20, H21);
           
            //T betTransPose = p1.between(p0);

            //d(hx)/d(poseTrans0) = H20;
            //d(PoseTrans0)/d(p0)   = H00;
            if (H1)
              //*H1 = H20 * H00;
              *H1 = -hx.inverse().AdjointMap(); 

            //d(hx)/d(pose1) = H21
            //d(pose1)/d(p1) = H10;
            if (H2)
            {
              if (boost::is_same<T, Pose3>::value) *H2 = eye(6);
              else *H2 = eye(3);
            }

            //d(hx)/d(transform) = d(hx)/d(poseTrans0) * d(poseTrans0)/d(transform)
            if (H3)
              //*H3 = *H20_ * *H01_;
              *H3 = -hx.inverse().AdjointMap();

            std::cout << "Error Landmark: " << measured_.localCoordinates(hx) << std::endl;

            return measured_.localCoordinates(hx);
        }
        else{
            T poseTrans0 = transform.compose(p0);
            T pose1      = p1;
            T hx         = poseTrans0.between(pose1);
            return measured_.localCoordinates(hx);
        }
    }

    /** return the measured */
    const VALUE& measured() const {
        return measured_;
    }

    /** number of variables attached to this factor */
    std::size_t size() const {
        return 3;
    }

private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
        ar & boost::serialization::make_nvp("NoiseModelFactor3",
                                            boost::serialization::base_object<Base>(*this));
        ar & BOOST_SERIALIZATION_NVP(measured_);
    }
}; // \class BetweenLandmarkTransformFactor


} /// namespace gtsam

#endif // BetweenLandmarkTransformFACTOR_H

