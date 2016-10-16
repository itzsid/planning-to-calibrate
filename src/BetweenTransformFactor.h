#ifndef BetweenTransformFACTOR_H
#define BetweenTransformFACTOR_H

/**
 * @file BetweenTransformFactor.h
 * @brief Derived from BetweenFactor, estimates the between transform in addition to transform to sensor pose
 * @author Siddharth Choudhary
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Lie.h>
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
class BetweenTransformFactor: public NoiseModelFactor3<VALUE, VALUE, TRANSFORM> {

public:

    typedef VALUE T;


private:

    typedef BetweenTransformFactor<VALUE, TRANSFORM> This; //Pose, Transform
    typedef NoiseModelFactor3<VALUE, VALUE, TRANSFORM> Base;

    VALUE measured_; /** The measurement */

    /** concept check by type */
    GTSAM_CONCEPT_LIE_TYPE(T)
    GTSAM_CONCEPT_TESTABLE_TYPE(T)

    public:

        // shorthand for a smart pointer to a factor
        typedef typename boost::shared_ptr<BetweenTransformFactor> shared_ptr;

    /** default constructor - only use for serialization */
    BetweenTransformFactor() {}

    /** Constructor */
    BetweenTransformFactor(Key key1, Key key2, Key key3, const VALUE& measured,
                           const SharedNoiseModel& model) :
        Base(model, key1, key2, key3), measured_(measured) {
    }

    virtual ~BetweenTransformFactor() {}

    /// @return a deep copy of this factor
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                    gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

    /** implement functions needed for Testable */

    /** print */
    virtual void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
        std::cout << s << "BetweenTransformFactor("
                  << keyFormatter(this->key1()) << ","
                  << keyFormatter(this->key2()) << ","
                  << keyFormatter(this->key3()) << ")\n";
        measured_.print("  measured: ");
        // Omnimapper complains about this line occasionally
        this->noiseModel_->print("  noise model: ");
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

            //d(poseTrans0)/d(p0) = H00;
            //d(poseTrans0)/d(transform) = H01;
            T poseTrans0 = p0.compose(transform, H00, H01);

            //d(poseTrans1)/d(p1) = H10;
            //d(poseTrans1)/d(transform) = H11;
            T poseTrans1 = p1.compose(transform, H10, H11);


            //d(hx)/d(poseTrans0) = H20;
            //d(hx)/d(poseTrans1) = H21;
            T hx = poseTrans0.between(poseTrans1, H20, H21); // h(x)

            //d(hx)/d(poseTrans0) = H20;
            //d(poseTrans0)/d(p0) = H00;
            //d(hx)/d(p0) = d(hx)/d(poseTrans0) * d(poseTrans0)/d(p0) = H20 * H00

            if(H1)
                *H1 = H20 * H00;

            //d(hx)/d(poseTrans1) = H21;
            //d(poseTrans1)/d(p1) = H10;
            //d(hx)/d(p1) = d(hx)/d(poseTrans1) * d(poseTrans1)/d(p1) = H21 * H10
            if(H2)
                *H2 = H21 * H10;

            //d(hx)/d(transform) = d(hx)/d(poseTrans0) * d(poseTrans0)/d(transform) + d(hx)/d(poseTrans1) * d(poseTrans1)/d(transform) = H20*H01 + H21*H11
            if(H3){
                *H3 = H20*H01 + H21*H11;

                //The following works too:
                //gtsam::Matrix M = -(hx.inverse().AdjointMap());
                //M = M+eye(3);
            }

            // manifold equivalent of h(x)-z -> log(z,h(x))
            return measured_.localCoordinates(hx);
        }
        else{
            T poseTrans0 = p0.compose(transform);
            T poseTrans1 = p1.compose(transform);
            T hx = poseTrans0.between(poseTrans1);
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
}; // \class BetweenTransformFactor


} /// namespace gtsam

#endif // BetweenTransformFACTOR_H
