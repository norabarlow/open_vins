#ifndef FLOATX_TYPES_HPP_
#define FLOATX_TYPES_HPP_

#include "floatx.h"
#include "costable_0_0001.h"
#include "sintable_0_0001.h"

#define LERP(w, v1, v2) ((1.0 - (w)) * (v1) + (w) * (v2))

// TYPES

using f_ts = flx::floatx<8,50,double>;
using f_ekf = flx::floatx<8,23,double>;
using f_imu = flx::floatx<8,23,float>;

// arithmetic operations
namespace flx {
    // custom absolute value
    template<int E, int M, typename BE>
    floatx<E,M,BE> abs(floatx<E,M,BE> x) {
        return x * ((x>0) - (x<0));
    }

    // custom sqrt
    template<int E, int M, typename BE>
    floatx<E,M,BE> sqrt(floatx<E,M,BE> a) {
        return floatx<E,M,BE>(std::sqrt(float(a)));
        //floatx<E,M,BE> x = floatx<E,M,BE>(1.0);
        //floatx<E,M,BE> xnext;
        //floatx<E,M,BE> err;

        //floatx<E,M,BE> tol = 1e-6;
    
        //do {
        //    xnext = 0.5 * (x + a / x);
        //    err = flx::abs((x - xnext));
        //    x = xnext;
        //} while (err > tol);
        //return xnext;
    }

    // custom mod, assumes y is positive
    template<int E, int M, typename BE>
    floatx<E,M,BE> mod(floatx<E,M,BE> x, floatx<E,M,BE> y) {
        floatx<E,M,BE> fac = x < 0 ? floatx<E,M,BE>(-1) : floatx<E,M,BE>(1);
        return x-fac*floatx<E,M,BE>(y*int(x/y));
    }

    // custom isnan
    template<int E, int M, typename BE>
    bool isnan(floatx<E,M,BE> x) {
        return std::isnan(float(x));
    }

    template<int E, int M, typename BE>
    floatx<E,M,BE> real(floatx<E,M,BE> x) {
        return x;
    }

    template<int E, int M, typename BE>
    floatx<E,M,BE> imag(floatx<E,M,BE> x) {
        return floatx<E,M,BE>(0.0);
    }

    // custom trig functions
    template<int E, int M, typename BE>
    floatx<E,M,BE> sin(floatx<E,M,BE> x) {
        return floatx<E,M,BE>(std::sin(float(x)));
        //floatx<E,M,BE> fac = x < 0 ? floatx<E,M,BE>(-1) : floatx<E,M,BE>(1);
        //x = flx::mod(flx::abs(x), floatx<E,M,BE>(6.28318530718));
        //floatx<E,M,BE> i = x * floatx<E,M,BE>(10000.0);
        //int index = (int)i;
        //return fac * LERP(i - index,        /* weight      */
        //    sintable_0_0001[index],     /* lower value */
        //    sintable_0_0001[index + 1]  /* upper value */
        //    );
    }

    template<int E, int M, typename BE>
    floatx<E,M,BE> cos(floatx<E,M,BE> x) {
        return floatx<E,M,BE>(std::cos(float(x)));
        //x = flx::abs(x);
        //x = flx::mod(x, floatx<E,M,BE>(6.28318530718));
        //floatx<E,M,BE> i = x * floatx<E,M,BE>(10000.0);
        //int index = (int)i;
        //return LERP(i - index,        /* weight      */
        //    costable_0_0001[index],     /* lower value */
        //    costable_0_0001[index + 1]  /* upper value */
        //    );
    }

    template<int E, int M, typename BE>
    floatx<E,M,BE> acos(floatx<E,M,BE> x) {
        // TODO: update to use same precision
        return floatx<E,M,BE>(std::acos(float(x)));
    }

    template<int E, int M, typename BE>
    floatx<E,M,BE> atan2(floatx<E,M,BE> x, floatx<E,M,BE> y) {
        // TODO: update to use same precision
        return floatx<E,M,BE>(std::atan2(float(x), float(y)));
    }

    template<int E, int M, typename BE>
    floatx<E,M,BE> atan(floatx<E,M,BE> x) {
        // TODO: update to use same precision
        return floatx<E,M,BE>(std::atan(float(x)));
    }

    template<int E, int M, typename BE>
    floatx<E,M,BE> pow(floatx<E,M,BE> x, floatx<E,M,BE> y) {
        // TODO: update to use same precision
        return floatx<E,M,BE>(std::pow(float(x), float(y)));
    }

    template<int E, int M, typename BE>
    floatx<E,M,BE> pow(floatx<E,M,BE> x, int y) {
        // TODO: update to use same precision
        return floatx<E,M,BE>(std::pow(float(x), y));
    }

    template<int E, int M, typename BE>
    floatx<E,M,BE> log(floatx<E,M,BE> x) {
        // TODO: update to use same precision
        return floatx<E,M,BE>(std::log(float(x)));
    }
}

namespace std {
    // hash function
    template <int E, int M, typename BE>
    struct hash<flx::floatx<E,M,BE>> {
        std::size_t operator()(flx::floatx<E,M,BE> const& x) const {
            return std::hash<double>{}(double(x));
        }
    };

    template <typename T>
    struct is_floatx : std::integral_constant<
        bool,
        std::is_same<f_ts, typename std::remove_cv<T>::type>::value ||
        std::is_same<f_ekf, typename std::remove_cv<T>::type>::value ||
        std::is_same<f_imu, typename std::remove_cv<T>::type>::value
    > {};

    template<int E, int M, typename BE>
    flx::floatx<E,M,BE> sqrt(flx::floatx<E,M,BE> x) {
        return flx::sqrt(x);
    }

    template<int E, int M, typename BE>
    flx::floatx<E,M,BE> abs(flx::floatx<E,M,BE> x) {
        return flx::abs(x);
    }

    template<int E, int M, typename BE>
    flx::floatx<E,M,BE> log(flx::floatx<E,M,BE> x) {
        return flx::log(x);
    }
}

// string to floatx conversion
namespace CLI {
namespace detail {
    template <int E, int M, typename BE>
    bool lexical_cast (const std::string &input, flx::floatx<E,M,BE> &output) {
        output = flx::floatx<E,M,BE>(std::stod(input));
        return true;
    }
}  // namespace detail
}  // namespace CLI


// support for eigen library
namespace Eigen {
    template<int E, int M, typename BE>
    struct NumTraits<flx::floatx<E,M,BE>> : NumTraits<double>
    {
        typedef flx::floatx<E,M,BE> Real;
        typedef flx::floatx<E,M,BE> NonInteger;
        typedef flx::floatx<E,M,BE> Nested;
      
        enum {
            IsComplex = 0,
            IsInteger = 0,
            IsSigned = 1,
            RequireInitialization = 0,
            ReadCost = 1,
            AddCost = 3,
            MulCost = 3
        };
    };

    template<int E, int M, typename BE, typename BinaryOp>
    struct ScalarBinaryOpTraits<double,flx::floatx<E,M,BE>,BinaryOp> { typedef flx::floatx<E,M,BE> ReturnType;  };
    template<int E, int M, typename BE, typename BinaryOp>
    struct ScalarBinaryOpTraits<flx::floatx<E,M,BE>,double,BinaryOp> { typedef flx::floatx<E,M,BE>  ReturnType;  };
}

#endif  // FLOATX_TYPES_HPP_
