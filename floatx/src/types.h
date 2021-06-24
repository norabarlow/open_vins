#ifndef FLOATX_TYPES_HPP_
#define FLOATX_TYPES_HPP_

#include "floatx.h"
#include "costable_0_0001.h"
#include "sintable_0_0001.h"

#define LERP(w, v1, v2) ((1.0 - (w)) * (v1) + (w) * (v2))

// TYPES

using f_ts = flx::floatx<8,50>;
using f_ekf = flx::floatx<8,23>;
using f_imu = flx::floatx<8,23>;

// arithmetic operations
namespace flx {
    // custom absolute value
    template<int E, int M>
    floatx<E,M> abs(floatx<E,M> x) {
        return x * ((x>0) - (x<0));
    }

    // custom sqrt
    template<int E, int M>
    floatx<E,M> sqrt(floatx<E,M> a) {
        return floatx<E,M>(std::sqrt(float(a)));
        //floatx<E,M> x = floatx<E,M>(1.0);
        //floatx<E,M> xnext;
        //floatx<E,M> err;

        //floatx<E,M> tol = 1e-6;
    
        //do {
        //    xnext = 0.5 * (x + a / x);
        //    err = flx::abs((x - xnext));
        //    x = xnext;
        //} while (err > tol);
        //return xnext;
    }

    // custom mod, assumes y is positive
    template<int E, int M>
    floatx<E,M> mod(floatx<E,M> x, floatx<E,M> y) {
        floatx<E,M> fac = x < 0 ? floatx<E,M>(-1) : floatx<E,M>(1);
        return x-fac*floatx<E,M>(y*int(x/y));
    }

    // custom isnan
    template<int E, int M>
    bool isnan(floatx<E,M> x) {
        return std::isnan(float(x));
    }

    template<int E, int M>
    floatx<E,M> real(floatx<E,M> x) {
        return x;
    }

    template<int E, int M>
    floatx<E,M> imag(floatx<E,M> x) {
        return floatx<E,M>(0.0);
    }

    // custom trig functions
    template<int E, int M>
    floatx<E,M> sin(floatx<E,M> x) {
        return floatx<E,M>(std::sin(float(x)));
        //floatx<E,M> fac = x < 0 ? floatx<E,M>(-1) : floatx<E,M>(1);
        //x = flx::mod(flx::abs(x), floatx<E,M>(6.28318530718));
        //floatx<E,M> i = x * floatx<E,M>(10000.0);
        //int index = (int)i;
        //return fac * LERP(i - index,        /* weight      */
        //    sintable_0_0001[index],     /* lower value */
        //    sintable_0_0001[index + 1]  /* upper value */
        //    );
    }

    template<int E, int M>
    floatx<E,M> cos(floatx<E,M> x) {
        return floatx<E,M>(std::cos(float(x)));
        //x = flx::abs(x);
        //x = flx::mod(x, floatx<E,M>(6.28318530718));
        //floatx<E,M> i = x * floatx<E,M>(10000.0);
        //int index = (int)i;
        //return LERP(i - index,        /* weight      */
        //    costable_0_0001[index],     /* lower value */
        //    costable_0_0001[index + 1]  /* upper value */
        //    );
    }

    template<int E, int M>
    floatx<E,M> acos(floatx<E,M> x) {
        // TODO: update to use same precision
        return floatx<E,M>(std::acos(float(x)));
    }

    template<int E, int M>
    floatx<E,M> atan2(floatx<E,M> x, floatx<E,M> y) {
        // TODO: update to use same precision
        return floatx<E,M>(std::atan2(float(x), float(y)));
    }

    template<int E, int M>
    floatx<E,M> atan(floatx<E,M> x) {
        // TODO: update to use same precision
        return floatx<E,M>(std::atan(float(x)));
    }

    template<int E, int M>
    floatx<E,M> pow(floatx<E,M> x, floatx<E,M> y) {
        // TODO: update to use same precision
        return floatx<E,M>(std::pow(float(x), float(y)));
    }

    template<int E, int M>
    floatx<E,M> pow(floatx<E,M> x, int y) {
        // TODO: update to use same precision
        return floatx<E,M>(std::pow(float(x), y));
    }

    template<int E, int M>
    floatx<E,M> log(floatx<E,M> x) {
        // TODO: update to use same precision
        return floatx<E,M>(std::log(float(x)));
    }
}

namespace std {
    // hash function
    template <int E, int M>
    struct hash<flx::floatx<E,M>> {
        std::size_t operator()(flx::floatx<E,M> const& x) const {
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

    template<int E, int M>
    flx::floatx<E,M> sqrt(flx::floatx<E,M> x) {
        return flx::sqrt(x);
    }

    template<int E, int M>
    flx::floatx<E,M> abs(flx::floatx<E,M> x) {
        return flx::abs(x);
    }

    template<int E, int M>
    flx::floatx<E,M> log(flx::floatx<E,M> x) {
        return flx::log(x);
    }
}

// string to floatx conversion
namespace CLI {
namespace detail {
    template <int E, int M>
    bool lexical_cast (const std::string &input, flx::floatx<E,M> &output) {
        output = flx::floatx<E,M>(std::stod(input));
        return true;
    }
}  // namespace detail
}  // namespace CLI


// support for eigen library
namespace Eigen {
    template<int E, int M>
    struct NumTraits<flx::floatx<E,M>> : NumTraits<double>
    {
        typedef flx::floatx<E,M> Real;
        typedef flx::floatx<E,M> NonInteger;
        typedef flx::floatx<E,M> Nested;
      
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

    template<int E, int M, typename BinaryOp>
    struct ScalarBinaryOpTraits<double,flx::floatx<E,M>,BinaryOp> { typedef flx::floatx<E,M> ReturnType;  };
    template<int E, int M, typename BinaryOp>
    struct ScalarBinaryOpTraits<flx::floatx<E,M>,double,BinaryOp> { typedef flx::floatx<E,M>  ReturnType;  };
}

#endif  // FLOATX_TYPES_HPP_
