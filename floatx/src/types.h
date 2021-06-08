#ifndef FLOATX_TYPES_HPP_
#define FLOATX_TYPES_HPP_

#include "floatx.h"
#include "costable_0_0001.h"
#include "sintable_0_0001.h"

#define LERP(w, v1, v2) ((1.0 - (w)) * (v1) + (w) * (v2))

// arithmetic operations
namespace flx {
    // custom absolute value
    template<int E, int M>
    floatx<E,M> abs(floatx<E,M> x) {
        return x * ((x>0) - (x<0));
        //return floatx<E,M>(abs(double(x)));
    }

    // custom sqrt
    template<int E, int M>
    floatx<E,M> sqrt(floatx<E,M> a) {
        floatx<E,M> x = floatx<E,M>(1.0);
        floatx<E,M> xnext;
        floatx<E,M> err;

        floatx<E,M> tol = 1e-6;
    
        do {
            xnext = 0.5 * (x + a / x);
            // for example fabs(...) is not defined for the floatx type
            // hence, we use a cast to double and back to our type
            err = flx::abs((x - xnext));
            x = xnext;
        } while (err > tol);
        return xnext;
    }

    // custom mod
    template<int E, int M>
    floatx<E,M> mod(floatx<E,M> x, floatx<E,M> y) {
        return x-floatx<E,M>(y*int(x/y));
    }

    // custom trig functions
    template<int E, int M>
    floatx<E,M> sin(floatx<E,M> x) {
        x = flx::abs(x);
        x = flx::mod(x, floatx<E,M>(6.28318530718));
        floatx<E,M> i = x * floatx<E,M>(100.0);
        int index = (int)i;
        return LERP(i - index,        /* weight      */
            sintable_0_0001[index],     /* lower value */
            sintable_0_0001[index + 1]  /* upper value */
            );
    }

    template<int E, int M>
    floatx<E,M> cos(floatx<E,M> x) {
        x = flx::abs(x);
        x = flx::mod(x, floatx<E,M>(6.28318530718));
        floatx<E,M> i = x * floatx<E,M>(100.0);
        int index = (int)i;
        return LERP(i - index,        /* weight      */
            costable_0_0001[index],     /* lower value */
            costable_0_0001[index + 1]  /* upper value */
            );
    }
}

// hash function
namespace std {
    template <int E, int M>
    struct hash<flx::floatx<E,M>> {
        std::size_t operator()(flx::floatx<E,M> const& x) const {
            return std::hash<double>{}(double(x));
        }
    };
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

// TYPES

using f_ts = flx::floatx<8,50>;

#endif  // FLOATX_TYPES_HPP_
