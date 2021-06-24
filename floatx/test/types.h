#ifndef FLOATX_TYPES_HPP_
#define FLOATX_TYPES_HPP_

#include <Eigen/Eigen>

#include "floatx.h"

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
}

#endif  // FLOATX_TYPES_HPP_
