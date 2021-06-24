#include <iostream>
#include <Eigen/Eigen>

#include "floatx.hpp"

using fx = flx::floatx<8,23>;

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

namespace flx {
    // custom abs
    template<int E, int M>
    floatx<E,M> abs(floatx<E,M> x) {
        return x * ((x>0) - (x<0));
    }


    // custom sqrt
    template<int E, int M>
    floatx<E,M> sqrt(floatx<E,M> a) {
        //return std::sqrt(double(a));
        floatx<E,M> x = floatx<E,M>(1.0);
        floatx<E,M> xnext;
        floatx<E,M> err;

        floatx<E,M> tol = 1e-6;
    
        do {
            xnext = 0.5 * (x + a / x);
            err = flx::abs((x - xnext));
            x = xnext;
        } while (err > tol);
        return xnext;
    }
}

int main() {
    Eigen::Matrix<fx,3,3> m;
    m << fx(1), fx(2), fx(3),
         fx(4), fx(5), fx(6),
         fx(7), fx(8), fx(9);

    Eigen::Matrix<fx,3,1> v;
    v << fx(1), fx(2), fx(3);

    Eigen::Matrix<fx,3,1> m2 = m * v;
    Eigen::HouseholderQR<Eigen::Matrix<fx,Eigen::Dynamic,Eigen::Dynamic>> qr(m);
    Eigen::Matrix<fx,Eigen::Dynamic,Eigen::Dynamic> Q = qr.householderQ();

    std::cout << m << std::endl;
    std::cout << m2 << std::endl;
    std::cout << Q << std::endl;
}
