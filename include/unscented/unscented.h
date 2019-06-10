#ifndef _UNSCENTED_H_
#define _UNSCENTED_H_
#include <Eigen/Core>

namespace unscented {

namespace e = Eigen;

struct measurement {
  e::VectorXd x;
  e::MatrixXd C;
} ;

struct measurement unscentedTransform(e::VectorXd x,e::MatrixXd Px, e::VectorXd (*fcn)(e::MatrixXd,double,double,double),double fleft,double fright, double fcenter);
  // Alpha = double(0.5);

}


#endif // _UNSCENTED_H_
