#ifndef _UNSCENTED_H_
#define _UNSCENTED_H_
#include <Eigen/Core>
#include <boost/function.hpp>

namespace unscented {

namespace e = Eigen;

struct measurement  {
  e::VectorXd x;
  e::MatrixXd C;
}
;

measurement unscentedTransform(e::VectorXd x,e::MatrixXd Px,  const boost::function<e::VectorXd(e::VectorXd,e::VectorXd)> &fcn,double fleft,double fright, double fcenter);
  // Alpha = double(0.5);

}


#endif // _UNSCENTED_H_
