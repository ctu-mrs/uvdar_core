#ifndef _UNSCENTED_H_
#define _UNSCENTED_H_
#include <Eigen/Core>
#include <boost/function.hpp>

namespace uvdar {

  namespace e = Eigen;

  namespace unscented {

    struct measurement  {
      e::VectorXd x;
      e::MatrixXd C;
    };

    measurement unscentedTransform(e::VectorXd x,e::MatrixXd Px,  const boost::function<e::VectorXd(e::VectorXd,e::VectorXd,int)> &fcn,double fleft,double fright, double fcenter, int camera_index=-1);
    std::vector<e::VectorXd> getSigmaPtsSource(e::VectorXd x,e::MatrixXd Px);

  } //unscented

} //uvdar


#endif // _UNSCENTED_H_
