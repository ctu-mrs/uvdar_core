#include "unscented.h"

#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>

using namespace uvdar;

double angdiff(double x,double y){
  double d = y-x;
  if ((d>-M_PI)&&(d<M_PI)) return d;
  return fmod(d+M_PI,2*M_PI)-M_PI;
}

unscented::measurement unscented::unscentedTransform(e::VectorXd x,e::MatrixXd Px, const boost::function<e::VectorXd(e::VectorXd,e::VectorXd,int)> &fcn,double fleft,double fright, double fcenter, int camera_index){
  int L = x.rows();
  double W0=1.0/3.0;
  Eigen::MatrixXd X(L,2*L+1);
  X.leftCols(1) = x;
  X.rightCols(2*L).Ones(L,2*L);

  Eigen::VectorXd W(2*L+1);
  W =e::VectorXd::Ones(2*L+1)*((1-W0)/(2*L));
  W(0)=W0;

  e::MatrixXd sf = ((L/(1-W0))*Px).sqrt();

  for (int i=0; i<L; i++){
    X.col(i*2+1) = (x+(sf.row(i)).transpose());
    X.col(i*2+2) = (x-(sf.row(i)).transpose());// check
  }
  e::MatrixXd Y(6,2*L+1);
  
  e::VectorXd expFrequencies;
  if (fcenter>0){
    expFrequencies = e::VectorXd(3);
    expFrequencies << fleft,fright,fcenter;
  }
  else {
    expFrequencies = e::VectorXd(2);
    expFrequencies << fleft,fright;
  }
  
  for (int i=0; i<(1+2*L); i++){
    Y.col(i)=fcn(X.col(i),expFrequencies, camera_index); //this is weird, check please
  }
  int nan_index = -1;
  int nan_count = 0;
  for (int i=0; i<(1+2*L); i++){
    if (Y.col(i).array().isNaN().any()){
      nan_count++;
      nan_index = i;
    }
  }

  if (nan_count == 1){
    Y.col(nan_index) = Y.col(0);
  }
  if (false){
  /* if (true){ */
    std::cout << "unscented W: "<< std::endl;
    std::cout << W.transpose() << std::endl;
    std::cout << "unscented X: "<< std::endl;
    std::cout << X << std::endl;
    std::cout << "unscented Y: "<< std::endl;
    std::cout << Y << std::endl;
  }
  e::Vector3d mr;
    mr << 0,0,0;
  e::VectorXd y = Y*W;

  e::MatrixXd Ye = (Y-y.replicate(1,2*L+1));
  e::MatrixXd Py = ((Ye*W.asDiagonal())*Ye.transpose());

  for (int i=0; i<3; i++){
      if (y(3+i)>M_PI){
        y(3+i)=-2*M_PI+y(3+i);
    }
  }
  struct measurement output;
  output.x = y;
  output.C = Py;
  return output;
}

std::vector<Eigen::VectorXd> unscented::getSigmaPtsSource(e::VectorXd x,e::MatrixXd Px){
  int L = x.rows();
  double W0=1.0/3.0;
  std::vector<e::VectorXd> output;
  output.push_back(x);
  Eigen::VectorXd W(2*L+1);
  W =e::VectorXd::Ones(2*L+1)*((1-W0)/(2*L));
  W(0)=W0;
  e::MatrixXd sf = ((L/(1-W0))*Px).sqrt();
  for (int i=0; i<L; i++){
    output.push_back((x+(sf.row(i)).transpose()));
    output.push_back((x-(sf.row(i)).transpose()));
  }
  return output;
}

/* } */
