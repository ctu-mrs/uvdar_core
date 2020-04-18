#include "unscented.h"

#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>

/* using namespace unscented; */

/* namespace unscented { */
/* namespace e = Eigen; */

double angdiff(double x,double y){
  double d = y-x;
  if ((d>-M_PI)&&(d<M_PI)) return d;
  return fmod(d+M_PI,2*M_PI)-M_PI;
}

unscented::measurement unscented::unscentedTransform(e::VectorXd x,e::MatrixXd Px, const boost::function<e::VectorXd(e::VectorXd,e::VectorXd,int)> &fcn,double fleft,double fright, double fcenter, int camera_index){
  /* std::cout << "unscented x: " << x << std::endl; */
  // Alpha = double(0.5);
  int L = x.rows();
  /* L = size(x,1); */
  // kappa = double(10/(Alpha^2)-L)/2
  double W0=1.0/3.0;
  // kappa = 0;
  // lambda = double((Alpha^2)*(L+kappa)-L);
  /* X = Eigen::MatrixXf::Ones(L,2*L+1); */
  /* X(all,0) = x; */
  Eigen::MatrixXd X(L,2*L+1);
  X.leftCols(1) = x;
  X.rightCols(2*L).Ones(L,2*L);

  /* W = ((1-W0)/(2*L))*ones(2*L+1,1); */
  /* W(0,0 = W0; */
  Eigen::VectorXd W(2*L+1);
  W =e::VectorXd::Ones(2*L+1)*((1-W0)/(2*L));
  W(0)=W0;

  /* std::cout << "unscented W: "<< std::endl; */
  /* std::cout << W << std::endl; */

  // Wm = [W(1)+(1-1/Alpha);W(2:end)/Alpha]; Wc = [W/(Alpha^2)];
  /* std::cout << "Px: " << std::endl; */
  /* std::cout << Px << std::endl; */
  e::MatrixXd sf = ((L/(1-W0))*Px).sqrt();

  /* std::cout << "sf: " << std::endl; */
  /* std::cout << sf << std::endl; */
  /* std::cout << "sf a: " << std::endl; */
  /* std::cout << ((L/(1-(2.0/3.0)))*Px).sqrt() << std::endl; */
  /* std::cout << "sf b: " << std::endl; */
  /* std::cout << ((L/(1-(1.0/6.0)))*Px).sqrt() << std::endl; */
  for (int i=0; i<L; i++){
    X.col(i*2+1) = (x+(sf.row(i)).transpose());
    X.col(i*2+2) = (x-(sf.row(i)).transpose());// check
  }
  // tst = 1./X([3,6,9],:)
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
    /* std::cout << "unscented X[" << i << "]: " << X.col(i) << std::endl; */
    Y.col(i)=fcn(X.col(i),expFrequencies, camera_index); //this is weird, check please
  }
  std::cout << "unscented X: "<< std::endl;
  std::cout << X << std::endl;
  std::cout << "unscented Y: "<< std::endl;
  std::cout << Y << std::endl;
  e::Vector3d mr;
    mr << 0,0,0;
  /* mr = [0;0;0]; */
  for (int i=3; i<6; i++){
    if (abs(angdiff(Y(i),M_PI))<(M_PI/2)){
      /* Y.row(i)=mod(Y.row(i),2*M_PI); */
      Y.row(i) = Y.row(i).unaryExpr([](double x) { return fmod(x,(2*M_PI)); });
      mr(i-3)=1;
    }
  }
  e::VectorXd y = Y*W;
  /* if (norm(y(1:3)-Y(1:3,1))>1) */
  // if (true)
  // [Y(:,1),mean(Y(:,2:end)')', y, Y(:,1)-y] 
    // (Y)*(W)
    // lambda
    // Y(1:3,:)
    // W'
    // Y(1:3,:).*repmat(W',3,1);
    //
    /* X */
    /* Y.topRows(3); */
    /* y(1:3,:) */
  /* end */
  // 1./X([3,6,9],:)

  e::MatrixXd Ye = (Y-y.replicate(1,2*L+1));
  // [rad2deg(Y(5,:));rad2deg(Ye(5,:))]
  e::MatrixXd Py = ((Ye*W.asDiagonal())*Ye.transpose());

  for (int i=0; i<3; i++){
    if (mr(i)==1){
      if (y(3+i)>M_PI){
        y(3+i)=-2*M_PI+y(3+i);
      }
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
