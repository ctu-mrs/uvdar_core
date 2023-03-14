#include "extendedSearch.h"



uvdar::ExtendedSearch::ExtendedSearch(double decayFactor, int polyOrder){
    decayFactor_ = decayFactor;
    polyOrder_ = polyOrder;
}

uvdar::ExtendedSearch::~ExtendedSearch(){

}

bool uvdar::ExtendedSearch::selectPointsForRegressionAndDoRegression(SeqWithTrajectory & prediction){
        
    std::vector<double> x,y;
    std::vector<ros::Time> time;

    for(const auto point : *(prediction.seq)){
        if(point.ledState){
            x.push_back(point.point.x);
            y.push_back(point.point.y);
            time.push_back(point.insertTime);
        }
    }
    
    if(x.size() == 0) return false;

    prediction.xCoeff = polyReg(x, time);
    prediction.yCoeff = polyReg(y, time);
    
    // if all coefficients are zero, the regression was not sucessfull 
    int xCount = 0, yCount = 0;
    for(auto coff : prediction.xCoeff){
        if(coff == 0.0 ){
            xCount++;
        }
    }
    for(auto coff : prediction.yCoeff){
        if(coff == 0.0 ){
            yCount++;
        }
    }
    if(yCount == (int)prediction.yCoeff.size() && xCount == (int)prediction.xCoeff.size()){
        return false;
    }

    return true;
}

std::vector<double> uvdar::ExtendedSearch::polyReg(const std::vector<double>& pixelCoordinate, const std::vector<ros::Time>& time){

    int order = polyOrder_;
    if(pixelCoordinate.size() < 10){
        order = 1; 
    }

    Eigen::MatrixXd DesignMat(time.size(), order + 1);
	Eigen::VectorXd pixelMat = Eigen::VectorXd::Map(&pixelCoordinate.front(), pixelCoordinate.size());
    Eigen::VectorXd result(order+1);

    double referenceTime = time.end()[-1].toSec(); 
    // fill the Design matrix
	for(int i = 0 ; i < (int)time.size(); ++i){
        double timeDist = referenceTime - time[i].toSec();
        double weight = exp(-decayFactor_*timeDist);
	    for(int j = 0; j < order + 1; ++j){
	        DesignMat(i, j) = pow(time[i].toSec(), j)*weight;
	    }
	}
	// Solve for linear least square fit
	result = DesignMat.householderQr().solve(pixelMat);
    std::vector<double> coeff;
    for(int i = 0; i < result.size(); ++i){
        coeff.push_back(result[i]);
    }

    return coeff;
}

