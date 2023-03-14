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

    // prepare for polynomial regression TODO: Maybe calculate the variance over the points for outlier rejection
    for(const auto point : *(prediction.seq)){
        if(point.ledState){
            x.push_back(point.point.x);
            y.push_back(point.point.y);
            time.push_back(point.insertTime);
        }
    }
    
    if(x.size() == 0) return false;

    auto x_reg = polyReg(x, time);
    auto y_reg = polyReg(y, time);

    prediction.xCoeff = x_reg.first;
    prediction.yCoeff = y_reg.first;
    prediction.rmse  = cv::Point2d(x_reg.second, y_reg.second);
    
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



std::pair<std::vector<double>,double> uvdar::ExtendedSearch::polyReg(const std::vector<double>& pixelCoordinate, const std::vector<ros::Time>& time){

    int order = polyOrder_;
    if(pixelCoordinate.size() < 10){
        order = 1; 
    }

    Eigen::MatrixXd design_mat(time.size(), order + 1);
	Eigen::VectorXd pixel_vect = Eigen::VectorXd::Map(&pixelCoordinate.front(), pixelCoordinate.size());
    std::vector<double> weights = calculateWeightVector(time);
    Eigen::VectorXd weight_vect = Eigen::VectorXd::Map(&weights.front(), weights.size());
    Eigen::VectorXd result(order+1);

    Eigen::MatrixXd weight_mat = weight_vect.asDiagonal();


    // fill the Design matrix
	for(int i = 0 ; i < (int)time.size(); ++i){
	    for(int j = 0; j < order + 1; ++j){
            if(j == 0) design_mat(i,j) = 1;
	        else design_mat(i, j) = pow(time[i].toSec(), j);
	    }
	}
    Eigen::MatrixXd weighed_design_mat = weight_mat * design_mat;
    Eigen::VectorXd weighted_pixel_vect = weight_mat * pixel_vect;

	// Solve for linear least square fit
	result = weighed_design_mat.householderQr().solve(weighted_pixel_vect);
    std::vector<double> coeff;
    for(int i = 0; i < result.size(); ++i){
        coeff.push_back(result[i]);
    }

    auto prediction = design_mat * result;
    double rmse = calcWeightedRMSE(prediction, pixel_vect, weight_vect);


    return std::make_pair(coeff, rmse);
}

std::vector<double> uvdar::ExtendedSearch::calculateWeightVector(const std::vector<ros::Time>& time){
    std::vector<double> weights;
    double referenceTime = time.end()[-1].toSec();
    for(int i = 0; i < (int)time.size(); ++i){
        double timeDist = referenceTime - time[i].toSec();
        double weight = exp(-decayFactor_*timeDist);
        weights.push_back(weight);
    }

    return weights;
}

double uvdar::ExtendedSearch::calcWeightedRMSE(const Eigen::VectorXd prediction, const Eigen::VectorXd pixel_vect, const Eigen::VectorXd weights){
    
    Eigen::VectorXd residuals = prediction - pixel_vect;

    double sum_squared_residuals = (weights.array().pow(2)*residuals.array().pow(2)).sum();
    double rmse = sqrt(sum_squared_residuals/(int)prediction.size());
    return rmse;
}


bool uvdar::ExtendedSearch::checkIfInsideEllipse(SeqWithTrajectory& seq, cv::Point2d& query_point){
    
    double result = pow( (query_point.x - seq.predicted.x) / seq.rmse.x, 2) + pow( (query_point.y - seq.predicted.y) / seq.rmse.y , 2);  
    if(result < 1){
        return true;
    }else if(result > 1){
        return false;
    }

    if(result == 1){
        std::cout << "Is on the ellipse" << std::endl;
        return true;
    }
    
} 