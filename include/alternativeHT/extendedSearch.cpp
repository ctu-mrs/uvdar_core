#include "extendedSearch.h"



uvdar::ExtendedSearch::ExtendedSearch(double decayFactor, int polyOrder){
    decayFactor_ = decayFactor;
    polyOrder_ = polyOrder;
}

uvdar::ExtendedSearch::~ExtendedSearch(){

}

bool uvdar::ExtendedSearch::doRegression(seqPointer& prediction, std::vector<double>& x, std::vector<double>& y, std::vector<ros::Time>& time ){

    auto x_reg = polyReg(x, time);
    auto y_reg = polyReg(y, time);

    auto lastPoint = &prediction->end()[-1];

    lastPoint->computedExtendedSearch = true;
    lastPoint->x_coeff = x_reg.first;
    lastPoint->y_coeff = y_reg.first;
    lastPoint->ellipse  = cv::Point2d(x_reg.second, y_reg.second);
    
    // if all coefficients are zero, the regression was not sucessfull 
    int xCount = 0, yCount = 0;
    for(auto coff : lastPoint->x_coeff){
        if(coff == 0.0 ){
            xCount++;
        }
    }
    for(auto coff : lastPoint->y_coeff){
        if(coff == 0.0 ){
            yCount++;
        }
    }
    if(yCount == (int)lastPoint->y_coeff.size() && xCount == (int)lastPoint->x_coeff.size()){
        return false;
    }

    return true;
}



std::pair<std::vector<double>,double> uvdar::ExtendedSearch::polyReg(const std::vector<double>& coordinate, const std::vector<ros::Time>& time){

    int order = polyOrder_;
    if(coordinate.size() < 10){
        order = 1; 
    }

    Eigen::MatrixXd design_mat(time.size(), order + 1);
	Eigen::VectorXd pixel_vect = Eigen::VectorXd::Map(&coordinate.front(), coordinate.size());
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
    double reference_time = time.end()[-1].toSec();
    for(int i = 0; i < (int)time.size(); ++i){
        double timeDist = reference_time - time[i].toSec();
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


bool uvdar::ExtendedSearch::checkIfInsideEllipse(PointState& point, cv::Point2d& query_point){
    
    double result = pow( (query_point.x - point.predicted.x) / point.ellipse.x, 2) + pow( (query_point.y - point.predicted.y) / point.ellipse.y , 2);  
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