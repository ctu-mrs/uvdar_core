#include "extended_search.h"

namespace uvdar{
    
ExtendedSearch::ExtendedSearch(double decay_factor, int poly_order){
    decay_factor_ = decay_factor;
    default_poly_order_ = poly_order;
}

ExtendedSearch::~ExtendedSearch(){

}

std::vector<double> ExtendedSearch::calculateWeightVector(const std::vector<double>& time){
    std::vector<double> weights;
    double reference_time = time.end()[-1];
    for(int i = 0; i < (int)time.size(); ++i){
        double time_dist = reference_time - time[i];
        double weight = exp(-decay_factor_*time_dist);
        weights.push_back(weight);
    }

    return weights;
}

double ExtendedSearch::calcWeightedMean(const std::vector<double>& values, const std::vector<double>& weights){
    if(weights.size() != values.size()) return -1; 
        
    double weighted_sum = 0, weights_summed = 0;

    for(int i = 0; i < (int)values.size(); i++){
        weighted_sum += ( values[i]*weights[i]); 
        weights_summed += weights[i];
    }
    double weighted_mean = weighted_sum/ weights_summed;
    
    return weighted_mean; 
}

double ExtendedSearch::calcWSTD(const std::vector<double>& values, const std::vector<double>& weights, const double& mean){

    double ss = 0; 
    double weights_summed = 0;
    int weight_zero = 0;
    for(int i = 0; i < (int)values.size(); ++i){
        ss += (weights[i]*pow(values[i] - mean, 2));
        weights_summed += weights[i];
        if(weights[i] == 0.0) weight_zero++;
    }

    int non_zero_weight = (int)weights.size() - weight_zero;
    double denominator = ( (non_zero_weight - 1) * weights_summed ) / non_zero_weight ; 
    double w_std = sqrt(ss / denominator); 

    return w_std;
} 

std::vector<double> ExtendedSearch::calcPredictionVector(const std::vector<double>& coeff, const std::vector<double>& time){

    std::vector<double> predictions;
    for(const auto curr_t : time ){
        double predicted = 0;
        for(int k = 0; k < (int)coeff.size(); k++){
            predicted += coeff[k]*pow(curr_t, coeff[k]);
        }
        predictions.push_back(predicted);
    }

    return predictions; 
}

double ExtendedSearch::calcWMSE(const Eigen::VectorXd& predictions, const std::vector<double>& values, const std::vector<double>& weights){
    
    std::vector<double> squared_residuals; 
    for(int i = 0; i < (int)values.size(); i++){
        squared_residuals.push_back( pow( (predictions(i) - values[i]), 2 ) );
    }

    double w_sum_residuals = 0, w_sum = 0;
    for(int i = 0; i < (int)values.size(); i++){
        w_sum_residuals += weights[i]*squared_residuals[i];
        w_sum += weights[i];
    }
    double w_mse = w_sum_residuals / predictions.size() * w_sum;

    return w_mse; 
}



std::pair<std::vector<double>,Eigen::VectorXd> ExtendedSearch::polyReg(const std::vector<double>& coordinate, const std::vector<double>& time, const std::vector<double>& weights){

    int order = default_poly_order_;
    if(coordinate.size() < 10){
        order = 1; 
    }

    Eigen::MatrixXd design_mat(time.size(), order + 1);
	Eigen::VectorXd pixel_vect = Eigen::VectorXd::Map(&coordinate.front(), coordinate.size());
    Eigen::VectorXd weight_vect = Eigen::VectorXd::Map(&weights.front(), weights.size());
    Eigen::VectorXd result(order+1);

    Eigen::MatrixXd weight_mat = weight_vect.asDiagonal();


    // fill the Design matrix
	for(int i = 0 ; i < (int)time.size(); ++i){
	    for(int j = 0; j < order + 1; ++j){
            if(j == 0) design_mat(i,j) = 1;
	        else design_mat(i, j) = pow(time[i], j);
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


    return std::make_pair(coeff, prediction);
}

double ExtendedSearch::standardErrorPrediction( Eigen::VectorXd& predicted_past, const std::vector<double>& values, std::vector<double>& weights, const double& mean, const double& predicted_future){

    double w_mse = calcWMSE(predicted_past, values, weights);
    double standard_error_prediction = 0;

    int n = values.size();

    double nominator = pow(predicted_future - mean, 2);
    double denominator = 0;
    for(auto val : values){
        denominator += pow(val -mean, 2);
    }
    
    standard_error_prediction = sqrt(w_mse * ( 1 + 1/n + nominator / denominator) );


    return standard_error_prediction;
}


// T - DISTRIBUTION missing or multiply sse * 2 = 95 % confidence interval



bool ExtendedSearch::checkIfInsideEllipse(const cv::Point2d& center_point, const cv::Point2d& variance, const cv::Point2d& query_point){
    
    // double result = pow( (query_point.x - point.predicted.x) / point.ellipse.x, 2) + pow( (query_point.y - point.predicted.y) / point.ellipse.y , 2);  
    double result = pow( (query_point.x - center_point.x) / variance.x, 2) + pow( (query_point.y - center_point.y) / variance.y , 2);  

    if(result < 1){
        return true;
    }else if(result > 1){
        return false;
    }

    if(result == 1){
        return true;
    }
    return false; 
} 


} // uvdar