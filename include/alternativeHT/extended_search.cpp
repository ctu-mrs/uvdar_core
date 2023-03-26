#include "extended_search.h"

namespace uvdar{
    
ExtendedSearch::ExtendedSearch(double decay_factor, int poly_order){
    decay_factor_ = decay_factor;
    default_poly_order_ = poly_order;
}

ExtendedSearch::~ExtendedSearch(){

}

PredictionStatistics ExtendedSearch::polyReg(const std::vector<double>& coordinate, const std::vector<double>& time, const std::vector<double>& weights){

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

    PredictionStatistics results; 
    results.coeff = coeff;
    results.used_poly_order = order;
    results.predicted_vals_past = prediction;

    return results; 
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
        
    double weighted_sum = 0.0, weights_summed = 0.0;

    for(int i = 0; i < (int)values.size(); i++){
        weighted_sum += ( values[i]*weights[i]); 
        weights_summed += weights[i];
    }
    if(weights_summed == 0.0){
        return -1;
    }
    double w_mean = weighted_sum/ weights_summed;
    return w_mean; 
}

double ExtendedSearch::calcWSTD(const std::vector<double>& values, const std::vector<double>& weights, const double& mean){

    double ss = 0.0; 
    double weights_summed = 0.0;
    unsigned int weight_zero = 0;
    for(int i = 0; i < (int)values.size(); ++i){
        ss += (weights[i]*pow(values[i] - mean, 2));
        weights_summed += weights[i];
        if(weights[i] == 0.0) weight_zero++;
    }

    int non_zero_weight = (int)weights.size() - weight_zero;
    double denominator = 0.0; 
    
    if(non_zero_weight == 0.0 || weights_summed == 0.0){
        return -1.0;    
    } 
    
    if(non_zero_weight == 1.0 && weights.size() == 1.0){
        denominator = weights_summed;
    }else{
        denominator = ( (non_zero_weight - 1) * weights_summed ) / non_zero_weight; 
    }

    if(denominator == 0.0){
        return -1.0;
    }
    double w_std = sqrt(ss / denominator); 

    return w_std;
} 

double ExtendedSearch::calcWMSE(const Eigen::VectorXd& predictions, const std::vector<double>& values, const std::vector<double>& weights){
    
    std::vector<double> squared_residuals; 
    for(int i = 0; i < (int)values.size(); i++){
        squared_residuals.push_back( pow( (predictions(i) - values[i]), 2 ) );
    }

    double w_sum_residuals = 0.0, w_sum = 0.0;
    for(int i = 0; i < (int)values.size(); i++){
        w_sum_residuals += (weights[i]*squared_residuals[i]);
        w_sum += weights[i];
    }

    if(w_sum == 0.0 || (int)predictions.size() == 0){
        return -1;
    }
    

    double w_mse = w_sum_residuals / ( predictions.size()  * w_sum );// TODO: theoretically wrong
    // std::cout << "WMSE " << w_mse << " Resiudlas " << w_sum_residuals << " WEIGHTED SUM " << w_sum   << "\n";
    // if(w_sum_residuals < 2){
    // for(int i = 0; i < (int)values.size(); ++i){
    //     std::cout << "REG " << predictions(i) << "\tVAl" << values[i] << std::endl; 
    // }
    // }
    return w_mse; 
}





double ExtendedSearch::confidenceInterval(const PredictionStatistics& prediction_vals, const std::vector<double>& values, const std::vector<double> weights, const int& wanted_percentage){

    double w_mse = calcWMSE(prediction_vals.predicted_vals_past, values, weights);
    
    const int n = (int)values.size();
    
    if(w_mse == -1.0 || prediction_vals.mean == -1.0 || n == 0){
        return -1;
    }

    double standard_error_prediction = 0.0, denominator = 0.0;
    double nominator = pow(prediction_vals.predicted_coordinate - prediction_vals.mean, 2);
    for(auto val : values){
        denominator += pow(val -prediction_vals.mean, 2);
    }

    double percentage_scaled = (100.0 - double(wanted_percentage)) / 100.0;
    // std::vector<double> alpha = { 0.5, 0.25, 0.1, 0.05, 0.01, 0.001, 0.0001, 0.00001 };
    boost::math::students_t dist(n - prediction_vals.used_poly_order); // TODO: -1 
    double t = quantile(complement(dist, percentage_scaled / 2));

    if(denominator == 0){
        return -1;
    }
    
    standard_error_prediction = t*sqrt(w_mse * ( 1 + 1/n + nominator / denominator) );


    return standard_error_prediction;
}

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