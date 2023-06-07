#include "extended_search.h"

namespace uvdar
{

    ExtendedSearch::ExtendedSearch(double decay_factor, int poly_order)
    {
        decay_factor_ = decay_factor;
        default_poly_order_ = poly_order;
    }

    ExtendedSearch::~ExtendedSearch()
    {
    }

    std::tuple<std::vector<double>, Eigen::VectorXd> ExtendedSearch::polyReg(const std::vector<double> &coordinate, const std::vector<double> &time, const std::vector<double> &weights)
    {

        int order = default_poly_order_;
        int threshold_order = 2;
        if (coordinate.size() < 10 && threshold_order < order)
        {
            order = threshold_order;
        }
        Eigen::MatrixXd design_mat(time.size(), order + 1);
        Eigen::VectorXd pixel_vect = Eigen::VectorXd::Map(&coordinate.front(), coordinate.size());
        Eigen::VectorXd weight_vect = Eigen::VectorXd::Map(&weights.front(), weights.size());
        Eigen::VectorXd result(order + 1);

        Eigen::MatrixXd weight_mat = weight_vect.asDiagonal();

        // fill the Design matrix
        for (int i = 0; i < (int)time.size(); ++i)
        {
            for (int j = 0; j < order + 1; ++j)
            {
                if (j == 0)
                    design_mat(i, j) = 1;
                else
                    design_mat(i, j) = pow(time[i], j);
            }
        }

        Eigen::MatrixXd weighed_design_mat = weight_mat * design_mat;
        Eigen::VectorXd weighted_pixel_vect = weight_mat * pixel_vect;

        // Solve for weighted linear least squares fit
        result = weighed_design_mat.householderQr().solve(weighted_pixel_vect);
        std::vector<double> coeff;
        for (int i = 0; i < result.size(); ++i)
        {
            coeff.push_back(result[i]);
        }

        auto prediction = design_mat * result;

        return {coeff, prediction};
    }

    std::vector<double> ExtendedSearch::calcNormalizedWeightVect(const std::vector<double> &time)
    {
        std::vector<double> weights;
        double sum_weights = 0.0;
        double reference_time = time.end()[-1];
        for (int i = 0; i < (int)time.size(); ++i)
        {
            double time_dist = reference_time - time[i];
            double weight = exp(-decay_factor_ * time_dist);
            sum_weights += weight;
            weights.push_back(weight);
        }

        // normalize to sum up to 1
        for (auto &weight : weights)
        {
            weight /= sum_weights;
        }

        return weights;
    }

    double ExtendedSearch::calcWeightedMean(const std::vector<double> &values, const std::vector<double> &weights)
    {
        if (weights.size() != values.size())
            return -1;

        double weighted_sum = 0.0;

        for (int i = 0; i < (int)values.size(); i++)
        {
            weighted_sum += (values[i] * weights[i]);
        }

        double w_mean = weighted_sum;
        return w_mean;
    }

    double ExtendedSearch::calcWSTD(const std::vector<double> &values, const std::vector<double> &weights, const double &mean)
    {

        double ss = 0.0;
        int n = (int)values.size();
        for (int i = 0; i < n; ++i)
        {
            ss += (weights[i] * pow(values[i] - mean, 2));
        }

        if (n == 1)
            return -1.0;

        double w_std = sqrt(ss / (n - 1));
        return w_std;
    }

    double ExtendedSearch::calcWSSR(const Eigen::VectorXd &predictions, const std::vector<double> &values, const std::vector<double> &weights)
    {

        double sum_squared_residuals = 0;
        for (int i = 0; i < (int)values.size(); i++)
        {
            sum_squared_residuals += (weights[i] * pow((predictions(i) - values[i]), 2));
        }
        return sum_squared_residuals;
    }

    double ExtendedSearch::confidenceInterval(const PredictionStatistics &prediction_vals, const std::vector<double> &time, const std::vector<double> &values, const std::vector<double> weights, const int &wanted_percentage)
    {

        double w_ssr = calcWSSR(prediction_vals.predicted_vals_past, values, weights);

        const int n = (int)values.size();
        const int dof = n - (int)prediction_vals.coeff.size();

        // earlier caught - here to guarantee standalone functionality 
        if (prediction_vals.mean_independent == -1.0 || prediction_vals.mean_dependent == -1.0 || dof <= 0)
        {
            return -1;
        }

        double unb_estimate_error_var = w_ssr / dof;
        double var_time = 0.0;
        for (auto t : time)
        {
            var_time += pow((t - prediction_vals.mean_independent), 2);
        }

        double standard_error = sqrt(unb_estimate_error_var + (1 + 1 / n + ((prediction_vals.time_pred - prediction_vals.mean_independent) / var_time)));

        double percentage_scaled = (100.0 - double(wanted_percentage)) / 100.0;
        boost::math::students_t dist(dof);
        double t = quantile(complement(dist, percentage_scaled / 2.0));
        double conf_interval_prediction = t * standard_error;

        return conf_interval_prediction;
    }

    bool ExtendedSearch::isInsideBB(const cv::Point2d &query_point, const cv::Point2d &left_top, const cv::Point2d &right_bottom)
    {
        if (left_top.x <= query_point.x && query_point.x <= right_bottom.x && left_top.y <= query_point.y && query_point.y <= right_bottom.y)
        {
            return true;
        }
        return false;
    }

} // uvdar