#include <ros/console.h>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>
#include <boost/math/distributions/students_t.hpp>
#include <bits/stdc++.h>



namespace uvdar{

    struct PredictionStatistics{
        double time_pred = -1;
        bool poly_reg_computed = false;
        bool extended_search = false;
        std::vector<double> coeff;
        Eigen::VectorXd predicted_vals_past; 
        double mean_dependent;
        double mean_independent;
        double predicted_coordinate = -1;
        double confidence_interval = -1;
    };
    
    class ExtendedSearch{
        
        private: 
            double decay_factor_;

            /**
             * @brief calcuate weighted sum of squared residuals 
             * @param predictions predictions for each past coordinate
             * @param values x or y coordinates
             * @param weights weight vector
             * @return sum_squared_residuals 
             */
            double calcWSSR(const Eigen::VectorXd&, const std::vector<double>&, const std::vector<double>&);

        public:
            ExtendedSearch(double);
            ~ExtendedSearch();

            /**
             * @brief Calculate weighted polynomial regression 
             * @param coordinate dependent vector 
             * @param time independent vector 
             * @param weights weight vector
             * @return regression coefficients + predicted values for each data value
             */
            std::tuple<std::vector<double>, Eigen::VectorXd> polyReg(const std::vector<double>&, const std::vector<double>&, const std::vector<double>&, const int &);

            /**
             * @brief calculates normalized weight vector with exponential decay function
             * @param time vector
             * @return weight vector
             */
            std::vector<double> calcNormalizedWeightVect(const std::vector<double>&);

            /**
             * @brief calculate the weighted mean
             * @param values x or y coordinates
             * @param weights weight vector
             * @return double 
             */
            double calcWeightedMean(const std::vector<double>&, const std::vector<double>&);

            /**
             * @brief check if point lies within box 
             * @param query_point 
             * @param left_top 
             * @param right_bottom
             * @return true/false for succress 
             */
            bool isInsideBB(const cv::Point2d&, const cv::Point2d&, const cv::Point2d&);

            /**
             * @brief compute the confidence interval by computing the regression accuracy and multiplying with wanted t-quantil percentage
             * 
             * @param prediction_vals statistics values 
             * @param time vector of independent variable
             * @param values x or y coordinates
             * @param weighs weight vector
             * @param wanted_percentage wanted percentage for the t-quantil
             * @return -1, if not possible to compute CI from poly regression 
             * @return value, if interval can be computed
             */
            double confidenceInterval(const PredictionStatistics&, const std::vector<double>&, const std::vector<double>&, const std::vector<double>, const int&);
    };
} // uvdar