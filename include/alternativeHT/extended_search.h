#include <ros/console.h>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>
#include <boost/math/distributions/students_t.hpp>
#include <bits/stdc++.h>



namespace uvdar{

    struct PredictionStatistics{
        double time_pred;
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
            int default_poly_order_;

            /**
             * @brief calcuate weighted sum of sqaured residuals 
             * 
             * @return sum_squared_residuals 
             */
            double calcWSSR(const Eigen::VectorXd&, const std::vector<double>&, const std::vector<double>&);

        public:
            ExtendedSearch(double, int);
            ~ExtendedSearch();

            /**
             * @brief Calculate weighted polynomial regression 
             * @param value vector 
             * @param time vector 
             * @param weight vector
             * @return PredictionStatistics with Coefficients + predicted values for each data value
             */
            PredictionStatistics polyReg(const std::vector<double>&, const std::vector<double>&, const std::vector<double>&);

            /**
             * @brief calculates weight vector with exponential decay function
             * @param time vector
             * @return weight vector
             */
            std::vector<double> calcNormalizedWeightVect(const std::vector<double>&);
            double calcWeightedMean(const std::vector<double>&, const std::vector<double>&);

            /**
             * @brief calculates the weighted standard deviation
             * @param values vector
             * @param weights vector
             * @param mean double
             * @return weighted standard deviation
             */
            double calcWSTD(const std::vector<double>&, const std::vector<double>&, const double&);

            /**
             * @brief check if point lies within box 
             * @param query_point 
             * @param left_top 
             * @param right_bottom
             * @return true/false for succress 
             */
            bool isInsideBB(const cv::Point2d&, const cv::Point2d&, const cv::Point2d&);
            double confidenceInterval(const PredictionStatistics&, const std::vector<double>&, const std::vector<double>&, const std::vector<double>, const int&);
    };
} // uvdar