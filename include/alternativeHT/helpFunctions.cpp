#include "helpFunctions.h"


namespace uvdar{

    HelpFunctions::HelpFunctions(){

    }

    HelpFunctions::~HelpFunctions(){

    }



    void HelpFunctions::permute( std::vector<PointState> a, int l, int r, std::vector<std::vector<PointState>> & result) {
        // Base case
        if (l == r)
        {
            result.push_back(a);
        }else {
            // Permutations made
            for (int i = l; i <= r; i++) {
            
                // Swapping done
                std::swap(a[l], a[i]);
    
                // Recursion called
                permute(a, l + 1, r, result);
    
                // backtrack
                std::swap(a[l], a[i]);
            }
        }
    }

    void HelpFunctions::calcVariance(std::vector<PointState> & distVector) {

        double sizeVector = (double)distVector.size();
        double xAvg = 0.0;
        double yAvg = 0.0;  

        // std::cout << "Vector " <<std::endl;
        // for(auto k : distVector){
        //     std::cout << "{" << k.x << "," << k.y << "}";
        // }
        // std::cout << std::endl;

        // compute average
        for( const auto vect : distVector){
            xAvg = xAvg + vect.point.x; 
            yAvg = yAvg + vect.point.y;
        }

        xAvg = xAvg/sizeVector;
        yAvg = yAvg/sizeVector;
        // std::cout << "avg " << xAvg << ", " << yAvg << "\n";

        PointState variance;
        std::vector<PointState> sum;
        for(const auto vect : distVector){
            variance.point.x = ((vect.point.x - xAvg)*(vect.point.x - xAvg));
            variance.point.y = ((vect.point.y - yAvg)*(vect.point.y - yAvg));  
            // std::cout << "var x" << variance.x << ", " << variance.y << std::endl;
            sum.push_back(variance);
        }

        PointState p; 
        p.point.x = 0;
        p.point.y = 0; 
        for(auto k : sum){
            p.point.x = p.point.x + k.point.x; 
            p.point.y = p.point.y + k.point.y;
        }

        p.point.x = p.point.x / (double)sizeVector;
        p.point.y = p.point.y / (double)sizeVector;

        // std::cout << "Variance x " << p.x << " Variance y " << p.y <<std::endl;

    }

    // void HelpFunctions::polynomialRegression(const std::vector<PointState> sequence){
    //     std::vector<PointState> selectedPts;
    //     this->selectLastFourOnLEDs(sequence, selectedPts);
        
    // }

    void HelpFunctions::selectLastNDataPoints(std::vector<PointState> seq, std::vector<PointState> selectedPoints, int numerOfDataPoints ){
    //     std::cout << "HERE\n";
        if((int)seq.size() < numerOfDataPoints){
            ROS_ERROR("[UVDAR_BP_Tim]: The wanted number for the polynomial regression is higher than the sequence lenght!");
            return;
        }
        for(auto itSeq = seq.end(); itSeq != seq.begin(); --itSeq){
            if(itSeq->ledState){
                ros::Duration timeInsertedIterator(itSeq->insertTime.toSec()); 
                
                
                auto point = *itSeq;
                // point.insertTime = seq.end()[-1].insertTime - timeInsertedIterator;
                
                
                
                // itSeq->insertTime = (seq.end()[-1].insertTime + lastInserted);
                std::cout << point.insertTime << "The seq end " << seq.end()[-1].insertTime.toSec() << " it " << itSeq->insertTime << "\n"; 
               selectedPoints.push_back(point);
            }
            if((int)selectedPoints.size() == numerOfDataPoints){
                return;
            }
        }

    //     std::cout << " No polynomial of degree 3 can be build!\n";

    }

    std::pair<std::vector<PointState>*, Eigen::VectorXd> HelpFunctions::polynomialRegression(std::vector<PointState>* sequence, int numberOfDataPoints){
        
        std::vector<PointState> selectedPts;
        HelpFunctions::selectLastNDataPoints(*sequence, selectedPts, numberOfDataPoints);
        // Input data: x, y coordinates of a 2D trajectory at different times t
        Eigen::MatrixXd data(numberOfDataPoints, 3);
        
        for(int i = 0; i < (int)selectedPts.size(); ++i){
            data(i,0) = selectedPts[i].point.x;
            data(i,1) = selectedPts[i].point.y;
            data(i,2) = selectedPts[i].insertTime.toSec(); 
        }

        //  data << selectedPts;

        //  for(const auto k : data){
            //  std::cout << k;
        //  }
         // Define the degree of the polynomial to fit
        int degree = 2;
        // Build the design matrix
        int n = data.rows(); // number of data points
        int m = (degree+1)*(degree+2)/2; // number of polynomial terms
        Eigen::MatrixXd A(n, m);
        for (int i = 0; i < n; i++) {
            int k = 0;
            for (int j = 0; j <= degree; j++) {
                for (int l = 0; l <= j; l++) {
                    A(i, k) = pow(data(i, 0), j-l) * pow(data(i, 1), l);
                    k++;
                }
            }
        }
        // Define the response vector y
        Eigen::VectorXd y(n);
        y = data.col(2);
        
        // Compute the least-squares solution using QR decomposition
        Eigen::VectorXd coeffs(m);
        coeffs = A.householderQr().solve(y);

        return std::make_pair(sequence, coeffs);

    }

} // uvdar