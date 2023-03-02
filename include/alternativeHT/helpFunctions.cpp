#include "helpFunctions.h"



uvdar::HelpFunctions::HelpFunctions(){
}

uvdar::HelpFunctions::~HelpFunctions(){

}



void uvdar::HelpFunctions::selectPointsForRegressionAndDoRegression(SeqWithTrajectory & prediction, const int polyRegOrder){
        
    std::vector<double> x,y;
    std::vector<ros::Time> time;

    for(const auto point : *(prediction.seq)){
        if(point.ledState){
            x.push_back(point.point.x);
            y.push_back(point.point.y);
            time.push_back(point.insertTime);
        }
    }
    prediction.xCoeff = polyReg(x, time, polyRegOrder);
    prediction.yCoeff = polyReg(y, time, polyRegOrder);

}

std::vector<double> uvdar::HelpFunctions::polyReg(const std::vector<double>& pixelCoordinate, const std::vector<ros::Time>& time, const int order){

    Eigen::MatrixXd DesignMat(time.size(), order + 1);
	Eigen::VectorXd pixelMat = Eigen::VectorXd::Map(&pixelCoordinate.front(), pixelCoordinate.size());
    Eigen::VectorXd result(order+1);

    // fill the Design matrix
	for(int i = 0 ; i < (int)time.size(); ++i){
	    for(int j = 0; j < order + 1; ++j){
	        DesignMat(i, j) = pow(time[i].toSec(), j);
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


std::vector<cv::Point2d> uvdar::HelpFunctions::findOrthogonalVectorWithLength(const cv::Point2d vect, const double len){

    cv::Point2d orthoFirst;
    cv::Point2d orthoSecond; 
      
    std::vector<cv::Point2d> solutions;
    if(vect.x != 0){
        orthoFirst.y =  sqrt( pow(len,2) / (pow((vect.y/vect.x),2) + 1 ) );  
        orthoFirst.x = - ( orthoFirst.y * vect.y ) / vect.x;

        orthoSecond.y = - sqrt( pow(len,2) / (pow((vect.y/vect.x),2) + 1) );  
        orthoSecond.x = - ( orthoSecond.y * vect.y ) / vect.x;

    }else if(vect.y != 0){
        orthoFirst.x = sqrt( pow(len,2) / ( pow((vect.x / vect.y),2) + 1 ) );
        orthoFirst.y = - (orthoFirst.x * vect.x/vect.y);

        orthoSecond.x = - sqrt( pow(len,2) / ( pow((vect.x / vect.y),2) + 1 ) );
        orthoSecond.y = - (orthoSecond.x * vect.x/vect.y);
    }
    solutions.push_back(orthoFirst);
    solutions.push_back(orthoSecond);
    return solutions;
}


float uvdar::HelpFunctions::area(const cv::Point2d p1, const cv::Point2d p2, const cv::Point2d p3){
    return abs((p1.x*(p2.y-p3.y) + p2.x*(p3.y-p1.y)+ p3.x*(p1.y-p2.y))/2.0);
}
    
bool uvdar::HelpFunctions::isInside(const cv::Point2d p1, const cv::Point2d p2, const cv::Point2d p3, const cv::Point2d query){  
    /* Calculate area of triangle ABC */
    float A = HelpFunctions::area (p1, p2, p3);
    
    /* Calculate area of triangle PBC */ 
    float A1 = HelpFunctions::area (query, p2, p3);
    
    /* Calculate area of triangle PAC */ 
    float A2 = HelpFunctions::area (p1, query, p3);
    
    /* Calculate area of triangle PAB */  
    float A3 = HelpFunctions::area (p1, p2, query);

   /* Check if sum of A1, A2 and A3 is same as A */
   return (A == A1 + A2 + A3);
}