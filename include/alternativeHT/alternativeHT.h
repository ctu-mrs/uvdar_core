#pragma once

#include "helpFunctions.h"
#include <iostream>
#include <list>
// #include <mutex>
#include <mrs_msgs/ImagePointsWithFloatStamped.h>
#include "signal_matcher/signal_matcher.h"



namespace uvdar
{



    class alternativeHT {
    
    private:
    
        bool debug_, visual_debug_ = false;
        
        const int max_pixel_shift_x_ = 2;
        const int max_pixel_shift_y_ = 2;
        const int boundingBox_x_Size_ = 3;
        const int boundingBox_y_Size_ = 3;
        double framerate_;


        std::vector<std::vector<bool>> originalSequences_;
        
        std::mutex mutex_generatedSequences_;
        std::list<std::vector<PointState>> generatedSequences_;

        std::unique_ptr<SignalMatcher> matcher_;


        void findClosestPixelAndInsert(std::vector<PointState>&);
        cv::Point2d computeXYDiff(const cv::Point2d, const cv::Point2d);
        void insertPointToSequence(std::vector<PointState> &, const PointState);


        // moving average approach 
        void expandedSearch(std::vector<PointState> &, std::vector<seqPointer>);
        bool checkSequenceValidityWithNewInsert(const seqPointer &);
        bool checkCoeffValidity(const SeqWithTrajectory &);
        void calculatePredictionTriangle(SeqWithTrajectory &, const ros::Time);
        void findOrthogonalVector(cv::Point2d);

        
        void checkSeqNewPointExpectation(std::vector<seqPointer>&);
        // bool bbIntersect(const PointState &, const PointState &);
        
        // depricated
        // std::vector<PointState>* findClosestWithinSelectedBB(std::vector<seqPointer>  , const PointState);
   



        void insertVPIfNoNewPointArrived(std::vector<PointState> &);
        void cleanPotentialBuffer();

        int findSequenceMatch(std::vector<bool>);

    public:

        alternativeHT();
        ~alternativeHT();
        
        void setDebugFlags(bool, bool);
        void setSequences(std::vector<std::vector<bool>>);
        void updateFramerate(double);

        void processBuffer(const mrs_msgs::ImagePointsWithFloatStampedConstPtr);

        std::vector<std::pair<PointState, int>> getResults();
        
        template <typename T>
        void printVectorIfNotEmpty(T vect, std::string name) {
            if (vect.empty())return;
            std::cout << "The " << name << " is: ";
            for (const auto r : vect){
                if (r) std::cout << "1,";
                else std::cout << "0,";
            }
            std::cout << std::endl;
        }

    };    
} // namespace uvdar