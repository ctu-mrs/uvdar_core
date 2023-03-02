#include "alternativeHT.h"

#include <bits/stdc++.h>

using namespace uvdar;

alternativeHT::alternativeHT(){

    if(trajectory_logfile_){
        logFile_.open(filename_, std::ostream::out);
        logFile_ << "$ Logfile Structure: For each sequence all pixel with the xPixel,yPixel,insertTime are inserted.\nThe Format is: \"# \" for each point in the sequence: \" xPixel,yPixel,inserTime xPixel,..\" \"# xPixelPredict,yPixelPredict,inserTime # lenToPredictFromGroundPoint # triangle Values # xCoefficients # yCoefficients # Points that are not associated yet\" ";
        logFile_.close();
    }
}

void alternativeHT::setDebugFlags(bool debug, bool visual_debug){
    debug_ = debug;
    visual_debug_ = visual_debug;
}

void alternativeHT::setSequences(std::vector<std::vector<bool>> i_sequences){
  
    originalSequences_ = i_sequences;
    matcher_ = std::make_unique<SignalMatcher>(originalSequences_);
}

void alternativeHT::updateFramerate(double input) {
  if (input > 1.0)
    framerate_ = input;
}

void alternativeHT::processBuffer(const mrs_msgs::ImagePointsWithFloatStampedConstPtr ptsMsg) {
    
    std::vector<PointState> currentFrame;
    for ( auto pointWithTimeStamp : ptsMsg->points) {
        PointState p;
        p.point = cv::Point(pointWithTimeStamp.x, pointWithTimeStamp.y);
        p.ledState = true;
        p.insertTime = ptsMsg->stamp;
        currentFrame.push_back(p);
    }

    // std::cout << "Sequences:\n";
    // for( auto k : generatedSequences_){
    //     for(auto r : k){
    //         if(r.ledState) std::cout << "1,"; 
    //         else std::cout << "0,";
    //     }
    //     std::cout << "\n";
    // }
    findClosestPixelAndInsert(currentFrame);

    cleanPotentialBuffer();  // TODO: NOT WORKING!!!!!
  
}

void alternativeHT::findClosestPixelAndInsert(std::vector<PointState> & currentFrame) {   
    
    // reference to sequences used for processing in the moving average functions
    std::vector<seqPointer> pGenSequence;
    {
       std::scoped_lock lock(mutex_generatedSequences_);
        for(auto & seq : generatedSequences_){
            pGenSequence.push_back(&seq);
        }

    }

    std::vector<PointState> noNN;
    for(auto & currPoint : currentFrame){
      std::scoped_lock lock(mutex_generatedSequences_);
      bool nearestNeighbor = false;
      for(auto seq = pGenSequence.begin(); seq != pGenSequence.end(); ++seq){
          PointState lastInserted = (*seq)->end()[-1];
          cv::Point2d diff = computeXYDiff(currPoint.point, lastInserted.point);
          if(diff.x <= max_pixel_shift_x_ && diff.y <= max_pixel_shift_y_){
              nearestNeighbor = true;
              insertPointToSequence(*(*seq), currPoint);    
              pGenSequence.erase(seq);
              break;
          }else{
              nearestNeighbor = false;
          }
      }
      if(nearestNeighbor == false){
        noNN.push_back(currPoint);
        }
    }
    expandedSearch(noNN, pGenSequence);
}

cv::Point2d uvdar::alternativeHT::computeXYDiff(const cv::Point2d first, const cv::Point2d second){
    
    cv::Point2d difference; 
    difference.x = std::abs(first.x - second.x);
    difference.y = std::abs(first.y - second.y);
    return difference;
}

void alternativeHT::insertPointToSequence(std::vector<PointState> & sequence, const PointState signal){
        sequence.push_back(signal);            
        if(sequence.size() > (originalSequences_[0].size()*10)){
            sequence.erase(sequence.begin());
        }
}


void uvdar::alternativeHT::expandedSearch(std::vector<PointState> & noNNCurrentFrame, std::vector<seqPointer> sequencesNoInsert){
    
    std::scoped_lock lock(mutex_generatedSequences_);
    auto timeI = ros::Time::now();
    std::vector<SeqWithTrajectory> sequences;
    if((int)sequencesNoInsert.size() != 0 ){
        for(auto seq : sequencesNoInsert){
            SeqWithTrajectory seqTrajectory;
            seqTrajectory.seq = seq;
            
            int polynomOrder = 2;
            // if the sequence is not long do only a line estimate  
            if(seqTrajectory.seq->size() < 10){
                polynomOrder = 1; 
            }
            HelpFunctions::selectPointsForRegressionAndDoRegression(seqTrajectory, polynomOrder);
            
            calculatePredictionTriangle(seqTrajectory, timeI);

            sequences.push_back(seqTrajectory);
        }
    }


    for(int k = 0; k < (int)sequences.size(); ++k){

        // TODO: eventually not needed!

        if(!checkSequenceValidityWithNewInsert(sequences[k].seq)){
            continue;
        }

        bool coffAllZero = false;
        if(!checkCoeffValidity(sequences[k])){
            coffAllZero = true;
        }


        cv::Point2d firstPoint = sequences[k].seq->end()[-1].firstEdgeTri;
        cv::Point2d secondPoint = sequences[k].seq->end()[-1].secEdgeTri;
        cv::Point2d initialPoint = sequences[k].seq->end()[-1].debug_gp;

        if(noNNCurrentFrame.size() != 0){

            if(trajectory_logfile_){
                logFile_.open(filename_, std::ofstream::out | std::ofstream::app); 
                logFile_ << "# ";
                for(int i = 0; i < (int)sequences[k].seq->size(); ++i){  
                    logFile_ << std::to_string(sequences[k].seq->at(i).point.x) << "," << std::to_string(sequences[k].seq->at(i).point.y) << "," << std::to_string(sequences[k].seq->at(i).insertTime.toSec())  <<" ";    
                }
                logFile_ << "# " << sequences[k].seq->end()[-1].predictedNextPoint.x << "," << sequences[k].seq->end()[-1].predictedNextPoint.y <<  "," << timeI.toSec() << " ";
                logFile_ << " # " << sequences[k].seq->end()[-1].lengthToPredict << " ";
                logFile_ << " # " <<  firstPoint.x << "," << firstPoint.y << " # " << secondPoint.x << "," << secondPoint.y << " "; 
                logFile_ << " # ";
                for(int i = 0; i < (int)sequences[k].xCoeff.size(); ++i){
                    logFile_ << sequences[k].xCoeff[i] << " "; 
                }
                logFile_ << "# ";
                for(int i = 0; i < (int)sequences[k].yCoeff.size(); ++i){
                    logFile_ << sequences[k].yCoeff[i] << " "; 
                }
                logFile_ << "# ";
            }

            for(int i = 0; i < noNNCurrentFrame.size(); ++i){
                if(trajectory_logfile_) logFile_ << noNNCurrentFrame[i].point.x  << "," << noNNCurrentFrame[i].point.y << " ";
                if(!coffAllZero){
                    if(HelpFunctions::isInside(firstPoint, secondPoint, initialPoint, noNNCurrentFrame[i].point)){
                        insertPointToSequence(*(sequences[k].seq), noNNCurrentFrame[i]);
                        noNNCurrentFrame.erase(noNNCurrentFrame.begin()+i);
                        sequences.erase(sequences.begin()+k);
                        break;
                    }
                }
                cv::Point2d diff = computeXYDiff(sequences[k].seq->end()[-1].point, noNNCurrentFrame[i].point);
                if(diff.x <= max_pixel_shift_x_+2 && diff.y <= max_pixel_shift_y_+2){
                    insertPointToSequence(*(sequences[k].seq), noNNCurrentFrame[i]);
                    noNNCurrentFrame.erase(noNNCurrentFrame.begin()+i);
                    sequences.erase(sequences.begin()+k);
                    break;
                }

            }
            if(trajectory_logfile_){
                logFile_ << " #\n";
                logFile_.close();
            }
        }
    }


    // insert VP to sequnces were no point was inserted
    if(sequences.size() != 0){
        
        for(auto seq : sequences){
            if(seq.seq->size() == 0){
                continue;
            }
            PointState pVirtual;
            pVirtual.insertTime = ros::Time::now();
            // eventually helpfull
            // if(seq.seq->end()[-1].predictedNextPoint.x != 0 && seq.seq->end()[-1].predictedNextPoint.y != 0 && seq.seq->end()[-1].lengthToPredict >= 1.0){
                
            //     int x = (int)seq.seq->end()[-1].predictedNextPoint.x;
            //     int y = (int)seq.seq->end()[-1].predictedNextPoint.y;
            //     cv::Point2d diff = computeXYDiff(seq.seq->end()[-1].predictedNextPoint, seq.seq->end()[-1].point);
            //     double len = sqrt(pow(diff.x,2) + pow(diff.y,2));
            //     if(len < 2.5){
            //         pVirtual.point = cv::Point2d(x, y);
            //     }else{
            //         pVirtual.point = (seq.seq)->end()[-1].point;
            //     }
            // }else{
            pVirtual.point = (seq.seq)->end()[-1].point;
            // }
            pVirtual.ledState = false;
            insertPointToSequence(*(seq.seq), pVirtual);
        }
                // std::cout << "1\n";  

    }


    // if still points are not inserted start new sequence
    if(noNNCurrentFrame.size() != 0){

        for(auto point : noNNCurrentFrame){
            std::vector<PointState> vect;
            vect.push_back(point);
            generatedSequences_.push_back(vect);
            // std::cout << " Point new seq: " << point.point.x << " " << point.point.y <<" "  <<point.insertTime.toSec() << "\n";
            for(auto k : generatedSequences_){
                auto l = k.end()[-1];
                std::cout << l.predictedNextPoint.x <<"," << l.predictedNextPoint.y << ";  " << l.point.x << ", " << l.point.y << "\n";  
                std::cout << std::endl;
            }
        }

    }
}

bool uvdar::alternativeHT::checkSequenceValidityWithNewInsert(const seqPointer & seq){

    if(seq->size() > 1){
        // if the last two led states were on - no "on" point expected -> no NN search necessary
       if(seq->end()[-1].ledState == true && seq->end()[-2].ledState == true){
            return false;
        }
    }
    return true; 
}

// if all coefficients are zero return false
bool uvdar::alternativeHT::checkCoeffValidity(const SeqWithTrajectory & trajectory){


    int xCount = 0, yCount = 0;
    for(auto coff : trajectory.xCoeff){
        if(coff == 0.0 ){
            xCount++;
        }
    }
    for(auto coff : trajectory.yCoeff){
        if(coff == 0.0 ){
            yCount++;
        }
    }
    // if all coefficients are zero 
    if(yCount == (int)trajectory.yCoeff.size() && xCount == (int)trajectory.xCoeff.size()){
        return false;
    }
    return true; 

}

void uvdar::alternativeHT::calculatePredictionTriangle(SeqWithTrajectory & path, const ros::Time insertTime){

    double xPredict = 0; 
    double yPredict = 0;

    double predictionTime = insertTime.toSec() + 0.3; 

    for(int i = 0; i < (int)path.xCoeff.size(); ++i){
        xPredict += path.xCoeff[i]*pow(predictionTime, i); 
    }
    for(int i = 0; i < (int)path.yCoeff.size(); ++i){
        yPredict += path.yCoeff[i]*pow(predictionTime, i);
    }
    cv::Point2d predictedPoint = cv::Point2d(xPredict, yPredict);

    cv::Point2d groundPointTriangle =  path.seq->end()[-1].point;

    // for( auto it = path.seq->end(); it != path.seq->begin(); --it ){
    //     if(it->point.x != path.seq->end()[-1].point.x || it->point.y != path.seq->end()[-1].point.y){
    //         groundPointTriangle = it->point;
    //         break;
    //     }
    // }
    path.seq->end()[-1].debug_gp = groundPointTriangle; 
    predictedPoint.x = std::round(predictedPoint.x); 
    predictedPoint.y = std::round(predictedPoint.y);  
    cv::Point2d diffVect = predictedPoint - groundPointTriangle;

    path.seq->end()[-1].debug_diff = diffVect;
    
    double len =  ( sqrt(pow(diffVect.x,2) + pow(diffVect.y, 2)) );

    if(len < 2){
        diffVect = diffVect*4;
    }else if(len < 6){
        diffVect = diffVect*2; 
    }

    len = ( sqrt(pow(diffVect.x,2) + pow(diffVect.y, 2)) );

    if(len > 20){
        len = len *2/3;
    }

    std::vector<cv::Point2d> orthoVects = HelpFunctions::findOrthogonalVectorWithLength(diffVect, len);
    // construct triangle in coordinate center
    cv::Point2d firstEdgeCenter   = diffVect + orthoVects[0];
    cv::Point2d secEdgeCenter   = diffVect + orthoVects[1]; 

    //project to the last seen point
    cv::Point2d firstEdge = groundPointTriangle + firstEdgeCenter;
    cv::Point2d secEdge = groundPointTriangle + secEdgeCenter; 

    firstEdge.x = firstEdge.x;
    firstEdge.y = firstEdge.y;
    secEdge.x = secEdge.x;
    secEdge.y = secEdge.y;

    
    path.seq->end()[-1].lengthToPredict = len;
    path.seq->end()[-1].firstEdgeTri    = firstEdge;
    path.seq->end()[-1].secEdgeTri      = secEdge;
    path.seq->end()[-1].predictedNextPoint = predictedPoint;
    path.seq->end()[-1].debug_first = firstEdgeCenter;
    path.seq->end()[-1].debug_sec   = secEdgeCenter;

}

void alternativeHT::cleanPotentialBuffer(){

    std::scoped_lock lock(mutex_generatedSequences_);

    double timeMargin = 5/60;
    if(framerate_ != 0){
        timeMargin = (1/framerate_) * 6; 
    }

    for (auto it = generatedSequences_.begin(); it != generatedSequences_.end(); ++it){ 
        if (it->empty()){
            continue;
        }

        // if( it->size() >= 2){
        //     if(!it->end()[-1].ledState && !it->end()[-2].ledState && !it->end()[-3].ledState){
        //         it = generatedSequences_.erase(it);
        //         std::cout << "delete here\n"; 
        //         continue;
        //     }
        // }
        double insertTime = it->end()[-1].insertTime.toSec(); 
        double timeDiffLastInsert = std::abs(insertTime - ros::Time::now().toSec());
        // std::cout << "time diff " << timeDiffLastInsert << " time margin " << timeMargin << "\n"; 
        if (timeDiffLastInsert > timeMargin){ //&& i != generatedSequences_.end()){
            ROS_ERROR("TIME");
            it = generatedSequences_.erase(it);
            continue;
        }

    }
}

std::vector<std::pair<PointState, int>> alternativeHT::getResults(){

    std::scoped_lock lock(mutex_generatedSequences_);
    std::vector<std::pair<PointState, int>> retrievedSignals;

    for (auto sequence : generatedSequences_){
        std::vector<bool> ledStates;

        std::vector<PointState> selected;
        if ((int)sequence.size() > (int)originalSequences_[0].size()){
            int diff = (int)sequence.size() - (int)originalSequences_[0].size();
            // std::cout << "the diff" << diff << "\n";

            for(int i = diff; i < sequence.size(); ++i){
                selected.push_back(sequence[i]);
            }
        }else{
            selected = sequence;
        }
        // std::cout << "size " << selected.size() << "\n";
        for (auto point : selected){
            ledStates.push_back(point.ledState);
        }
        // std::cout << "The seq" << std::endl; 
        // for(auto k : ledStates){
        //     if(k)std::cout << "1,";
        //     else std::cout << "0,";
        // }
        // std::cout << std::endl;


        int id = findSequenceMatch(ledStates);
        retrievedSignals.push_back(std::make_pair(sequence.end()[-1], id));
    }

    return retrievedSignals;
}

int alternativeHT::findSequenceMatch(std::vector<bool> sequence){

    for ( int i = 2; i < (int)sequence.size(); i++) {
        if (sequence[i] == false && sequence[i-1] == false && sequence[i-2] == false){
            return -2;
        }
    }
    int id = matcher_->matchSignalWithCrossCorr(sequence);

    return id;
}

alternativeHT::~alternativeHT() {
    // delete generatedSequences_;
}