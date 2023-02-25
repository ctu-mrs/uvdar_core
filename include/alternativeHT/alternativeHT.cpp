#include "alternativeHT.h"

#include <bits/stdc++.h>

using namespace uvdar;

alternativeHT::alternativeHT(){
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
        p.ledState = true; // every existent point is "on"
        p.insertTime = ptsMsg->stamp;
        // cv::Point2d leftUpperCorner, rightDownCorner;
        // leftUpperCorner.x = p.point.x - boundingBox_x_Size_; 
        // leftUpperCorner.y = p.point.y - boundingBox_y_Size_;
        // rightDownCorner.x = p.point.x + boundingBox_x_Size_;
        // rightDownCorner.y = p.point.y + boundingBox_y_Size_;
        // if (p.bbLeftUp.x < 0) p.bbLeftUp.x = 0;
        // if (p.bbLeftUp.y < 0) p.bbLeftUp.y = 0;
        // p.bbLeftUp = leftUpperCorner;
        // p.bbRightDown = rightDownCorner;
        currentFrame.push_back(p);
    }

// std::cout << "---------------------------------------------\n";
    findClosestPixelAndInsert(currentFrame);

    // insertVPIfNoNewPointArrived(currentFrame);
    cleanPotentialBuffer();  // TODO: NOT WORKING!!!!!
    // std::cout << " Size " << generatedSequences_.size() << std::endl;
    
    // for(auto k : generatedSequences_){
    //     for(auto l : k){
    //         if(l.ledState) std::cout << "1,";
    //         else std::cout << "0,";
    //     }
    //     std::cout << std::endl; 
    // }
    // std::cout << "\n";
  
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

    // std::cout << "pt3n" << pGenSequence.size() << std::endl;
    
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
        // std::cout << "here\n"; 
        noNN.push_back(currPoint);

//TODO: here was before the "start new sequence" stuff
            // std::vector<PointState> vect;
            // vect.push_back(currPoint);
            // generatedSequences_.push_back(vect);
        }
    }

    // std::cout << "no NN" << noNN.size() << "\n";

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
        if(sequence.size() > (originalSequences_[0].size()*2)){
            sequence.erase(sequence.begin());
        }
}


void uvdar::alternativeHT::expandedSearch(std::vector<PointState> & noNNCurrentFrame, std::vector<seqPointer> sequencesNoInsert){
    
    std::scoped_lock lock(mutex_generatedSequences_);
    std::cout << "===========================\n";
    // makes absolutely no sense to use this function!
    // movAvgCheckLastTwoLEDStates(sequencesNoInsert);
    if(noNNCurrentFrame.size() != 0){
        ros::Time insertTime = noNNCurrentFrame[0].insertTime; 
        if(sequencesNoInsert.size() != 0){
            for(auto seq = sequencesNoInsert.begin(); seq != sequencesNoInsert.end(); ++seq){

                SeqWithTrajectory seqTrajectory;
                seqTrajectory.seq = *seq;
                // std::cout << "seq";
                // for(int i = 0; i < (*seq)->size(); ++i){
                //     std::cout << (*seq)->at(i).point.x << "," << (*seq)->at(i).point.y << "~";
                // }
                // std::cout << "\n";
                if(!HelpFunctions::prepareForPolyReg(seqTrajectory, 2)){
                    continue;
                }

                if(!calculatePredictionTriangle(seqTrajectory, insertTime)){
                    continue;
                }   

                // // take the constructed triangle from the 
                cv::Point2d firstPoint = seqTrajectory.seq->end()[-1].firstEdgeTri;
                cv::Point2d secondtPoint = seqTrajectory.seq->end()[-1].secEdgeTri;
                cv::Point2d initialPoint = seqTrajectory.seq->end()[-1].point;
                std::cout << "Points: ";
                for(auto it = noNNCurrentFrame.begin(); it != noNNCurrentFrame.end(); ++it){
                    std::cout << it->point.x << ", " << it->point.y << " ~ ";  
                    if(HelpFunctions::isInside(firstPoint, secondtPoint, initialPoint, it->point)){
                        // insertPointToSequence(*(*seq), *it);
                        ROS_ERROR("HIT");
                        // it = noNNCurrentFrame.erase(it);
                        // seq = sequencesNoInsert.erase(seq);
                        continue;
                    }
                    continue;
                }std::cout << "\n";
            }
        }
    }


    // insert VP to sequnces were no point was inserted
    if(sequencesNoInsert.size() != 0){
        for(auto seq : sequencesNoInsert){
            if(seq->size() == 0){
                continue;
            }
            // SeqWithTrajectory seqTrajectory;
            // seqTrajectory.seq = *seq;
            // HelpFunctions::prepareForPolyReg(seqTrajectory, 2); 
            PointState pVirtual;
            pVirtual.insertTime = ros::Time::now();
            pVirtual.point = seq->end()[-1].point;
            pVirtual.ledState = false;
            insertPointToSequence(*seq, pVirtual);
        }
                // std::cout << "1\n";  

    }

    // std::cout << "====================The PSIZE " << sequencesNoInsert.size() << "\n"; 

    // if still points are not inserted start new sequence
    if(noNNCurrentFrame.size() != 0){

        for(auto point : noNNCurrentFrame){
            std::vector<PointState> vect;
            vect.push_back(point);
            generatedSequences_.push_back(vect);
        }

    }
}

void uvdar::alternativeHT::movAvgCheckLastTwoLEDStates(std::vector<seqPointer> sequencesNoInsert){

    for(auto it = sequencesNoInsert.begin(); it != sequencesNoInsert.end(); ++it){
        if((*it)->size() >= 2){
            // if the last two led states were on - no new inserted point is expected and the sequence can be erased from the NoInsert vector
            if((*it)->end()[-1].ledState && (*it)->end()[-2].ledState){
                it = sequencesNoInsert.erase(it);
                continue; 
            }
        }
    }
}

bool uvdar::alternativeHT::calculatePredictionTriangle(SeqWithTrajectory & path, const ros::Time insertTime){
// TODO: not tested yet

    double xPredict = 0, yPredict = 0;
    if(path.yCoeff.size() != path.xCoeff.size()){
        return false;
    }

    double predictionTime = insertTime.toSec() + 0.1; 

    for(int i = 0; i < (int)path.xCoeff.size(); ++i){
        xPredict += path.xCoeff[i]*pow(predictionTime, i); 
        yPredict += path.yCoeff[i]*pow(predictionTime, i);
    }

    cv::Point2d predictedPoint = cv::Point2d(xPredict, yPredict);

    cv::Point2d lastInserted = path.seq->end()[-1].point;
    cv::Point2d diffVect = predictedPoint - lastInserted;

    if(predictedPoint.x == 0 && predictedPoint.y == 0){
        return false; 
    }
    std::cout << "Prediction " << predictedPoint.x << ", " << predictedPoint.y <<  "last inserted  " << lastInserted.x << ", " << lastInserted.y << "\n";

    double len = sqrt(pow(diffVect.x,2) + pow(diffVect.y, 2));

    std::vector<cv::Point2d> orthoVects = HelpFunctions::findOrthogonalVectorWithLength(diffVect, 2.5); // TODO: THINK ABOUT USEFULL HEURISTIC!!!

    // construct triangle in coordinate center
    cv::Point2d firstEdgeCenter   = diffVect + orthoVects[0];
    cv::Point2d secEdgeCenter   = diffVect + orthoVects[1]; 

    //project to the last seen point
    cv::Point2d firstEdge = lastInserted + firstEdgeCenter;
    cv::Point2d secEdge = lastInserted + secEdgeCenter; 

    if(firstEdge.x < 0 || firstEdge.y < 0 || secEdge.x < 0|| secEdge.y < 0){
        // std::cout << "I'm here\n";
        return false; 
    }

    path.seq->end()[-1].firstEdgeTri    = firstEdge;
    path.seq->end()[-1].secEdgeTri      = secEdge;

    return true;

}

void alternativeHT::cleanPotentialBuffer(){

    std::scoped_lock lock(mutex_generatedSequences_);

    double timeMargin = 0;
    if(framerate_ != 0){
        timeMargin = (1/framerate_) * 6; 
    }

    timeMargin = 50;

    for (auto it = generatedSequences_.begin(); it != generatedSequences_.end(); ++it){ 
        if (it->empty()){
            continue;
        }

           
        
        
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
            for(int i = diff; diff < sequence.size(); ++diff){
                selected.push_back(sequence[i]);
            }
        }else{
            selected = sequence;
        }
        // std::cout << "size " << selected.size() << "\n";
        for (auto point : sequence){
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