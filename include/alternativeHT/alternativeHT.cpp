#include "alternativeHT.h"

#include <fstream>

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
    // std::cout << " =================\n";
    
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
        if(sequence.size() > (originalSequences_[0].size()*3)){
            sequence.erase(sequence.begin());
        }
}


void uvdar::alternativeHT::expandedSearch(std::vector<PointState> & noNNCurrentFrame, std::vector<seqPointer> sequencesNoInsert){
    
    std::scoped_lock lock(mutex_generatedSequences_);

    std::vector<SeqWithTrajectory> sequences;
    if((int)sequencesNoInsert.size() != 0 ){
        for(auto seq : sequencesNoInsert){
            SeqWithTrajectory seqTrajectory;
            seqTrajectory.seq = seq;
            HelpFunctions::prepareForPolyReg(seqTrajectory, 4);
            calculatePredictionTriangle(seqTrajectory, ros::Time::now());

            sequences.push_back(seqTrajectory);
        }
    }


    for(int k = 0; k < sequences.size(); ++k){

        if(!movAvgCheckLastTwoLEDStates(sequences[k].seq)){
            continue;
        }

        if(!checkCoeffValidity(sequences[k])){
            continue;
        }


        cv::Point2d firstPoint = sequences[k].seq->end()[-1].firstEdgeTri;
        cv::Point2d secondPoint = sequences[k].seq->end()[-1].secEdgeTri;
        cv::Point2d initialPoint = sequences[k].seq->end()[-1].point;




        // ***************** write to file

        std::string filename = "../sequences/sequence_" +  std::to_string(k) +  ".txt";  
        std::ofstream MyFile(filename);
        for(int i = 0; i < sequences[k].seq->size(); ++i){
            MyFile << std::to_string(sequences[k].seq->at(i).point.x) << " " << std::to_string(sequences[k].seq->at(i).point.y) << " " << std::to_string(sequences[k].seq->at(i).insertTime.toSec())  <<"\n";
        }

        cv::Point2d diffVect = sequences[k].seq->end()[-1].predictedNextPoint - sequences[k].seq->end()[-1].point;
        
        double len = sqrt(pow(diffVect.x, 2) + pow(diffVect.y,2));

        MyFile << "#\n";
        MyFile << "Len " << len << "\n";
        MyFile << "Predicted: " << sequences[k].seq->end()[-1].predictedNextPoint.x << " " << sequences[k].seq->end()[-1].predictedNextPoint.y << "\n";
        MyFile << "Rectangle Vals " <<  firstPoint.x << " " << firstPoint.y << " " << secondPoint.x << " " << secondPoint.y << "\n"; 
 
        for(int i = 0; i < sequences[k].xCoeff.size(); ++i){
            MyFile << "X Coff " << sequences[k].xCoeff[i] << " "; 
        }
        MyFile << "\n";
        for(int i = 0; i < sequences[k].yCoeff.size(); ++i){
            MyFile << "Y Coff " << sequences[k].yCoeff[i] << " "; 
        }
        MyFile << "\n";



        if(noNNCurrentFrame.size() != 0){
        
            // predict(sequences[k].xCoeff, sequences[k].yCoeff, noNNCurrentFrame[0].insertTime.toSec());
            for(int i = 0; i < noNNCurrentFrame.size(); ++i){
                MyFile << noNNCurrentFrame[i].point.x  << " " << noNNCurrentFrame[i].point.y << "\n";
                // std::cout << " x: " << noNNCurrentFrame[i].point.x << " y: " << noNNCurrentFrame[i].point.y;
                if(HelpFunctions::isInside(firstPoint, secondPoint, initialPoint, noNNCurrentFrame[i].point)){
                    insertPointToSequence(*(sequences[k].seq), noNNCurrentFrame[i]);
                    ROS_ERROR("INSERT");
                    noNNCurrentFrame.erase(noNNCurrentFrame.begin()+i);
                    sequences.erase(sequences.begin()+k);
                }
            }
        }
            // else{
            //     MyFile << "\n";
            // }
            MyFile.close();
    }


    // insert VP to sequnces were no point was inserted
    if(sequences.size() != 0){
        for(auto seq : sequences){
            if(seq.seq->size() == 0){
                continue;
            }
            PointState pVirtual;
            pVirtual.insertTime = ros::Time::now();
            
            // if(seq->end()[-1].predictedNextPoint.x == 0 && seq->end()[-1].predictedNextPoint.y == 0){
                pVirtual.point = seq.seq->end()[-1].point;
            // }else{
                // pVirtual.point = seq->end()[-1].predictedNextPoint;
            // }

            pVirtual.ledState = false;
            insertPointToSequence(*(seq.seq), pVirtual);
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

bool uvdar::alternativeHT::movAvgCheckLastTwoLEDStates(const seqPointer & seq){

    if(seq->size() >= 2){
        // if the last two led states were on - no new inserted point is expected and the sequence can be erased from the NoInsert vector
       if(seq->end()[-1].ledState && seq->end()[-2].ledState){
            return true;
        }
    }
    return false; 
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
// 
    if(yCount == (int)trajectory.yCoeff.size() && xCount == (int)trajectory.xCoeff.size()){
        return false;
    }
    return true; 

}

void uvdar::alternativeHT::calculatePredictionTriangle(SeqWithTrajectory & path, const ros::Time insertTime){
// TODO: not tested yet

    double xPredict = 0, yPredict = 0;
    if(path.yCoeff.size() != path.xCoeff.size()){
        ROS_ERROR("[UVDAR_BP_Tim]: The Coefficients for the x and y trajectory have different sizes!"); 
    }

    double predictionTime = insertTime.toSec() + 0.1; 

    for(int i = 0; i < (int)path.xCoeff.size(); ++i){
        xPredict += path.xCoeff[i]*pow(predictionTime, i); 
        yPredict += path.yCoeff[i]*pow(predictionTime, i);
    }
    cv::Point2d predictedPoint = cv::Point2d(xPredict, yPredict);

    cv::Point2d lastInserted = path.seq->end()[-1].point;
    cv::Point2d diffVect = predictedPoint - lastInserted;

    double len = sqrt(pow(diffVect.x,2) + pow(diffVect.y, 2));
    std::vector<cv::Point2d> orthoVects = HelpFunctions::findOrthogonalVectorWithLength(diffVect, len); // TODO: THINK ABOUT USEFULL HEURISTIC!!!

    // construct triangle in coordinate center
    cv::Point2d firstEdgeCenter   = diffVect + orthoVects[0];
    cv::Point2d secEdgeCenter   = diffVect + orthoVects[1]; 

    //project to the last seen point
    cv::Point2d firstEdge = lastInserted + firstEdgeCenter;
    cv::Point2d secEdge = lastInserted + secEdgeCenter; 

    path.seq->end()[-1].firstEdgeTri    = firstEdge;
    path.seq->end()[-1].secEdgeTri      = secEdge;
    path.seq->end()[-1].predictedNextPoint = predictedPoint;

    // std::cout << "Prediction " << path.seq->end()[-1].predictedNextPoint.x <<" " << path.seq->end()[-1].predictedNextPoint.y << " First " << path.seq->end()[-1].firstEdgeTri.x << " " << path.seq->end()[-1].firstEdgeTri.y << " Second " << path.seq->end()[-1].secEdgeTri.x << " " << path.seq->end()[-1].secEdgeTri.y << "\n";
}

void alternativeHT::cleanPotentialBuffer(){

    std::scoped_lock lock(mutex_generatedSequences_);

    double timeMargin = 0;
    if(framerate_ != 0){
        timeMargin = (1/framerate_) * 5; 
    }

    timeMargin = 50;

    for (auto it = generatedSequences_.begin(); it != generatedSequences_.end(); ++it){ 
        if (it->empty()){
            continue;
        }

        if( it->size() >= 2){
            if(!it->end()[-1].ledState && !it->end()[-2].ledState && !it->end()[-3].ledState){
                it = generatedSequences_.erase(it);
                std::cout << "delete here\n"; 
                continue;
            }
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