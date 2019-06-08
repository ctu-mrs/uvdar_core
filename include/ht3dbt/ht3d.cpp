#ifndef HT3D_H
#define HT3D_H
#include "ht3d.h"
#include "opencv2/highgui/highgui.hpp"
/* #include "opencv2/imgproc/imgproc.hpp" */
/* #define maxPixelShift 1 */
/* #define framerate 70 */
/* #define weightFactor 0 */
#define weightFactor 1

/* #define timeProfiling false */

#define index2d(X, Y) (imRes.width * (Y) + (X))
#define index3d(X, Y, Z) (imArea * (Z) + imRes.width * (Y) + (X))

double cot(double input) {
  return cos(input) / sin(input);
}

double acot(double input) {
  if (input > 0) {
    return (M_PI / 2.0) - atan(input);
  } else if (input < 0) {
    return -(M_PI / 2.0) + atan(input);
  } else {
    throw std::invalid_argument("Invalid input value; Cannot do arccot of zero");
  }
}

double HT3DBlinkerTracker::mod2(double a, double n) {  // modulo without sign
  return a - (floor(a / n) * n);
}

double HT3DBlinkerTracker::angDiff(double a, double b) {
  double diff = a - b;
  diff        = mod2(diff + CV_PI, 2 * CV_PI) - CV_PI;
  /* std::cout << "angDiff: " << a <<std::endl << b <<std::endl <<  diff << std::endl; */
  return diff;
}

double HT3DBlinkerTracker::angMeanXY(std::vector< cv::Point > input) {
  cv::Point sum       = std::accumulate(input.begin(), input.end(), cv::Point(0.0, 0.0));
  cv::Point mean      = (cv::Point2d)sum / (double)(input.size());
  double    meanAngle = atan2(mean.y, mean.x);
  return meanAngle;
}

HT3DBlinkerTracker::HT3DBlinkerTracker(int i_memSteps,
    int i_pitchSteps,
    int i_yawSteps,
    int i_maxPixelShift, cv::Size i_imRes, int i_nullifyRadius, int i_reasonableRadius,
                                       double i_framerate) {
  memSteps      = i_memSteps;
  pitchSteps    = i_pitchSteps;
  yawSteps      = i_yawSteps;
  framerate     = i_framerate;
  maxPixelShift = i_maxPixelShift;
  frameScale    = round(3 * framerate / 10);
  maskWidth     = 1 + 2 * maxPixelShift * (frameScale - 1);
  if ((memSteps * memSteps * memSteps * memSteps) * 2.25 > (UINT_MAX))
    bitShift = ceil(log2((memSteps * memSteps * memSteps * memSteps) * 2.25 / (UINT_MAX)));
  else
    bitShift       = 0;
  houghThresh      =
    (unsigned int)((memSteps * memSteps * memSteps * memSteps) * 0.25 * 2.25 * 0.9) >> bitShift;
  nullifyRadius    = i_nullifyRadius;
  reasonableRadius = i_reasonableRadius;
  imRes            = i_imRes;
  imArea           = imRes.width * imRes.height;
  imRect           = cv::Rect(cv::Point(0, 0), imRes);

  DEBUG    = false;
  VisDEBUG = false;

  for (int i = 0; i < memSteps; i++) {
    ptsPerLayer.push_back(0);
  }

  sinSet.clear();
  cosSet.clear();
  cotSetMax.clear();
  cotSetMin.clear();

  minPitch = acot(maxPixelShift);
  pitchDiv = ((CV_PI * 0.5) - minPitch) / pitchSteps;
  for (int i = 0; i < pitchSteps; i++) {
    pitchVals.push_back((i + 0.5) * pitchDiv + minPitch);

    if (i == pitchSteps)
      cotSetMin.push_back(0.0);
    else
      cotSetMin.push_back(cot(pitchVals[i] + (pitchDiv / 2.0)));


    if (i == 0)
      cotSetMax.push_back(cot(minPitch));
    else
      cotSetMax.push_back(cot(pitchVals[i] - (pitchDiv / 2.0)));
  }


  yawDiv = (2.0 * CV_PI) / yawSteps;
  for (int i = 0; i < yawSteps; i++) {
    yawVals.push_back((i)*yawDiv);
    sinSet.push_back(sin(yawVals[i]));
    cosSet.push_back(cos(yawVals[i]));
  }

  /* int hspacePitchSize[] = {imRes.width, imRes.height, i_pitchSteps}; */
  /* houghSpacePitch       = cv::Mat(3, hspacePitchSize, CV_16UC1, cv::Scalar(0)); */
  houghSpacePitch = new unsigned int[imArea * pitchSteps];
  resetToZero(houghSpacePitch, imArea * pitchSteps);
  /* houghSpacePitchMaxima = cv::Mat(imRes, CV_16UC1, cv::Scalar(0)); */
  houghSpacePitchMaxima = new unsigned int[imArea];
  pitchMatrix           = cv::Mat(imRes, CV_8UC1, cv::Scalar(0));
  /* int hspaceYawSize[]   = {imRes.width, imRes.height, i_yawSteps}; */
  /* houghSpaceYaw         = cv::Mat(3, hspaceYawSize, CV_16UC1, cv::Scalar(0)); */
  houghSpaceYaw = new unsigned int[imRes.width * imRes.height * yawSteps];
  resetToZero(houghSpaceYaw, imArea * yawSteps);
  /* houghSpaceYawMaxima = cv::Mat(imRes, CV_16UC1, cv::Scalar(0)); */
  houghSpaceYawMaxima = new unsigned int[imArea];
  yawMatrix           = cv::Mat(imRes, CV_8UC1, cv::Scalar(0));

  combinedMaximaMatrix = new unsigned int[imArea];
  resetToZero(combinedMaximaMatrix, imArea);
  touchedMatrix = new unsigned char[imArea];
  resetToZero(touchedMatrix, imArea);

  accumulator.push_back(std::vector< cv::Point2i >());
  accumulatorLocalCopy.push_back(std::vector< cv::Point2i >());

  generateMasks();

  initFast();

  currBatchProcessed = false;
  return;
}

template < typename T >
void HT3DBlinkerTracker::resetToZero(T *__restrict__ input, int steps) {
  for (int i = 0; i < steps; i++) {
    input[i] = (T)(0);
  }
}


void HT3DBlinkerTracker::setDebug(bool i_DEBUG, bool i_VisDEBUG) {
  DEBUG    = i_DEBUG;
  VisDEBUG = i_VisDEBUG;
}

void HT3DBlinkerTracker::updateFramerate(double input) {
  /* std::cout << "I. fr: " << input << std::endl; */
  if (input > 1.0)
    framerate = input;
}

HT3DBlinkerTracker::~HT3DBlinkerTracker() {
  return;
}


void HT3DBlinkerTracker::insertFrame(std::vector< cv::Point > newPoints) {
  mutex_accum.lock();
  {
    accumulator.insert(accumulator.begin(), newPoints);
    ptsPerLayer.insert(ptsPerLayer.begin(), (int)(newPoints.size()));
    if ((int)(accumulator.size()) > memSteps) {
      accumulator.pop_back();
      ptsPerLayer.pop_back();
    }
    /* if (DEBUG) */
    /* std::cout << "Expected matches: " << expectedMatches << std::endl; */
    currBatchProcessed = false;
  }
  mutex_accum.unlock();
  return;
}

bool HT3DBlinkerTracker::isCurrentBatchProcessed() {
  return currBatchProcessed;
}

int HT3DBlinkerTracker::getTrackerCount() {
  /* if (!currBatchProcessed) */
  /*   return 0; */
  return frequencies.size();
}
double HT3DBlinkerTracker::getFrequency(int index) {
  /* if (!currBatchProcessed) */
  /*   return -666.0; */
  /* throw std::logic_error("Cannot retrieve frequencies. Current batch is not processed yet"); */
  return frequencies[index];
}
double HT3DBlinkerTracker::getYaw(int index) {
  /* if (!currBatchProcessed) */
  /*   return -666.0; */
  /* throw std::logic_error("Cannot retrieve yaws. Current batch is not processed yet"); */
  return yawAvgs[index];
}
double HT3DBlinkerTracker::getPitch(int index) {
  /* if (!currBatchProcessed) */
  /*   return -666.0; */
  /* throw std::logic_error("Cannot retrieve yaws. Current batch is not processed yet"); */
  return pitchAvgs[index];
}

std::vector< cv::Point3d > HT3DBlinkerTracker::getResults() {
  clock_t begin, end;
  double  elapsedTime;

  /* begin = std::clock(); */
  accumulatorLocalCopy.clear();
  ptsPerLayerLocalCopy.clear();
  mutex_accum.lock();
  for (int i = 0; i < (int)(accumulator.size()); i++) {
    accumulatorLocalCopy.push_back(std::vector< cv::Point2i >());
    for (int j = 0; j < (int)(accumulator[i].size()); j++) {
      accumulatorLocalCopy[i].push_back(accumulator[i][j]);
    }
    ptsPerLayerLocalCopy.push_back(ptsPerLayer[i]);
  }
  mutex_accum.unlock();
  expectedMatches = *std::max_element(ptsPerLayerLocalCopy.begin(), ptsPerLayerLocalCopy.end()) - ptsPerLayerLocalCopy[0];
  if (DEBUG)
    std::cout << "Exp. Matches: " << expectedMatches << std::endl;

  /* end         = std::clock(); */
  /* elapsedTime = double(end - begin) / CLOCKS_PER_SEC; */
  /* std::cout << "Fetching points: " << elapsedTime << " s" << std::endl; */
  begin = std::clock();

  projectAccumulatorToHT();
  end         = std::clock();
  elapsedTime = double(end - begin) / CLOCKS_PER_SEC;
  if (DEBUG)
    std::cout << "Projecting to HT: " << elapsedTime << " s" << std::endl;
  /* begin = std::clock(); */

  nullifyKnown();
  /* end         = std::clock(); */
  /* elapsedTime = double(end - begin) / CLOCKS_PER_SEC; */
  /* std::cout << "Nullification of known: " << elapsedTime << " s" << std::endl; */

  std::vector< cv::Point > originPts = accumulatorLocalCopy[0];

  /* for(int i =0; i<originPts.size();i++) { */
  /*   for(int j = 0; j<originPts.size();j++) { */
  /*     if (i == j) */
  /*       continue; */

  /*     if (cv::norm(originPts[i] - originPts[j]) < reasonableRadius){ */
  /*       originPts[i] = (originPts[i]+originPts[j])/2.0; */
  /*       originPts.erase(originPts.begin() + j); */
  /*       j--; */
  /*     } */

  /*   } */

  /* } */


  begin                                 = std::clock();
  std::vector< cv::Point > houghOrigins = findHoughPeaks(combinedMaximaMatrix, expectedMatches);
  end                                   = std::clock();
  elapsedTime                           = double(end - begin) / CLOCKS_PER_SEC;
  if (DEBUG)
    std::cout << "Hough peaks search: " << elapsedTime << " s" << std::endl;
  begin = std::clock();
  originPts.insert(originPts.end(), houghOrigins.begin(), houghOrigins.end());
  std::vector< cv::Point3d > result;

  pitchAvgs.clear();
  yawAvgs.clear();
  frequencies.clear();
  for (int i = 0; i < originPts.size(); i++) {
    double frequency, yawAvg, pitchAvg;
    frequency = retrieveFreqency(originPts[i], yawAvg, pitchAvg);
    result.push_back(cv::Point3d(originPts[i].x, originPts[i].y, frequency));
    frequencies.push_back(frequency);
    yawAvgs.push_back(yawAvg);
    pitchAvgs.push_back(pitchAvg);
  }
  end         = std::clock();
  elapsedTime = double(end - begin) / CLOCKS_PER_SEC;
  if (DEBUG)
    std::cout << "Frequency retrieval: " << elapsedTime << " s" << std::endl;
  begin              = std::clock();
  currBatchProcessed = true;
  return result;
}

void HT3DBlinkerTracker::nullifyKnown() {
  cv::Point currKnownPt;
  int       top, left, bottom, right;
  for (int i = 0; i < accumulatorLocalCopy[0].size(); i++) {
    currKnownPt = accumulatorLocalCopy[0][i];
    top         = std::max(0, currKnownPt.y - (int)(nullifyRadius));
    left        = std::max(0, currKnownPt.x - (int)(nullifyRadius));
    bottom      = std::min(imRes.height - 1, currKnownPt.y + (int)nullifyRadius);
    right       = std::min(imRes.width - 1, currKnownPt.x + (int)nullifyRadius);
    for (int x = left; x <= (right); x++) {
      for (int y = top; y <= (bottom); y++) {
        combinedMaximaMatrix[index2d(x, y)] = 0;
      }
    }
  }
  /* expectedMatches -= accumulatorLocalCopy[0].size(); */
}

void HT3DBlinkerTracker::generateMasks() {
  int     center = maskWidth / 2;
  cv::Mat radiusBox(maskWidth, maskWidth, CV_32F);
  cv::Mat yawBox(maskWidth, maskWidth, CV_32F);
  for (int x = 0; x < maskWidth; x++) {
    for (int y = 0; y < maskWidth; y++) {
      radiusBox.at< float >(y, x) = sqrt((x - center) * (x - center) + (y - center) * (y - center));
      yawBox.at< float >(y, x)    = atan2((y - center), (x - center)) + CV_PI;
    }
  }

  for (int i = 0; i < memSteps; i++) {
    pitchMasks.push_back(std::vector< cv::Point3i >());
    for (int j = 0; j < pitchSteps; j++) {
      double Rmin = (1.0 / tan(pitchVals[j] + (pitchDiv / 2))) * i;
      double Rmax = (1.0 / tan(pitchVals[j] - (pitchDiv / 2))) * i;
      for (int x = 0; x < maskWidth; x++) {
        for (int y = 0; y < maskWidth; y++) {
          if ((radiusBox.at< float >(y, x) >= Rmin - 1) && (radiusBox.at< float >(y, x) <= Rmax + 1)) {
            pitchMasks[i].push_back(cv::Point3i(x - center, y - center, j));
          }
        }
      }
    }
  }
  for (int i = 0; i < memSteps; i++) {
    yawMasks.push_back(std::vector< cv::Point3i >());
    for (int j = 0; j < yawSteps; j++) {
      yawMasks[i].push_back(cv::Point3i(0, 0, j));
      for (int x = 0; x < maskWidth; x++) {
        for (int y = 0; y < maskWidth; y++) {
          if (radiusBox.at< float >(y, x) < (i * maxPixelShift)) {               // decay
            if (fabs(angDiff(yawBox.at< float >(y, x), yawVals[j])) < yawDiv) {  // yaw steps
              yawMasks[i].push_back(cv::Point3i(x - center, y - center, j));
            }
          }
        }
      }
    }
  }
  /* std::cout << yawMasks[3] << std::endl; */
  return;
}

void HT3DBlinkerTracker::applyMasks(std::vector< std::vector< cv::Point3i > > & __restrict__ maskSet, unsigned int *__restrict__ houghSpace) {
  /* for (int i=0;i<houghSpace.size[0];i++){ */
  /*   for (int j=0;j<houghSpace.size[1];j++){ */
  /*     for (int k=0;k<houghSpace.size[2];k++){ */
  /*       houghSpace.at<uint16_t>(i,j,k) = 0; */
  /*     }}} */
  /* houghSpace = cv::Scalar(0); */
  int index;
  int x, y, z;
  for (int t = 0; t < std::min((int)(accumulatorLocalCopy.size()), memSteps); t++) {
    for (int j = 0; j < accumulatorLocalCopy[t].size(); j++) {
      for (int m = 0; m < maskSet[t].size(); m++) {
        /* std::cout << "here a" << std::endl; */
        /* std::cout << "here b: " << cv::Point(maskSet[t][m].x + accumulatorLocalCopy[t][j].x, maskSet[t][m].y + accumulatorLocalCopy[t][j].y) << std::endl; */
        /* std::cout <<  "here" << std::endl; */
        x = maskSet[t][m].x + accumulatorLocalCopy[t][j].x;
        y = maskSet[t][m].y + accumulatorLocalCopy[t][j].y;
        z = maskSet[t][m].z;
        /* if (!(imRect.contains(cv::Point(x, y)))) */
        if (x < 0)
          continue;
        if (y < 0)
          continue;
        if (x >= imRes.width)
          continue;
        if (y >= imRes.height)
          continue;

        houghSpace[index3d(x, y, z)] += weightFactor * (memSteps - t) + memSteps;
        touchedMatrix[index2d(x, y)] = 255;
      }
    }
  }
}

void HT3DBlinkerTracker::flattenTo2D(unsigned int *__restrict__ input, int thickness, unsigned int *__restrict__ outputMaxima, cv::Mat &outputIndices) {
  unsigned int tempPos;
  unsigned int tempMax;
  for (int y = 0; y < imRes.height; y++) {
    for (int x = 0; x < imRes.width; x++) {
      /* cv::Range ranges[3] = {cv::Range(x, x + 1), cv::Range(y, y + 1), cv::Range::all()}; */
      if (touchedMatrix[index2d(x, y)] == 0)
        continue;

      tempMax = 0;
      tempPos = 0;
      for (int j = 0; j < thickness; j++) {
        if (input[index3d(x, y, j)] > tempMax) {
          tempMax = input[index3d(x, y, j)];
          tempPos = j;
        }
      }

      outputMaxima[index2d(x, y)]             = tempMax;
      outputIndices.at< unsigned char >(y, x) = tempPos;
      /* if (cv::norm(tempPos)>0) */
      /*   std::cout << "Loc" << tempPos.x << tempPos.y << std::endl; */
    }
  }
}

void HT3DBlinkerTracker::cleanTouched() {
  for (int i = 0; i < imRes.height; i++) {
    for (int j = 0; j < imRes.width; j++) {
      if (touchedMatrix[index2d(j, i)] == 255) {
        for (int k = 0; k < pitchSteps; k++) {
          houghSpacePitch[index3d(j, i, k)] = 0;
        }
        for (int k = 0; k < yawSteps; k++) {
          houghSpaceYaw[index3d(j, i, k)] = 0;
        }

        /* houghSpacePitchMaxima.at< uint16_t >(i, j) = 0; */
        /* houghSpaceYawMaxima.at< uint16_t >(i, j)   = 0; */
        /* pitchMatrix.at<uint16_t>(i,j) = 0; */
        /* yawMatrix.at<uint16_t>(i,j) = 0; */
        combinedMaximaMatrix[index2d(j, i)] = 0;


        touchedMatrix[index2d(j, i)] = 0;
      }
    }
  }
}

void HT3DBlinkerTracker::projectAccumulatorToHT() {
  clock_t begin, end;
  double  elapsedTime;

  begin = std::clock();
  cleanTouched();
  end         = std::clock();
  elapsedTime = double(end - begin) / CLOCKS_PER_SEC;
  if (DEBUG)
    std::cout << "  Cleaning touched: " << elapsedTime << " s" << std::endl;
  begin = std::clock();
  applyMasks(pitchMasks, houghSpacePitch);
  end         = std::clock();
  elapsedTime = double(end - begin) / CLOCKS_PER_SEC;
  if (DEBUG)
    std::cout << "  Pitch masking: " << elapsedTime << " s" << std::endl;
  begin = std::clock();
  applyMasks(yawMasks, houghSpaceYaw);
  end         = std::clock();
  elapsedTime = double(end - begin) / CLOCKS_PER_SEC;
  if (DEBUG)
    std::cout << "  Yaw masking: " << elapsedTime << " s" << std::endl;
  begin = std::clock();
  flattenTo2D(houghSpacePitch, pitchSteps, houghSpacePitchMaxima, pitchMatrix);
  end         = std::clock();
  elapsedTime = double(end - begin) / CLOCKS_PER_SEC;
  if (DEBUG)
    std::cout << "  Pitch flattening: " << elapsedTime << " s" << std::endl;
  begin = std::clock();
  flattenTo2D(houghSpaceYaw, yawSteps, houghSpaceYawMaxima, yawMatrix);
  end         = std::clock();
  elapsedTime = double(end - begin) / CLOCKS_PER_SEC;
  if (DEBUG)
    std::cout << "  Yaw flattening: " << elapsedTime << " s" << std::endl;
  begin = std::clock();
  /* cv::Mat houghSpacePitchMaximaDS = cv::Mat(houghSpacePitchMaxima.size(), CV_16UC1); */
  /* cv::Mat houghSpaceYawMaximaDS = cv::Mat(houghSpaceYawMaxima.size(), CV_16UC1); */
  for (int i = 0; i < imRes.height; i++) {
    for (int j = 0; j < imRes.width; j++) {
      /* houghSpacePitchMaximaDS.at<uint16_t>(i,j) = houghSpacePitchMaxima.at<uint16_t>(i,j)>>8; */
      /* houghSpaceYawMaximaDS.at<uint16_t>(i,j) = houghSpaceYawMaxima.at<uint16_t>(i,j)>>8; */
      /* std::cout <<  "BS: " << bitShift << std::endl; */
      /* std::cout <<  "UINT_MAX: " << UINT_MAX << std::endl; */
      if (touchedMatrix[index2d(j, i)] == 255) {
        /* combinedMaximaMatrix[index2d(j,i)] = (houghSpaceYawMaxima[index2d(j,i)] * houghSpacePitchMaxima[index2d(j,i)]) >> bitShift; */
        combinedMaximaMatrix[index2d(j, i)] = (houghSpaceYawMaxima[index2d(j, i)] * houghSpacePitchMaxima[index2d(j, i)]);
      }
    }
  }
  end         = std::clock();
  elapsedTime = double(end - begin) / CLOCKS_PER_SEC;
  if (DEBUG)
    std::cout << "  Combining: " << elapsedTime << " s" << std::endl;
  begin = std::clock();
  /* combinedMaximaMatrix = houghSpacePitchMaximaDS.mul(houghSpaceYawMaximaDS); */
  /* if (VisDEBUG) { */
  /*   cv::Mat viewerA = cv::Mat(combinedMaximaMatrix.size(), CV_8UC1); */
  /*   cv::Mat viewerB = cv::Mat(combinedMaximaMatrix.size(), CV_8UC1); */
  /*   cv::Mat viewerC = cv::Mat(combinedMaximaMatrix.size(), CV_8UC1); */
  /*   cv::Mat viewerD = cv::Mat(combinedMaximaMatrix.size(), CV_8UC1); */
  /*   cv::normalize(houghSpacePitchMaxima, viewerA, 0, 255, cv::NORM_MINMAX, CV_8UC1); */
  /*   cv::normalize(houghSpaceYawMaxima, viewerB, 0, 255, cv::NORM_MINMAX, CV_8UC1); */
  /*   cv::normalize(combinedMaximaMatrix, viewerC, 0, 255, cv::NORM_MINMAX, CV_8UC1); */
  /*   cv::normalize(pitchMatrix, viewerD, 0, 255, cv::NORM_MINMAX, CV_8UC1); */
  /*   cv::Mat combined1, combined2, combined3; */
  /*   cv::hconcat(viewerA, viewerB, combined1); */
  /*   cv::hconcat(viewerC, viewerD, combined2); */
  /*   cv::vconcat(combined1, combined2, combined3); */

  /*   /1* imshow("maxmatPitch", viewerA); *1/ */
  /*   /1* imshow("maxmatYaw", viewerB); *1/ */
  /*   /1* imshow("maxmatComb", viewerC); *1/ */
  /*   imshow("maxmats", combined3); */
  /*   cv::waitKey(10); */
  /* } */
  return;
}

/* cv::Mat HT3DBlinkerTracker::downSample(const cv::Mat &input, cv::Mat &output,int bits){ */
/*   /1* if (input.size() != output.size()) *1/ */
/*   /1*   output = cv::Mat(input.size(),CV_16UC1); *1/ */


/* } */

std::vector< cv::Point > HT3DBlinkerTracker::findHoughPeaks(unsigned int *__restrict__ input, int peakCount) {
  /* cv::Mat                  inputCp = input.clone(); */
  std::vector< cv::Point > peaks;
  unsigned int             currMax;
  cv::Point                currMaxPos;
  for (int i = 0; i < peakCount; i++) {
    currMax = 0;
    for (int y = 0; y < imRes.height; y++) {
      for (int x = 0; x < imRes.width; x++) {
        if (touchedMatrix[index2d(x, y)] == 0)
          continue;
        if (input[index2d(x, y)] > currMax) {
          currMax    = input[index2d(x, y)];
          currMaxPos = cv::Point(x, y);
        }
      }
    }
    if (DEBUG)
      std::cout << "Bit Shift: " << bitShift << " Thresh: " << houghThresh << " Curr. Peak: " << currMax << std::endl;
    /* if ((currMax < houghThresh) || !miniFast(currMaxPos))  */
    if (!miniFast(currMaxPos)) {
      break;
    }
    if (DEBUG)
      std::cout << "Passed FAST test" << std::endl;
    peaks.push_back(currMaxPos);
    int top, left, bottom, right;
    top    = std::max(0, currMaxPos.y - (int)(nullifyRadius));
    left   = std::max(0, currMaxPos.x - (int)(nullifyRadius));
    bottom = std::min(imRes.height - 1, currMaxPos.y + (int)nullifyRadius);
    right  = std::min(imRes.width - 1, currMaxPos.x + (int)nullifyRadius);
    for (int x = left; x <= (right); x++) {
      for (int y = top; y <= (bottom); y++) {
        input[index2d(x, y)] = 0;
      }
    }
  }
  return peaks;
}


double HT3DBlinkerTracker::retrieveFreqency(cv::Point originPoint, double &avgYaw, double &avgPitch) {
  unsigned char initPitch = pitchMatrix.at< unsigned char >(originPoint);
  int                      stepCount = std::min((int)(accumulatorLocalCopy.size()), memSteps);
  double                   radExpectedMax, radExpectedMin, currPointRadius;
  std::vector< cv::Point > positivePointAccum;
  std::vector< cv::Point > positivePointAccumPitch;
  std::vector< double >    yawAccum;
  cv::Point                currPoint, currPointCentered;
  std::vector< int >       positiveCountAccum = std::vector< int >(accumulatorLocalCopy.size(), 0);
  /* double reasonableRadiusScaled; */
  for (int t = 0; t < stepCount; t++) {
    radExpectedMin        = (cotSetMin[initPitch] * t) - (reasonableRadius);  //+t*0.2
    radExpectedMax        = (cotSetMax[initPitch] * t) + (reasonableRadius);  //+t*0.2
    positiveCountAccum[t] = 0;
    for (int k = 0; k < accumulatorLocalCopy[t].size(); k++) {
      currPoint         = accumulatorLocalCopy[t][k];
      currPointCentered = currPoint - originPoint;
      currPointRadius   = cv::norm(currPointCentered);
      /* std::cout << "ExpMin: " << radExpectedMin << std::endl; */
      /* std::cout << "ExpMax: " << radExpectedMax << std::endl; */
      /* std::cout << "Current: " << currPointRadius << std::endl; */
      /* reasonableRadiusScaled = reasonableRadius + reasonableRadius*((double)(t)/(double)memSteps); */
      /* std::cout << "ranging"  << std::endl; */
      /* std::cout <<  currPointRadius << " " << radExpectedMin << " " <<radExpectedMax << std::endl; */
      if ((currPointRadius >= radExpectedMin) && (currPointRadius <= radExpectedMax)) {
        positivePointAccum.push_back(currPointCentered);
        positivePointAccumPitch.push_back(cv::Point(currPointRadius, t));
        yawAccum.push_back(atan2(currPointCentered.y, currPointCentered.x));
        positiveCountAccum[t]++;
      }
    }
  }
  if (positivePointAccum.size() == 0) {
    return -666.0;
  }

  if (DEBUG){
    std::cout << "Before culling" << std::endl;
    for (int i = 0; i < positiveCountAccum.size(); i++) {
      std::cout << positiveCountAccum[i];
    }
  std::cout << std::endl;
  }

  avgYaw                      = angMeanXY(positivePointAccum);
  std::vector< bool > correct = std::vector< bool >(positivePointAccum.size(), true);
  for (int u = 0; u < positivePointAccum.size(); u++) {
    /* std::cout << "correct: "; */
    if ((fabs(angDiff(yawAccum[u], avgYaw)) > (CV_PI / 4.0)) && (cv::norm(positivePointAccum[u]) > (reasonableRadius))) {
      /* if ((fabs(angDiff(yawAccum[u], avgYaw)) > (CV_PI / 4.0)) )  */
      /* std::cout << "Culling" << std::endl; */
      /* std::cout << "AngDiff: " <<fabs(angDiff(yawAccum[u], avgYaw)) << std::endl; */
      /* std::cout << "Norm: " <<cv::norm(positivePointAccum[u]) << std::endl; */
      correct[u] = false;
    }
    /* std::cout << correct[u]; */
  }
  /* std::cout <<  std::endl; */
  int o = 0;
  for (int u = 0; u < correct.size(); u++) {
    if (!correct[u]) {
      /* std::cout <<  "Culling n. " << u << std::endl; */
      positiveCountAccum[positivePointAccumPitch[o].y]--;
      positivePointAccum.erase(positivePointAccum.begin() + o);
      positivePointAccumPitch.erase(positivePointAccumPitch.begin() + o);
      o--;
    }
    o++;
  }
  avgYaw   = angMeanXY(positivePointAccum);
  avgPitch = angMeanXY(positivePointAccumPitch);
  /* std::cout << "avgYaw " << avgYaw * (180.0 / CV_PI) << std::endl; */
  /* std::cout << "initPitch " << (int)initPitch << ": " << pitchVals[initPitch] * (180.0 / CV_PI) << " of " << (positivePointAccumPitch.size()) << std::endl;
   */
  /* std::cout << "avgPitch " << avgPitch * (180.0 / CV_PI) << std::endl; */


  bool   state             = false;
  bool   prevState         = state;
  bool   downStep          = false;
  int    lastDownStepIndex = 0;
  bool   upStep            = false;
  int    lastUpStepIndex   = 0;
  double period;
  double maxPeriod = 0;
  double minPeriod = memSteps;
  int    cntPeriod = 0;
  int    sumPeriod = 0;
  for (int t = 0; t < accumulatorLocalCopy.size(); t++) {
    if (positiveCountAccum[t] > 0)
      state = true;
    else
      state = false;

    if (!state && prevState) {
      if (!downStep) {
        /* std::cout <<  "here" << std::endl; */
        lastDownStepIndex = t;
        downStep          = true;
      } else {
        /* std::cout <<  t << std::endl; */
        period            = (t - lastDownStepIndex);
        if (period < minPeriod)
          minPeriod = period;
        if (period > maxPeriod)
          maxPeriod = period;
        lastDownStepIndex = t;
        sumPeriod         = sumPeriod + period;
        /* std::cout << "up " << sumPeriod << std::endl; */
        cntPeriod = cntPeriod + 1;
      }
    }
    if (state && !prevState && (t > 0)) {
      if (!upStep) {
        /* std::cout <<  "here" << std::endl; */
        lastUpStepIndex = t;
        upStep          = true;
      } else {
        /* std::cout <<  t << std::endl; */
        period          = (t - lastUpStepIndex);
        if (period < minPeriod)
          minPeriod = period;
        if (period > maxPeriod)
          maxPeriod = period;
        lastUpStepIndex = t;
        sumPeriod       = sumPeriod + period;
        /* std::cout << "up " << sumPeriod << std::endl; */
        cntPeriod = cntPeriod + 1;
      }
    }
    prevState = state;
  }
  if ((maxPeriod - minPeriod) > 3){
    return -3;
  }

  double periodAvg;

  if (cntPeriod == 0){
  /* if ((cntPeriod == 0) && (accumulatorLocalCopy.size() == memSteps)) { */
    /* std::cout << "No steps recorded???" << std::endl; */
    /* if (upStep && downStep) { */
    /*   if (lastDownStepIndex > (lastUpStepIndex + 3)) { */
    /*     periodAvg = 2 * (lastDownStepIndex - lastUpStepIndex); */
    /*   } */
    /* } */
    return -1;
  } else if (cntPeriod != 0) {
    periodAvg = (double)sumPeriod / (double)cntPeriod;
  }


  if ((periodAvg * ((double)(cntPeriod+1)/2.0) ) < ( memSteps - (1.5*periodAvg))){
    return -4;
  }

  if (DEBUG){
  std::cout << "After culling" << std::endl;
  for (int i = 0; i < positiveCountAccum.size(); i++) {
    std::cout << positiveCountAccum[i];
  }
  std::cout << std::endl;
  /* std::cout << "CNT: " << cntPeriod << " SUM: " <<sumPeriod << std::endl; */
  /* std::cout << "count is " << cntPeriod << std::endl; */
  /* std::cout << framerate << std::endl; */
  std::cout << "Frequency: " << (double)framerate / periodAvg << std::endl;
  }
  return (double)framerate / periodAvg;
}

bool HT3DBlinkerTracker::miniFast(cv::Point input) {
  for (int i = 0; i < fastPoints.size(); i++) {
    if (
        (combinedMaximaMatrix[index2d(input.x, input.y)]
         - 
         combinedMaximaMatrix[index2d(input.x + fastPoints[i].x, input.y + fastPoints[i].y)])
       <
        (houghThresh / 4.0)
       )
      return false;
  }
  return true;
}

void HT3DBlinkerTracker::initFast() {
  fastPoints.clear();

  fastPoints.push_back(cv::Point(0, -3));
  fastPoints.push_back(cv::Point(0, 3));
  fastPoints.push_back(cv::Point(3, 0));
  fastPoints.push_back(cv::Point(-3, 0));

  fastPoints.push_back(cv::Point(2, -2));
  fastPoints.push_back(cv::Point(-2, 2));
  fastPoints.push_back(cv::Point(-2, -2));
  fastPoints.push_back(cv::Point(2, 2));

  fastPoints.push_back(cv::Point(-1, -3));
  fastPoints.push_back(cv::Point(1, 3));
  fastPoints.push_back(cv::Point(3, -1));
  fastPoints.push_back(cv::Point(-3, 1));

  fastPoints.push_back(cv::Point(1, -3));
  fastPoints.push_back(cv::Point(-1, 3));
  fastPoints.push_back(cv::Point(3, 1));
  fastPoints.push_back(cv::Point(-3, -1));
}

#endif  // HT3D_H
