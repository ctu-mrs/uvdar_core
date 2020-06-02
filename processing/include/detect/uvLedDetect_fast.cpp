#include "uvLedDetect_fast.h"
#include <dirent.h>
#include <algorithm>
#include <iostream>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#define maxCornersPerBlock 96
#define invalidFlow -5555
#define enableBlankBG false
#define maxPassedPoints 2000
#define maxConsideredWindows 4
#define windowAvgMin 10
#define windowExtendedShell 20
#define simpleDisplay true
#define maxWindowNumber 3
#define vanishTime 2.0
#define vanishArea 40000.0
#define maxContours 30
#define maxConnectionDist 50
#define minConnectionSimilarity 10

#define index2d(X, Y) (m_imCurr->cols * (Y) + (X))

uvLedDetect_fast::uvLedDetect_fast() {

  initialized = false;
  m_first     = true;
  initFAST();
  initialized = true;
  stepInPeriod = 0;
  return;
}

void uvLedDetect_fast::addMask(cv::Mat i_mask){
  m_masks.push_back(i_mask);
}

std::vector< cv::Point2i > uvLedDetect_fast::processImage(const cv::Mat *i_imCurr,const cv::Mat *i_imView, std::vector<cv::Point2i>& sunPoints, bool i_gui, bool i_debug, int threshVal, int mask_id) {
  /* clock_t begin, end; */
  /* double  elapsedTime; */
  /* begin                             = std::clock(); */
  DEBUG                             = i_debug;
  gui                               = i_gui;
  std::vector< cv::Point2i > outvec = std::vector< cv::Point2i >();
  /* i_imCurr.copyTo(m_imCurr); */
  m_imCurr=i_imCurr;

  if (gui)
    (*i_imView).copyTo(m_imView);
  /* m_imCheck = cv::Scalar(0); */
  /* end         = std::clock(); */
  /* elapsedTime = double(end - begin) / CLOCKS_PER_SEC; */
  /* std::cout << "1: " << elapsedTime << " s" << std::endl; */

  /* begin = std::clock(); */
  if (m_first) {
    m_first = false;
    if (DEBUG)
      std::cout << "Thresh: " << threshVal << std::endl;
    m_roi     = cv::Rect(cv::Point(0, 0), m_imCurr->size());
    m_imCheck = cv::Mat(m_imCurr->size(), CV_8UC1);
    m_imCheck = cv::Scalar(0);
  }
  /* std::cout << "hey" << std::endl; */
  /* end         = std::clock(); */
  /* elapsedTime = double(end - begin) / CLOCKS_PER_SEC; */
  /* std::cout << "2: " << elapsedTime << " s" << std::endl; */

  /* begin = std::clock(); */
  clearMarks();

  /* end         = std::clock(); */
  /* elapsedTime = double(end - begin) / CLOCKS_PER_SEC; */
  /* std::cout << "3: " << elapsedTime << " s" << std::endl; */

  cv::Point peakPoint;

  /* begin = std::clock(); */
  bool          test;
  unsigned char maximumVal = 0;
  /* bool          gotOne     = false; */
  bool sunPointPotential = false;
  /* std::vector<cv::Point> sunPoints; */
  for (int j = 0; j < m_imCurr->rows; j++) {
    for (int i = 0; i < m_imCurr->cols; i++) {
      if (mask_id >= 0)
        if (m_masks[mask_id].data[index2d(i, j)] == 0)
          continue;
      if (m_imCheck.data[index2d(i, j)] == 0) {
        if (m_imCurr->data[index2d(i, j)] > threshVal) {
          int sunTestPoints = 0;
          sunPointPotential = true;
          /* gotOne = true; */
          test   = true;
          for (int m = 0; m < (int)(fastPoints.size()); m++) {
            x = i + fastPoints[m].x;
            if (x < 0) {
              test = false;
              break;
            }
            if (x >= m_roi.width) {
              test = false;
              break;
            }

            y = j + fastPoints[m].y;
            if (y < 0) {
              test = false;
              break;
            }
            if (y >= m_roi.height) {
              test = false;
              break;
            }

            if (m_imCheck.data[index2d(x,y)] == 255){
              test =false;
              break;
            }


            if ((m_imCurr->data[index2d(i, j)] - m_imCurr->data[index2d(x, y)]) < (threshVal/2)) {
              /* std::cout << "BREACH" << std::endl; */

              test = false;
              if (!sunPointPotential)
                break;
              else sunTestPoints++;
            }
            else 
              sunPointPotential = false;
            /* std::cout << (int)(m_imCurr.at< unsigned char >(j, i) - m_imCurr.at< unsigned char >(y, x)) << std::endl; */
          }
          /* std::cout << "here: " << x << ":" << y << std::endl; */
          if (test) {
            maximumVal = 0;
            for (int m = 0; m < (int)(fastInterior.size()); m++) {
            /* for (int m = 0; m < 1; m++) { */
              x = i + fastInterior[m].x;
              if (x < 0) {
                continue;
              }
              if (x >= m_roi.width) {
                continue;
              }

              y = j + fastInterior[m].y;
              if (y < 0) {
                continue;
              }
              if (y >= m_roi.height) {
                continue;
              }
              /* std::cout << "here: " << x << ":" << y << std::endl; */

              if (m_imCheck.data[index2d(x, y)] == 0) {
                if (m_imCurr->data[index2d(x, y)] > maximumVal) {
                  maximumVal  = m_imCurr->data[index2d(x, y)];
                  peakPoint.x = x;
                  peakPoint.y = y;
                }
                m_imCheck.data[index2d(x, y)] = 255;
              }
            }
            outvec.push_back(peakPoint);
          }
          else{
            if (sunPointPotential)
              if (sunTestPoints == (int)(fastPoints.size()))
                sunPoints.push_back(cv::Point(i,j));
          }

        }
      }
    }
  }
  /* end         = std::clock(); */
  /* elapsedTime = double(end - begin) / CLOCKS_PER_SEC; */
  /* std::cout << "5: " << elapsedTime << " s" << std::endl; */

  /* begin = std::clock(); */


  if (gui && (stepInPeriod == 75)) {
    for (int i = 0; i < (int)(outvec.size()); i++) {
      cv::circle(m_imView, outvec[i], 5, cv::Scalar(200));
    }
    cv::imshow("_Main", m_imView);
    stepInPeriod = 0;
    cv::waitKey(0);
  }
  stepInPeriod++;

  /* end         = std::clock(); */
  /* elapsedTime = double(end - begin) / CLOCKS_PER_SEC; */
  /* std::cout << "1: " << elapsedTime << " s" << "f: " << 1.0/elapsedTime << std::endl; */

  /* begin = std::clock(); */
  /* if ((outvec.size() == 0) && (gotOne)) */
  /*   std::cout << "@2" <<std::endl; */
  /* else if (outvec.size() == 0) */
  /*   std::cout << "@0" <<std::endl; */
  /* else */
  /*   std::cout << "@1" <<std::endl; */

  for (int i=0; i<(int)(outvec.size()); i++){
    for (int j=0; j<(int)(sunPoints.size()); j++){
      if (cv::norm(outvec[i]-sunPoints[j])<50){
        outvec.erase(outvec.begin()+i);
        i--;
        break;
      }
    }
  }

  return outvec;
}

void uvLedDetect_fast::clearMarks() {
  for (int j = 0; j < m_imCurr->rows; j++) {
    for (int i = 0; i < m_imCurr->cols; i++) {
      if (m_imCheck.at< unsigned char >(j, i) == 255) {
        m_imCheck.at< unsigned char >(j, i) = 0;
      }
    }
  }
}

bool uvLedDetect_fast::miniFAST(cv::Point input, cv::Point& maximum, unsigned char threshold) {
  /* if (m_imCheck.at< unsigned char >(input) == 255) { */
  /*   return false; */
  /* } */
  for (int i = 0; i < (int)(fastPoints.size()); i++) {
    if (m_imCurr->at< unsigned char >(input) < threshold) {
      return false;
    }
    x = input.x + fastPoints[i].x;
    if (x < 0) {
      return false;
    }
    if (x >= m_roi.width) {
      return false;
    }

    y = input.y + fastPoints[i].y;
    if (y < 0) {
      return false;
    }
    if (y >= m_roi.height) {
      return false;
    }

    /* return false; */

    /* if ((m_imCurr.at< unsigned char >(input) - m_imCurr.at< unsigned char >(y,x)) < threshold) { */
    if (m_imCurr->at< unsigned char >(input) < threshold) {
      return false;
    }
  }
  /* unsigned char maximumVal = 0; */
  /* for (int i = 0; i < fastInterior.size(); i++) { */
  /*   if (m_imCurr.at< unsigned char >(input + fastInterior[i]) > maximumVal) { */
  /*     maximumVal                                             = m_imCurr.at< unsigned char >(input); */
  /*     maximum                                                = input + fastInterior[i]; */
  /*     /1* m_imCheck.at< unsigned char >(input + fastInterior[i]) = 255; *1/ */
  /*   } */
  /* } */
  /* markOutInterior(input); */
  maximum = input;
  return true;
}


void uvLedDetect_fast::initFAST() {
  /* fastPoints.clear(); */

  /* fastPoints.push_back(cv::Point(0, -3)); */
  /* fastPoints.push_back(cv::Point(0, 3)); */
  /* fastPoints.push_back(cv::Point(3, 0)); */
  /* fastPoints.push_back(cv::Point(-3, 0)); */

  /* fastPoints.push_back(cv::Point(2, -2)); */
  /* fastPoints.push_back(cv::Point(-2, 2)); */
  /* fastPoints.push_back(cv::Point(-2, -2)); */
  /* fastPoints.push_back(cv::Point(2, 2)); */

  /* fastPoints.push_back(cv::Point(-1, -3)); */
  /* fastPoints.push_back(cv::Point(1, 3)); */
  /* fastPoints.push_back(cv::Point(3, -1)); */
  /* fastPoints.push_back(cv::Point(-3, 1)); */

  /* fastPoints.push_back(cv::Point(1, -3)); */
  /* fastPoints.push_back(cv::Point(-1, 3)); */
  /* fastPoints.push_back(cv::Point(3, 1)); */
  /* fastPoints.push_back(cv::Point(-3, -1)); */

  /* fastInterior.clear(); */

  /* /1* fastInterior.push_back(cv::Point(-1, -2)); *1/ */
  /* /1* fastInterior.push_back(cv::Point(0, -2)); *1/ */
  /* /1* fastInterior.push_back(cv::Point(1, -2)); *1/ */

  /* /1* fastInterior.push_back(cv::Point(-2, -1)); *1/ */
  /* /1* fastInterior.push_back(cv::Point(-1, -1)); *1/ */
  /* /1* fastInterior.push_back(cv::Point(0, -1)); *1/ */
  /* /1* fastInterior.push_back(cv::Point(1, -1)); *1/ */
  /* /1* fastInterior.push_back(cv::Point(2, -1)); *1/ */

  /* /1* fastInterior.push_back(cv::Point(-2, 0)); *1/ */
  /* /1* fastInterior.push_back(cv::Point(-1, 0)); *1/ */
  /* fastInterior.push_back(cv::Point(0, 0)); */
  /* fastInterior.push_back(cv::Point(1, 0)); */
  /* fastInterior.push_back(cv::Point(2, 0)); */

  /* /1* fastInterior.push_back(cv::Point(-2, 1)); *1/ */
  /* /1* fastInterior.push_back(cv::Point(-1, 1)); *1/ */
  /* fastInterior.push_back(cv::Point(0, 1)); */
  /* fastInterior.push_back(cv::Point(1, 1)); */
  /* fastInterior.push_back(cv::Point(2, 1)); */

  /* /1* fastInterior.push_back(cv::Point(-1, 2)); *1/ */
  /* fastInterior.push_back(cv::Point(0, 2)); */
  /* fastInterior.push_back(cv::Point(1, 2)); */
  fastPoints.clear();

  fastPoints.push_back(cv::Point(0, -4));
  fastPoints.push_back(cv::Point(0, 4));
  fastPoints.push_back(cv::Point(4, 0));
  fastPoints.push_back(cv::Point(-4, 0));

  fastPoints.push_back(cv::Point(3, -3));
  fastPoints.push_back(cv::Point(-3, 3));
  fastPoints.push_back(cv::Point(-3, -3));
  fastPoints.push_back(cv::Point(3, 3));

  fastPoints.push_back(cv::Point(-1, -4));
  fastPoints.push_back(cv::Point(1, 4));
  fastPoints.push_back(cv::Point(4, -1));
  fastPoints.push_back(cv::Point(-4, 1));

  fastPoints.push_back(cv::Point(1, -4));
  fastPoints.push_back(cv::Point(-1, 4));
  fastPoints.push_back(cv::Point(4, 1));
  fastPoints.push_back(cv::Point(-4, -1));

  fastPoints.push_back(cv::Point(-2, -4));
  fastPoints.push_back(cv::Point(2, 4));
  fastPoints.push_back(cv::Point(4, -2));
  fastPoints.push_back(cv::Point(-4, 2));

  fastPoints.push_back(cv::Point(2, -4));
  fastPoints.push_back(cv::Point(-2, 4));
  fastPoints.push_back(cv::Point(4, 2));
  fastPoints.push_back(cv::Point(-4, -2));

  fastInterior.clear();

  /* fastInterior.push_back(cv::Point(-1, -2)); */
  /* fastInterior.push_back(cv::Point(0, -2)); */
  /* fastInterior.push_back(cv::Point(1, -2)); */

  /* fastInterior.push_back(cv::Point(-2, -1)); */
  /* fastInterior.push_back(cv::Point(-1, -1)); */
  /* fastInterior.push_back(cv::Point(0, -1)); */
  /* fastInterior.push_back(cv::Point(1, -1)); */
  /* fastInterior.push_back(cv::Point(2, -1)); */

  /* fastInterior.push_back(cv::Point(-2, 0)); */
  /* fastInterior.push_back(cv::Point(-1, 0)); */
  fastInterior.push_back(cv::Point(0, 0));
  fastInterior.push_back(cv::Point(1, 0));
  fastInterior.push_back(cv::Point(2, 0));
  fastInterior.push_back(cv::Point(3, 0));

  /* fastInterior.push_back(cv::Point(-2, 1)); */
  /* fastInterior.push_back(cv::Point(-1, 1)); */
  fastInterior.push_back(cv::Point(0, 1));
  fastInterior.push_back(cv::Point(1, 1));
  fastInterior.push_back(cv::Point(2, 1));
  fastInterior.push_back(cv::Point(3, 1));

  fastInterior.push_back(cv::Point(0, 2));
  fastInterior.push_back(cv::Point(1, 2));
  fastInterior.push_back(cv::Point(2, 2));
  fastInterior.push_back(cv::Point(3, 2));

  /* fastInterior.push_back(cv::Point(-1, 2)); */
  fastInterior.push_back(cv::Point(0, 3));
  fastInterior.push_back(cv::Point(1, 3));
  fastInterior.push_back(cv::Point(2, 3));
}
