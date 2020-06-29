#include "color_selector.h"

using namespace uvdar; 

cv::Scalar ColorSelector::rainbow(double value, double max) {
    unsigned char r, g, b;

    //rainbow gradient
    double fraction = value / max;
    r = 255 * (fraction < 0.25 ? 1 : fraction > 0.5 ? 0 : 2 - fraction * 4);
    g = 255 * (fraction < 0.25 ? fraction * 4 : fraction < 0.75 ? 1 : 4 - fraction * 4);
    b = 255 * (fraction < 0.5 ? 0 : fraction < 0.75 ? fraction * 4 - 2 : 1);

    return cv::Scalar(b, g, r);
  }

cv::Scalar ColorSelector::markerColor(unsigned int index, double max){
    if (index < 7){
      cv::Scalar selected;
    //MATLAB colors
    switch(index){
      case 0: selected = cv::Scalar(0.7410,        0.4470,   0);
              break;
      case 1: selected = cv::Scalar(0.0980,   0.3250,   0.8500);
              break;
      case 2: selected = cv::Scalar(0.1250,   0.6940,   0.9290);
              break;
      case 3: selected = cv::Scalar(0.5560,   0.1840,   0.4940);
              break;
      case 4: selected = cv::Scalar(0.1880,   0.6740,   0.4660);
              break;
      case 5: selected = cv::Scalar(0.9330,   0.7450,   0.3010);
              break;
      case 6: selected = cv::Scalar(0.1840,   0.0780,   0.6350);
    }
    return 255*selected;
    }
    else
      return rainbow((double)(index - 7),max);
  }
