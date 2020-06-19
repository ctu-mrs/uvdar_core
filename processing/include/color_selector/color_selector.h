#ifndef COLOR_PICKER_H
#define COLOR_PICKER_H
#include <opencv2/core/core.hpp>

class ColorSelector {
  public:
    static cv::Scalar rainbow(double value, double max);
    static cv::Scalar marker_color(int index, double max=14.0);
};
#endif //COLOR_PICKER_H
