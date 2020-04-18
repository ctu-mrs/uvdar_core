#ifndef SLIDER_H
#define SLIDER_H
#include <mutex>
#include <vector>
#include <iostream>
#include <numeric>
#include <algorithm>

class slider{
  public:
  slider();
  slider(const slider &input);
  void filterInit(double init_value, int i_slider_size);
  void filterPush(double i_value);
  double filterSlide();
  
  private:
  std::vector<double> values;
  int slider_size;
  std::mutex mutex_slider;
};

#endif //SLIDER_H
