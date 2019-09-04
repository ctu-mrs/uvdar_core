#include "slider.h"

slider::slider(){
  values = std::vector<double>();
}

slider::slider(const slider &input){
  values = input.values;
  slider_size = input.slider_size;
}

void slider::filterInit(double init_value, int size) {
  mutex_slider.lock();
  /* values.reserve(size); */
  for (int i = 0; i < size; i++) {
    values.push_back(init_value);
  }
  mutex_slider.unlock();
}

void slider::filterPush(double distance) {
  mutex_slider.lock();
  std::rotate(values.begin(), values.begin() + 1, values.end());
  values.back() = distance;
  mutex_slider.unlock();
}

double slider::filterSlide() {
  mutex_slider.lock();
  for (int i = 0; i < (int)(values.size()); i++) {
    std::cout << values[i] << " ";
  }
  std::cout << std::endl;
  double sum = (std::accumulate(values.begin(), values.end(), 0.0) / (double)values.size());
  mutex_slider.unlock();
  return sum;
}
