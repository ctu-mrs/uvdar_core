#include <frequency_classifier/frequency_classifier.h>
#include <algorithm>
#include <iostream>

#define min_frequency 3
#define max_frequency 36.0
#define boundary_ratio 0.7

using namespace uvdar;

/* constructor //{ */
UVDARFrequencyClassifier::UVDARFrequencyClassifier(std::vector<double> i_frequencies){
  std::vector<double> frequencies = i_frequencies;
  int i=0;
  for (auto fr : frequencies){
    ranges.push_back({.order_index = i++, .frequency = fr, .period = 1.0 / fr, .bottom_bound = -1, .top_bound = -1 });
  }

  struct {
    bool operator()(FrequencyRange a, FrequencyRange b) const
    {   
      return a.period > b.period;
    }   
  } periodCompare;
  std::sort(ranges.begin(), ranges.end(), periodCompare);

  for (int i = 0; i < (int)(ranges.size()) - 1; i++) {
    ranges[i].bottom_bound = (ranges[i].period * (1.0 - boundary_ratio) + ranges[i + 1].period * boundary_ratio);
  }
  ranges.back().bottom_bound = (1.0 / max_frequency);

  ranges.front().top_bound = (1.0 / min_frequency);
  for (int i = 1; i < (int)(ranges.size()); i++) {
    ranges[i].top_bound = (ranges[i].period * boundary_ratio + ranges[i - 1].period * (1.0 - boundary_ratio));
  }

}
//}

/* findMatch() //{ */
int UVDARFrequencyClassifier::findMatch(double i_frequency) {
  double period = 1.0 / i_frequency;
  for (auto r : ranges) { 
    if ((period > r.bottom_bound) && (period < r.top_bound)) {
      return r.order_index;
    }
  }
  return -1;
}

//}
