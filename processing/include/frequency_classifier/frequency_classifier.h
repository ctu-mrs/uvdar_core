#ifndef _FREQUENCY_CLASSIFIER_
#define _FREQUENCY_CLASSIFIER_
#include <vector>

namespace uvdar {
  struct FrequencyRange {
    int order_index;
    double frequency;
    double period;
    double bottom_bound;
    double top_bound;
  };

  class UVDARFrequencyClassifier {
    public:
      UVDARFrequencyClassifier(std::vector<double> i_frequencies);
      int findMatch(double i_frequency);

    private:
      std::vector<FrequencyRange> ranges;

  };
}

#endif //_FREQUENCY_CLASSIFIER_
