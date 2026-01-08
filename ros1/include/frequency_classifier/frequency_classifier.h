#ifndef _FREQUENCY_CLASSIFIER_
#define _FREQUENCY_CLASSIFIER_
#include <vector>

namespace uvdar {

  /**
   * @brief Structure for classifying frequencies by similarity with templates
   */
  class UVDARFrequencyClassifier {
    public:

      /**
       * @brief The constructor
       *
       * @param i_frequencies A vector of frequencies used as templates for comparison
       */
      UVDARFrequencyClassifier(std::vector<double> i_frequencies);

      /**
       * @brief Retrieves the index of closest frequency to the set given in constructor
       *
       * @param i_frequency The input frequency to be comapred to templates
       *
       * @return The index of the closest template frequency or -1 in case no reasonable match was found
       */
      int findMatch(double i_frequency);

    private:
      struct FrequencyRange {
        int order_index;
        double frequency;
        double period;
        double bottom_bound;
        double top_bound;
      };

      std::vector<FrequencyRange> ranges;

  };
}

#endif //_FREQUENCY_CLASSIFIER_
