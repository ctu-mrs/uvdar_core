#ifndef COLOR_PICKER_H
#define COLOR_PICKER_H
#include <opencv2/core/core.hpp>

namespace uvdar {
/**
 * @brief Class for selecting visualization color based on an input index. These colors are set such that they ideally differ from each enough for easy recognition.
 */
class ColorSelector {
  public:

    /**
     * @brief Returns color for arbitrary float value based on sampling RGB rainbow gradient
     *
     * @param value Value for which a color is selected
     * @param max The value corresponding to the end of the spectrum (the start is 0) from which a color is selected at \p value
     *
     * @return An OpenCV RGB color
     */
    static cv::Scalar rainbow(double value, double max);

    /**
     * @brief Returns a color for an integer index
     *
     * @param index The index to which a color is attributed
     * @param max A maximum used in the rainbow method - in case that \p index is greater than 7, the output is obtained by the rainbow method with \p value = (\p index-7).
     *
     * @return An OpenCV RGB color
     */
    static cv::Scalar markerColor(unsigned int index, double max=14.0);
};
} //namespace uvdar
#endif //COLOR_PICKER_H
