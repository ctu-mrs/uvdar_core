#ifndef AMT_HELPER_FUNCTIONS_H
#define AMT_HELPER_FUNCTIONS_H

#include <memory>
#include <string>
#include <vector>

namespace uvdar_core::tracking::ami {

/**
 * @brief Common params shared by AMI components.
 */
struct DefaultParams {
    std::string sequence_file;
    bool debug;
    bool minimal_output;

    /**
     * @brief Shared parameter group for blink tracking helpers.
     * @param sequence_file Path to a file with sequence definitions.
     * @param debug Enable verbose logging.
     * @param minimal_output Keep output compact.
     */
    DefaultParams(std::string sequence_file, bool debug, bool minimal_output)
        : sequence_file(std::move(sequence_file))
        , debug(debug)
        , minimal_output(minimal_output)
    {
    }
};

/**
 * @brief Integer 2D point used by the tracker internals.
 */
struct Point2D {
    int x;
    int y;

    /**
     * @brief Default constructor initializes to (0,0).
     */
    Point2D() = default;
    /**
     * @brief Construct a point from coordinates.
     * @param i_x X coordinate.
     * @param i_y Y coordinate.
     */
    Point2D(int i_x, int i_y)
        : x(i_x)
        , y(i_y)
    {
    }

    /**
     * @brief Subtract another point.
     */
    Point2D operator-(const Point2D& other) const
    {
        return Point2D(this->x - other.x, this->y - other.y);
    }
    /**
     * @brief Add another point.
     */
    Point2D operator+(const Point2D& other) const
    {
        return Point2D(this->x + other.x, this->y + other.y);
    }
};

/**
 * @brief Lightweight stamped list of points consumed by AMI.
 */
struct ImagePointsWithFloatStamped {
    double stamp = 0.0;
    uint16_t img_height = 0;
    uint16_t img_width = 0;
    std::vector<Point2D> points;
};

} // namespace uvdar_core::tracking::ami

#endif // AMT_HELPER_FUNCTIONS_H
