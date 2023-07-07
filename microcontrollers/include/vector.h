#ifndef VECTOR_H
#define VECTOR_H

#include <cmath>

// A cartesian coordinate in cm with (0, 0) being the center of the field
struct Point {
    double x;
    double y;

    bool operator==(const Point &other) const {
        return x == other.x && y == other.y;
    }

    bool operator!=(const Point &other) const { return !(*this == other); }
};

// A vector initialised in polar form with angle in degrees and distance in cm
class Vector {
  public:
    Vector(double angle = NAN, double distance = NAN);
    static Vector fromPoint(Point point);

    double angle;
    double distance;

    double x() const;
    double y() const;

    Vector &operator=(const Vector &other);
    Vector operator+(const Vector &other) const;
    Vector operator-() const;
    Vector operator-(const Vector &other) const;
    Vector operator*(const double other) const;
    Vector operator/(const double other) const;

    bool exists() const { return !std::isnan(angle) && !std::isnan(distance); }
};

#endif
