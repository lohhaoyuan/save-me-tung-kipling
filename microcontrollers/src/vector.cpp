#include "vector.h"

#include <Arduino.h>
#include <cmath>

#include "util.h"

Vector::Vector(double angle, double distance)
    : angle(angle), distance(distance) {}

Vector Vector::fromPoint(Point point) {
    return {atan2d(point.y, point.x),
            sqrtf(point.x * point.x + point.y * point.y)};
}

double Vector::x() const { return distance * cosd(angle); }

double Vector::y() const { return distance * sind(angle); }

Vector &Vector::operator=(const Vector &other) {
    this->angle = other.angle;
    this->distance = other.distance;
    return *this;
}

Vector Vector::operator+(const Vector &other) const {
    const auto x = distance * cosd(angle) + other.distance * cosd(other.angle);
    const auto y = distance * sind(angle) + other.distance * sind(other.angle);
    return {atan2d(x, y), sqrtf(x * x + y * y)};
}

Vector Vector::operator-() const {
    return {clipAngleTo180(angle + 180), distance};
}

Vector Vector::operator-(const Vector &other) const {
    const auto x = distance * cosd(angle) - other.distance * cosd(other.angle);
    const auto y = distance * sind(angle) - other.distance * sind(other.angle);
    return {atan2d(x, y), sqrtf(x * x + y * y)};
}

Vector Vector::operator*(const double other) const {
    return {angle, distance * other};
}

Vector Vector::operator/(const double other) const {
    return {angle, distance / other};
}
