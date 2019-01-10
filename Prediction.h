#ifndef PREDICTION_H
#define PREDICTION_H
#include <cmath>

#include "Strategy.h"

extern model::Rules g_rules;

struct Point2D {
    double x {0.0};
    double z {0.0};
    Point2D() {}
    Point2D(double x, double z) : x(x), z(z) {}
    void set(double x_, double z_) { x = x_; z = z_; }
    double dist() { return sqrt(x*x + z*z); }
    Point2D normalize(double len) { return { x/len, z/len }; }
    double distTo(double x_, double z_) { return sqrt((x-x_)*(x-x_) + (z-z_)*(z-z_)); }
    double distTo(Point2D p) { return distTo(p.x, p.z); }
    Point2D operator+(Point2D p) { return { x+p.x, z+p.z }; }
    Point2D operator*(double val) { return { x*val, z*val }; }
    void operator*=(double val) { x *= val; z *= val; }
};

struct Point3D : Point2D {
    double y {0.0}; // высота
    Point3D() {}
    Point3D(double x, double z, double y) : y(y) { Point2D::set(x, z); }
    void set(double x_, double z_, double y_) { x = x_; z = z_; y = y_; }
    double dist() { return sqrt(x*x + y*y + z*z); }
    double distTo(double x_, double z_, double y_) { return sqrt((x-x_)*(x-x_) + (y-y_)*(y-y_) + (z-z_)*(z-z_)); }
    double distTo(Point3D p) { return distTo(p.x, p.z, p.y); }
    Point3D operator+(Point3D p) { return { x+p.x, z+p.z, y+p.y };}
    Point3D operator*(double val) { return { x*val, z*val, y*val }; }
    void operator*=(double val) { x *= val; z *= val; y *= val; }
};

double clamp(double x, double minValue, double maxValue);

namespace predict {

template <class T>
void move(T &e, double delta_time)
{
//    e.velocity = clamp(e.velocity, MAX_ENTITY_SPEED);
    e.x += e.velocity_x * delta_time;
    e.y += e.velocity_y * delta_time;
    e.z += e.velocity_z * delta_time;
    e.y -= g_rules.GRAVITY * delta_time * delta_time / 2;
    e.velocity_y -= g_rules.GRAVITY * delta_time;
}

class Prediction
{
public:
    Prediction();
};

}
#endif // PREDICTION_H
