#pragma once
// Minimal dependency-free 3-vector and 3x3 matrix for the geometry/estimator
// (architecture §10, §9.2). The project keeps its dependency set modest (§51);
// a full linear-algebra library is not needed for two-axis gimbal kinematics.
#include <cmath>

namespace ota {
namespace geo {

struct Vec3 {
  double x{0.0}, y{0.0}, z{0.0};

  Vec3() = default;
  Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

  double norm() const { return std::sqrt(x * x + y * y + z * z); }
  Vec3 normalized() const {
    const double n = norm();
    return (n > 0.0) ? Vec3{x / n, y / n, z / n} : Vec3{0.0, 0.0, 0.0};
  }
  double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
  Vec3 cross(const Vec3& o) const {
    return Vec3{y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
  }
  Vec3 operator+(const Vec3& o) const { return Vec3{x + o.x, y + o.y, z + o.z}; }
  Vec3 operator-(const Vec3& o) const { return Vec3{x - o.x, y - o.y, z - o.z}; }
  Vec3 operator*(double s) const { return Vec3{x * s, y * s, z * s}; }
  Vec3 operator/(double s) const { return Vec3{x / s, y / s, z / s}; }
};

inline Vec3 operator*(double s, const Vec3& v) { return v * s; }

struct Mat3 {
  // Row-major: m[row][col]. Identity by default.
  double m[3][3]{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};

  Mat3() = default;
  Mat3(double a, double b, double c, double d, double e, double f,
       double g, double h, double i) {
    m[0][0] = a; m[0][1] = b; m[0][2] = c;
    m[1][0] = d; m[1][1] = e; m[1][2] = f;
    m[2][0] = g; m[2][1] = h; m[2][2] = i;
  }

  Vec3 operator*(const Vec3& v) const {
    return Vec3{m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
                m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
  }

  Mat3 operator*(const Mat3& o) const {
    Mat3 r;
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) {
        double s = 0.0;
        for (int k = 0; k < 3; ++k) s += m[i][k] * o.m[k][j];
        r.m[i][j] = s;
      }
    return r;
  }

  Mat3 transposed() const {
    Mat3 r;
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) r.m[i][j] = m[j][i];
    return r;
  }

  // Rotation about the base Z axis by angle (rad).
  static Mat3 rot_z(double a) {
    const double c = std::cos(a), s = std::sin(a);
    return Mat3{c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0};
  }
  // Rotation about the (gimbal) Y axis by angle (rad).
  static Mat3 rot_y(double a) {
    const double c = std::cos(a), s = std::sin(a);
    return Mat3{c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0, c};
  }
};

}  // namespace geo
}  // namespace ota
