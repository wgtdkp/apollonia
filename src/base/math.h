#pragma once

#include <array>
#include <cassert>
#include <cmath>
#include <limits>

namespace apollonia {

using Float = float;
using std::cos;
using std::sin;
using std::acos;
using std::asin;
static const Float kPi = acos(-1);
static const Float kInf = std::numeric_limits<Float>::infinity();

struct Vec2;
struct Mat22;

static inline Vec2 operator+(const Vec2& a, const Vec2& b);
static inline void operator+=(Vec2& a, const Vec2& b);
static inline Vec2 operator-(const Vec2& a, const Vec2& b);
static inline void operator-=(Vec2& a, const Vec2& b);
static inline Vec2 operator*(const Vec2& a, Float b);
static inline Vec2 operator*(Float a, const Vec2& b);
static inline void operator*=(Vec2& a, Float b);
static inline Vec2 operator/(const Vec2& a, Float b);
static inline void operator/=(Vec2& a, Float b);
static inline Float Dot(const Vec2& a, const Vec2& b);
static inline Float Cross(const Vec2& a, const Vec2& b);

static inline Mat22 operator+(const Mat22& a, const Mat22& b);
static inline void operator+=(Mat22& a, const Mat22& b);
static inline Mat22 operator+(const Mat22& a, Float b);
static inline Mat22 operator+(Float b, const Mat22& a);
static inline void operator+=(Mat22& a, Float b);
static inline Mat22 operator-(const Mat22& a, const Mat22& b);
static inline void operator-=(Mat22& a, const Mat22& b);
static inline Mat22 operator-(const Mat22& a, Float b);
static inline Mat22 operator-(Float b, const Mat22& a);
static inline void operator-=(Mat22& a, Float b);
static inline Mat22 operator*(const Mat22& a, Float b);
static inline Mat22 operator*(Float a, const Mat22& b);
static inline void operator*=(Mat22& a, Float b);
static inline Vec2 operator*(const Vec2& a, const Mat22& b);
static inline void operator*=(Vec2& a, const Mat22& b);

struct Vec2 {
  Float x;
  Float y;

  Vec2(Float x, Float y) : x(x), y(y) {}
  Float& operator[](size_t idx) {
    assert(idx < 2);
    return idx == 0 ? x : y;
  }
  const Float& operator[](size_t idx) const {
    return (*const_cast<Vec2*>(this))[idx];
  }
  Float Magnitude() const {
    return sqrt(x * x + y * y);
  }
  // Return the normal that 'v x n' points into paper
  Vec2 Normal() const {
    return Vec2(y, -x) / Magnitude();
  }
  Vec2 Normalized() const {
    return *this / Magnitude();
  }
};

struct Mat22 {
 public:
  static const Mat22 I;
  Mat22(const std::array<Vec2, 2>& mat) : mat_(mat) {}
  Mat22(Float a, Float b, Float c, Float d)
    : mat_{{ {a, b}, {c, d} }} {}
  Mat22(Float theta)
    : mat_{{ {cos(theta), -sin(theta)}, {sin(theta), cos(theta)} }} {}
  Float Det() const {
    return 1 / (mat_[0][0] * mat_[1][1] - mat_[0][1] * mat_[1][0]);
  }
  Mat22 Inv() const {
    auto det = Det();
    return (1 / det) * (*this);
  }
  Mat22 Transpose() const {
    auto ans = mat_;
    std::swap(ans[0][1], ans[1][0]);
    return ans;
  }
  Vec2& operator[](size_t idx) {
    return mat_[idx];
  }
  const Vec2& operator[](size_t idx) const {
    return (*const_cast<Mat22*>(this))[idx];
  }

 private:
  std::array<Vec2, 2> mat_;
};


static inline Vec2 operator+(const Vec2& a, const Vec2& b) {
  return {a.x + b.x, a.y + b.y};
}

static inline void operator+=(Vec2& a, const Vec2& b) {
  a = a + b;
}

static inline Vec2 operator-(const Vec2& a, const Vec2& b) {
  return {a.x - b.x, a.y - b.y};
}

static inline void operator-=(Vec2& a, const Vec2& b) {
  a = a - b;
}

static inline Vec2 operator*(const Vec2& a, Float b) {
  return {a.x * b, a.y * b};
}

static inline Vec2 operator*(Float a, const Vec2& b) {
  return b * a;
}

static inline void operator*=(Vec2& a, Float b) {
  a = a * b;
}

static inline Vec2 operator/(const Vec2& a, Float b) {
  return {a.x / b, a.y / b};
}

static inline void operator/=(Vec2& a, Float b) {
  a = a / b;
}

static inline Float Dot(const Vec2& a, const Vec2& b) {
  return a.x * b.x + a.y * b.y;
}

static inline Float Cross(const Vec2& a, const Vec2& b) {
  return a.x * b.y - a.y * b.x;
}

static inline Mat22 operator+(const Mat22& a, const Mat22& b) {
  return {a[0].x + b[0].x, a[0].y + b[0].y,
          a[1].x + b[1].x, a[1].y + b[1].y};
}

static inline void operator+=(Mat22& a, const Mat22& b) {
  a = a + b;
}

static inline Mat22 operator+(const Mat22& a, Float b) {
  return a + b * Mat22::I;
}
static inline Mat22 operator+(Float a, const Mat22& b) {
  return b + a;
}
static inline void operator+=(Mat22& a, Float b) {
  a = a + b;
}

static inline Mat22 operator-(const Mat22& a, const Mat22& b) {
  return {a[0].x - b[0].x, a[0].y - b[0].y,
          a[1].x - b[1].x, a[1].y - b[1].y};
}

static inline void operator-=(Mat22& a, const Mat22& b) {
  a = a - b;
}

static inline Mat22 operator-(const Mat22& a, Float b) {
  return a - b * Mat22::I;
}

static inline Mat22 operator-(Float a, const Mat22& b) {
  return a * Mat22::I - b;
}

static inline void operator-=(Mat22& a, Float b) {
  a = a - b;
}

static inline Mat22 operator*(const Mat22& a, Float b) {
  return {a[0].x * b, a[0].y * b,
          a[1].x * b, a[1].y * b};
}

static inline Mat22 operator*(Float a, const Mat22& b) {
  return b * a;
}

static inline void operator*=(Mat22& a, Float b) {
  a = a * b;
}

static inline Vec2 operator*(const Vec2& a, const Mat22& b) {
  return {a[0] * b[0][0] + a[1] * b[1][0],
          a[0] * b[0][1] + a[1] * b[1][1]};
}

static inline Vec2 operator*(const Mat22& a, const Vec2& b) {
  return {a[0][0] * b[0] + a[0][1] * b[1],
          a[1][0] * b[0] + a[1][1] * b[1]};
}

static inline void operator*=(Vec2& a, const Mat22& b) {
  a = a * b;
}

static inline Mat22 operator*(const Mat22& a, const Mat22& b) {
  return {a[0][0] * b[0][0] + a[0][1] * b[1][0],
          a[0][0] * b[0][1] + a[0][1] * b[1][1],
          a[1][0] * b[0][0] + a[1][1] * b[1][0],
          a[1][0] * b[0][1] + a[1][1] * b[1][1]};
}

static inline void operator*=(Mat22& a, const Mat22& b) {
  a = a * b;
}

}
