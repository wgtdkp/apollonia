#pragma once

#include <array>
#include <cassert>

namespace apollonia {

using Float = float;

struct Vec2 {
  Float& operator[](size_t idx) {
    assert(idx < 2);
    return idx == 0 ? x : y;
  }
  const Float& operator[](size_t idx) const {
    return (*const_cast<Vec2*>(this))[idx];
  }

  Float x;
  Float y;
};

class Mat22 {
 public:
  static const Mat22 I;
  Mat22(const std::array<Vec2, 2>& mat) : mat_(mat) {}
  Mat22(Float a, Float b, Float c, Float d)
    : mat_{{ {a, b}, {c, d} }} {}
  Float Det() const {
    return 1 / (mat_[0][0] * mat_[1][1] - mat_[0][1] * mat_[1][0]);
  }
  Mat22 Inv() const {
    auto det = Det();
    return {mat_[1][1] / det, -mat_[0][1] / det,
            -mat_[1][0] / det, mat_[0][0] / det};
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

static inline Vec2 operator-(const Vec2& a, const Vec2& b) {
  return {a.x - b.x, a.y - b.y};
}

static inline Vec2 operator*(const Vec2& a, Float b) {
  return {a.x * b, a.y * b};
}

static inline Vec2 operator*(Float a, const Vec2& b) {
  return b * a;
}

static inline Vec2 operator/(const Vec2& a, Float b) {
  return {a.x / b, a.y / b};
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

static inline Mat22 operator-(const Mat22& a, const Mat22& b) {
  return {a[0].x - b[0].x, a[0].y - b[0].y,
          a[1].x - b[1].x, a[1].y - b[1].y};
}

static inline Mat22 operator*(const Mat22& a, Float b) {
  return {a[0].x * b, a[0].y * b,
          a[1].x * b, a[1].y * b};
}

static inline Mat22 operator*(Float a, const Mat22& b) {
  return b * a;
}

}
