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
  Mat22(std::initializer_list<Vec2> init) : mat_(init) {}
  Mat22 Inv() const {
    // TODO(wgtdkp):
    return {{0, 0}, {0, 0}};
  }
  Mat22 Transpose() const {
    return {{this->[0].x, this->[1].x}, {this->[0].y, this->[1].y}};
  }

 private:
  std::array<Vec2, 2> mat_;
};

Vec2 operator+(const Vec2& a, const Vec2& b) {
  return {a.x + b.x, a.y + b.y};
}

Vec2 operator-(const Vec2& a, const Vec2& b) {
  return {a.x - b.x, a.y - b.y};
}

Vec2 operator*(const Vec2& a, Float b) {
  return {a.x * b, a.y * b};
}

Vec2 operator*(Float a, const Vec2& b) {
  return b * a;
}

Vec2 operator/(const Vec2& a, Float b) {
  return {a.x / b, a.y / b};
}

Float Dot(const Vec2& a, const Vec2& b) {
  return a.x * b.x + a.y * b.y;
}

Float Cross(const Vec2& a, const Vec2& b) {
  return a.x * b.y - a.y * b.x;
}

Mat22 operator+(const Mat22& a, const Mat22& b) {
  return {{a[0].x + b[0].x, a[0].y + b[0].y},
          {a[1].x + b[1].x, a[1].y + b[1].y}};
}

Mat22 operator-(const Mat22& a, const Mat22& b) {
  return {{a[0].x - b[0].x, a[0].y - b[0].y},
          {a[1].x - b[1].x, a[1].y - b[1].y}};
}

Mat22 operator*(const Mat22& a, Float b) {
  return {{a[0].x * b, a[0].y * b},
          {a[1].x * b, a[1].y * b}};
}

Mat22 operator*(Float a, const Mat22& b) {
  return b * a;
}

}