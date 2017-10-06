#pragma once

#include "base/math.h"
#include <vector>

namespace apollonia {

class World;
struct Contact;

struct Body {
  friend class World;
  Float mass            {0};
  Float inertia         {0};
  // Gravity center
  Vec2  center          {0, 0};
  Vec2  position        {0, 0};
  Mat22 rotation        {Mat22::I};
  Vec2  velocity        {0, 0};
  Float angularVelocity {0};
  Vec2  force           {0, 0};
  Float torque          {0};
  Float friction        {1};
  Float bouncy          {0};

  void set_vertices(const std::vector<Vec2>& vertices) {
    vertices_ = vertices;
  }
  size_t Count() const {
    return vertices_.size();
  }
  // Get local vertices with rotation
  Vec2 operator[](size_t idx) const {
    return rotation * (vertices_[idx] - center) + center;
  }
  Vec2 EdgeAt(size_t idx) const {
    return (*this)[(idx+1)%Count()] - (*this)[idx];
  }
  // Convert local point to world
  Vec2 LocalToWorld(const Vec2& local_point) const {
    return position + local_point;
  }
  // Project the body to a line
  std::pair<Float, Float> ProjectTo(Vec2 line) const;
  Float FindMinSeparatingAxis(size_t& idx, const Body& other) const;

 private:
  Body(Float mass, const Vec2& position, const std::vector<Vec2>& vertices);
  Body(Float mass, const Vec2& position, Float width, Float height)
      : Body(mass, position, {{width/2, height/2}, {-width/2, height/2},
                              {-width/2, -height/2}, {width/2, -height/2}}) {}
  std::vector<Vec2> vertices_;
};

class PolygonBody : public Body {
 public:
  
 private:
  std::vector<Vec2> vertices_;
};

}
