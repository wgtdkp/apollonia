#pragma once

#include "base/math.h"
#include <vector>

namespace apollonia {

struct Body {
  Float mass {0};
  Vec2 position {0, 0};
  Vec2 velocity {0, 0};
  Mat22 rotation {Mat22::I};
  Float friction {0};
  Float bouncy {0};
  Float force {0};

  void set_vertices(const std::vector<Vec2>& vertices) {
    vertices_ = vertices;
  }
  size_t Count() const {
    return vertices_.size();
  }
  // Get local vertices with rotation
  Vec2 operator[](size_t idx) const {
    return vertices_[idx] * rotation;
  }
  Vec2 LocalToWorld(const Vec2& local_point) const {
    return position + local_point;
  }

 private:
  std::vector<Vec2> vertices_;
};

class PolygonBody : public Body {
 public:
  
 private:
  std::vector<Vec2> vertices_;
};

}
