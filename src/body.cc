#include "body.h"

namespace apollonia {

using std::abs;

// Return negative if them overlap to each other
static Float Separation(const std::pair<Float, Float>& a,
                        const std::pair<Float, Float>& b) {
  if (a.first > b.first) {
    return Separation(b, a);
  }
  return std::max(a.first - b.second, b.first - a.second);
}

static Float PolygonArea(const std::vector<Vec2>& vertices) {
  Float area = 0;
  for (size_t i = 0; i < vertices.size(); ++i) {
    auto j = (i+1) % vertices.size();
    area += Cross(vertices[i], vertices[j]);
  }
  return area / 2;
}

static Vec2 PolygonGravityCenter(const std::vector<Vec2>& vertices) {
  Vec2 gc {0, 0};
  for (size_t i = 0; i < vertices.size(); ++i) {
    auto j = (i+1) % vertices.size();
    gc += (vertices[i] + vertices[j]) * Cross(vertices[i], vertices[j]);
  }
  return gc / 6 / PolygonArea(vertices);
}

static Float PolygonMomentOfInertia(Float mass,
    const std::vector<Vec2>& vertices) {
  Float acc0 = 0, acc1 = 0;
  for (size_t i = 0; i < vertices.size(); ++i) {
    auto a = vertices[i], b = vertices[(i+1)%vertices.size()];
    auto cross = abs(Cross(a, b));
    acc0 += cross * (Dot(a, a) + Dot(b, b) + Dot(a, b));
    acc1 += cross;
  }
  return mass * acc0 / 6 / acc1;
}

Body::Body(Float mass, const Vec2& position, const std::vector<Vec2>& vertices)
    : mass(mass), inertia(PolygonMomentOfInertia(mass, vertices)),
      center(PolygonGravityCenter(vertices)),
      position(position), vertices_(vertices) {
}

std::pair<Float, Float> Body::ProjectTo(Vec2 line) const {
  line = line.Normalized();
  Float mini = Dot(this->LocalToWorld((*this)[0]), line);
  Float maxi = mini;
  for (size_t i = 1; i < this->Count(); ++i) {
    auto val = Dot(this->LocalToWorld((*this)[i]), line);
    mini = std::min(mini, val);
    maxi = std::max(maxi, val);
  }
  return {mini, maxi};
}

Float Body::FindMinSeparatingAxis(size_t& idx, const Body& other) const {
  Float separation = -kInf;
  for (size_t i = 0; i < this->Count(); ++i) {
    auto normal = this->EdgeAt(i).Normal();
    auto project_a = this->ProjectTo(normal);
    auto project_b = other.ProjectTo(normal);
    auto sep = Separation(project_a, project_b);
    if (sep > separation) {
      separation = sep;
      idx = i;
    }
  }
  return separation;
}

}
