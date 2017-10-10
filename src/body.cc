#include "body.h"
#include <algorithm>

namespace apollonia {

using std::abs;

void Body::set_mass(Float mass) {
  mass_ = mass;
  inv_mass_ = 1 / mass;
}

void Body::set_inertia(Float inertia) {
  inertia_ = inertia;
  inv_inertia_ = 1 / inertia;
}

bool Body::ShouldCollide(const Body& other) const {
  return !(mass_ == kInf && other.mass_ == kInf);
}

void Body::ApplyImpulse(const Vec2& impulse, const Vec2& r) {
  velocity_ += impulse * inv_mass_;
  angular_velocity_ += inv_inertia_ * Cross(r, impulse);
}

void Body::Integrate(const Vec2& gravity, Float dt) {
  if (mass_ == kInf) {
    return;
  }
  velocity_ += (gravity + force_ * inv_mass_) * dt;
  angular_velocity_ += (torque_ * inv_inertia_) * dt;
  position_ += velocity_ * dt;
  rotation_ = Mat22(angular_velocity_ * dt) * rotation_;
}

static Float PolygonArea(const std::vector<Vec2>& vertices) {
  Float area = 0;
  for (size_t i = 0; i < vertices.size(); ++i) {
    auto j = (i+1) % vertices.size();
    area += Cross(vertices[i], vertices[j]);
  }
  return area / 2;
}

static Vec2 PolygonCentroid(const std::vector<Vec2>& vertices) {
  Vec2 gc {0, 0};
  for (size_t i = 0; i < vertices.size(); ++i) {
    auto j = (i+1) % vertices.size();
    gc += (vertices[i] + vertices[j]) * Cross(vertices[i], vertices[j]);
  }
  return gc / 6 / PolygonArea(vertices);
}

static Float PolygonInertia(Float mass, const PolygonBody::VertexList& vertices) {
  Float acc0 = 0, acc1 = 0;
  for (size_t i = 0; i < vertices.size(); ++i) {
    auto a = vertices[i], b = vertices[(i+1)%vertices.size()];
    auto cross = abs(Cross(a, b));
    acc0 += cross * (Dot(a, a) + Dot(b, b) + Dot(a, b));
    acc1 += cross;
  }
  return mass * acc0 / 6 / acc1;
}

PolygonBody::PolygonBody(Float mass, const VertexList& vertices)
    : Body(mass), vertices_(vertices) {
  set_inertia(PolygonInertia(mass, vertices));
  set_centroid(PolygonCentroid(vertices));
}

Float PolygonBody::FindMinSeparatingAxis(size_t& idx, const PolygonBody& other) const {
  Float separation = -kInf;
  for (size_t i = 0; i < this->Count(); ++i) {
    auto va = this->LocalToWorld((*this)[i]);
    auto normal = this->EdgeAt(i).Normal();
    auto min_sep = kInf;
    for (size_t j = 0; j < other.Count(); ++j) {
      auto vb = other.LocalToWorld(other[j]);
      min_sep = std::min(min_sep, Dot(vb - va, normal));
    }
    if (min_sep > separation) {
      separation = min_sep;
      idx = i;
    }
  }
  return separation;
}

}
