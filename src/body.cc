#include "body.h"

namespace apollonia {

// return negative if them overlap to each other
static Float Separation(const std::pair<Float, Float>& a,
                        const std::pair<Float, Float>& b) {
  if (a.first > b.first) {
    return Separation(b, a);
  }
  return std::max(a.first - b.second, b.first - a.second);
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

std::array<Contact, 2> Body::ContactWith(const Body& other) const {
  for (size_t i = 0; i < this->Count(); ++i) {
    for (size_t j = 0; j < other.Count(); ++j) {
      auto va1 = this->LocalToWorld((*this)[i]);
      auto va2 = this->LocalToWorld((*this)[(i+1)%this->Count()]);
      auto ea = va2 - va1;
      auto vb1 = other.LocalToWorld(other[j]);
      auto vb2 = other.LocalToWorld(other[(j+1)%other.Count()]);
      auto eb = vb2 - vb1;
      
    }
  }
}

}
