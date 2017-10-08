#pragma once

#include "base/math.h"

namespace apollonia {

class World;
struct Body;

class Joint {
 public:
  friend class World;
  void PrevStep(Float dt);
  void ApplyImpulse();
  Body* a() { return a_; }
  const Body* a() const { return a_; }
  Body* b() { return b_; }
  const Body* b() const { return b_; }

 private:
  Joint(Body* a, Body* b, const Vec2& anchor);
  Body* a_;
  Body* b_;
  Vec2 anchor_;
  Vec2 local_anchor_a_;
  Vec2 local_anchor_b_;
  Vec2 ra_;
  Vec2 rb_;
  Mat22 mass_;
  Vec2 p_;
  Vec2 bias_;
};

}
