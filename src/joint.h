#pragma once

#include "apollonia.h"
#include "base/math.h"
#include "body.h"

namespace apollonia {

class World;

class Joint {
 public:
  friend class World;
  // Prev step before iteration, reduce calculation
  virtual void PreStep(Float dt) = 0;
  
  // Apply impluse to maintain constrains
  virtual void ApplyImpulse() = 0;
  
  Body& a() { return a_; }
  const Body& a() const { return a_; }
  Body& b() { return b_; }
  const Body& b() const { return b_; }
  
 protected:
  Joint(Body& a, Body& b) : a_(a), b_(b) {}
  virtual ~Joint() {}
  DISABLE_COPY_AND_ASSIGN(Joint)

 private:
  Body& a_;
  Body& b_;
};

class RevoluteJoint : public Joint {
 public:
  friend class World;
  void PreStep(Float dt) override;
  void ApplyImpulse() override;

  const Vec2& anchor() const { return anchor_; }
  Vec2 WorldAnchorA() const {
    return a().LocalToWorld(a().rotation() * local_anchor_a_ + a().centroid());
  }
  Vec2 WorldAnchorB() const {
    return b().LocalToWorld(b().rotation() * local_anchor_b_ + b().centroid());
  }

 private:
  RevoluteJoint(Body& a, Body& b, const Vec2& anchor);
  DISABLE_COPY_AND_ASSIGN(RevoluteJoint)

  // World anchor
  Vec2 anchor_;
  // Local anchor relative to centroid of body a
  Vec2 local_anchor_a_;
  // Local anchor relative to centroid of body b
  Vec2 local_anchor_b_;

  // Cached status in prev step
  // Anchor point to body a' centroid
  Vec2 ra_;
  // Anchor point to body b' centroid
  Vec2 rb_;
  // The combined mass
  Mat22 mass_;
  // Accumulated impulse
  Vec2 p_;
  // The bias for position correction
  Vec2 bias_;
};

}
