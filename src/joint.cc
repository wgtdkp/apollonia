#include "joint.h"
#include "body.h"

namespace apollonia {

RevoluteJoint::RevoluteJoint(Body& a, Body& b, const Vec2& anchor)
    : Joint(a, b), anchor_(anchor) {
  local_anchor_a_ = a.rotation.Transpose() * (anchor_ - a.LocalToWorld(a.centroid));
  local_anchor_b_ = b.rotation.Transpose() * (anchor_ - b.LocalToWorld(b.centroid));
}

void RevoluteJoint::PrevStep(Float dt) {
  static const Float kBiasFactor = 0.2;
  auto& a = this->a();
  auto& b = this->b();
  ra_ = a.rotation * local_anchor_a_;
  rb_ = b.rotation * local_anchor_b_;
  auto k = (a.inv_mass + b.inv_mass) * Mat22::I +
           a.inv_inertia * Mat22(ra_.y*ra_.y, -ra_.y*ra_.x, -ra_.y*ra_.x, ra_.x*ra_.x) +
           b.inv_inertia * Mat22(rb_.y*rb_.y, -rb_.y*rb_.x, -rb_.y*rb_.x, rb_.x*rb_.x);
  mass_ = k.Inv();
  bias_ = -kBiasFactor / dt * (b.LocalToWorld(b.centroid) + rb_ - a.LocalToWorld(a.centroid) - ra_);
  a.velocity -= p_ * a.inv_mass;
  a.angular_velocity -= a.inv_inertia * Cross(ra_, p_);
  b.velocity += p_ * b.inv_mass;
  b.angular_velocity += b.inv_inertia * Cross(rb_, p_);
}

void RevoluteJoint::ApplyImpulse() {
  auto& a = this->a();
  auto& b = this->b();
  auto dv = b.velocity + Cross(b.angular_velocity, rb_) - a.velocity - Cross(a.angular_velocity, ra_);
  auto p = mass_ * (-1 * dv + bias_);

  a.velocity -= p * a.inv_mass;
  a.angular_velocity -= a.inv_inertia * Cross(ra_, p);
  b.velocity += p * b.inv_mass;
  b.angular_velocity += b.inv_inertia * Cross(rb_, p);
  
  p_ += p;
}

}
