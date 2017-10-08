#include "joint.h"
#include "body.h"

namespace apollonia {

Joint::Joint(Body* a, Body* b, const Vec2& anchor)
    : a_(a), b_(b), anchor_(anchor) {
  local_anchor_a_ = a_->rotation.Transpose() * (anchor_ - a_->LocalToWorld(a_->center));
  local_anchor_b_ = b_->rotation.Transpose() * (anchor_ - b_->LocalToWorld(b_->center));
}

void Joint::PrevStep(Float dt) {
  static const Float kBiasFactor = 0.2;
  ra_ = a_->rotation * local_anchor_a_;
  rb_ = b_->rotation * local_anchor_b_;
  auto k = (a_->inv_mass + b_->inv_mass) * Mat22::I +
           a_->inv_inertia * Mat22(ra_.y*ra_.y, -ra_.y*ra_.x, -ra_.y*ra_.x, ra_.x*ra_.x) +
           b_->inv_inertia * Mat22(rb_.y*rb_.y, -rb_.y*rb_.x, -rb_.y*rb_.x, rb_.x*rb_.x);
  mass_ = k.Inv();
  bias_ = -kBiasFactor / dt * (b_->LocalToWorld(b_->center) + rb_ - a_->LocalToWorld(a_->center) - ra_);
  a_->velocity -= p_ * a_->inv_mass;
  a_->angular_velocity -= a_->inv_inertia * Cross(ra_, p_);
  b_->velocity += p_ * b_->inv_mass;
  b_->angular_velocity += b_->inv_inertia * Cross(rb_, p_);
}

void Joint::ApplyImpulse() {
  auto dv = b_->velocity + Cross(b_->angular_velocity, rb_) - a_->velocity - Cross(a_->angular_velocity, ra_);
  auto p = mass_ * (-1 * dv + bias_);

  a_->velocity -= p * a_->inv_mass;
  a_->angular_velocity -= a_->inv_inertia * Cross(ra_, p);
  b_->velocity += p * b_->inv_mass;
  b_->angular_velocity += b_->inv_inertia * Cross(rb_, p);
  
  p_ += p;
}


}