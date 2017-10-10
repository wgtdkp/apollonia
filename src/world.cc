#include "world.h"
#include "body.h"
#include "collision.h"
#include "joint.h"
#include <algorithm>

namespace apollonia {

World::~World() {
  Clear();
}

Body* World::NewBody(Float mass, const Vec2& position,
                     Float width, Float height) {
  return new Body(mass, position, width, height);
}

Arbiter* World::NewArbiter(Body& a, Body& b, const Vec2& normal,
                           const Arbiter::ContactList& contacts) {
  return new Arbiter(a, b, normal, contacts);
}

RevoluteJoint* World::NewRevoluteJoint(Body& a, Body& b, const Vec2& anchor) {
  return new RevoluteJoint(a, b, anchor);
}

void World::Step(Float dt) {
  // Collide
  for (size_t i = 0; i < bodies_.size(); ++i) {
    for (size_t j = i + 1; j < bodies_.size(); ++j) {
      auto& a = *bodies_[i];
      auto& b = *bodies_[j];
      if (a.mass == kInf && b.mass == kInf) {
        continue;
      }
      auto arbiter = Collide(&a, &b, dt);
      auto iter = arbiters_.find({a, b});
      if (arbiter == nullptr) {
        if (iter != arbiters_.end()) {
          delete iter->second;
          arbiters_.erase(iter);
        }
      } else if (iter == arbiters_.end()) {
        arbiters_[*arbiter] = arbiter;
      } else {
        auto& old_arbiter = iter->second;
        arbiter->AccumulateImpulse(*old_arbiter);
        delete old_arbiter;
        old_arbiter = arbiter;
      }
    }
  }

  // Velocity integration
  for (auto body : bodies_) {
    if (body->mass == kInf) {
      continue;
    }
    body->velocity += (gravity_ + body->force * body->inv_mass) * dt;
    body->angular_velocity += (body->torque * body->inv_inertia) * dt;
  }

  for (auto joint : joints_) {
    joint->PrevStep(dt);
  }

  // Apply impulse
  for (size_t i = 0; i < iterations_; ++i) {
    for (auto& kv : arbiters_) {
      kv.second->ApplyImpulse();
    }
    for (auto joint : joints_) {
      joint->ApplyImpulse();
    }
  }

  // Position integration
  for (auto body : bodies_) {
    body->position += body->velocity * dt;
    body->rotation = Mat22(body->angular_velocity * dt) * body->rotation;
    body->force = {0, 0};
    body->torque = 0;
  }
}

void World::Clear() {
  for (auto& kv : arbiters_) {
    delete kv.second;
  }
  arbiters_.clear();
  for (auto joint : joints_) {
    delete joint;
  }
  joints_.clear();
  for (auto body : bodies_) {
    delete body;
  }
  bodies_.clear();
}

};
