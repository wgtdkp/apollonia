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

Arbiter* World::NewArbiter(Body* a, Body* b,
                           const Arbiter::ContactList& contacts) {
  return new Arbiter(a, b, contacts);
}

void World::Step(Float dt) {
  for (size_t i = 0; i < bodies_.size(); ++i) {
    for (size_t j = i + 1; j < bodies_.size(); ++j) {
      auto arbiter = Collide(*bodies_[i], *bodies_[j]);
      if (arbiter == nullptr) {
        arbiters_.erase(ArbiterKey(bodies_[i], bodies_[j]));
        continue;
      }
      auto iter = arbiters_.find(*arbiter);
      if (iter == arbiters_.end()) {
        arbiters_[*arbiter] = arbiter;
        continue;
      }
      auto prev_arbiter = iter->second;
      if (*prev_arbiter == *arbiter) {
        prev_arbiter->UpdateContacts(*arbiter);
      } else {
        prev_arbiter->SetContacts(*arbiter);
      }
      delete arbiter;
    }
  }

  for (auto body : bodies_) {
    if (body->mass == kInf) {
      continue;
    }
    body->velocity += (gravity_ + body->force / body->mass) * dt;
    body->angularVelocity += (body->torque / body->inertia) * dt;
  }

  for (size_t i = 0; i < iterations_; ++i) {
    for (auto& kv : arbiters_) {
      kv.second->ApplyImpulse();
    }
  }
  arbiters_.clear();

  for (auto body : bodies_) {
    body->position += body->velocity * dt;
    body->rotation = Mat22(body->angularVelocity * dt) * body->rotation;
    body->force = {0, 0};
    body->torque = 0;
  }

  // TODO(wgtdkp): loop joints
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
